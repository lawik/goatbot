defmodule Goatbot.YawController do
  @moduledoc """
  Holds the direction the robot is facing, and turns it on request.

  Runs its own loop at `rate` Hz and talks to the balancer only through PubSub:
  it reads the yaw rate from the `:heading` sensor and turn requests from
  `[:drive]`, and publishes a `Goatbot.Message.Differential` on `[:yaw]` for
  the balancer to mix into the two wheels.

  ## The law

      differential = feedforward * turn - (gain * (heading - hold) + damping * (rate - turn))

  clamped to ±`limit` wheel rad/s, with the heading error wrapped to ±180° so a
  robot that has been spun round several times turns back the short way rather
  than unwinding every revolution. Positive turns the robot left. This is the
  stock `bb_nsk` balancer's heading law and gains; the wheel mixing that makes
  "positive" mean "left" is in `Goatbot.Balancer`, and differs from the stock
  one.

  `feedforward` is what a turn needs before any error builds up: turning at
  `turn` rad/s takes `turn * track / (2 * wheel radius)` of wheel-speed
  difference, 0.11 m / (2 * 0.0215 m) = 2.56 on this robot. Without it the
  robot only turned by falling behind the moving `hold`: about 9° behind at gain
  16 for 1 rad/s, which felt sluggish.

  `hold` is the heading the robot had when it was stood up. While a turn is
  requested (`BB.NSK.Message.Drive`'s `turn`, in rad/s, positive left) `hold`
  moves at that rate, and the damping term asks for the turn rate rather than for
  none. When a turn starts or stops, `hold` jumps to wherever the robot is
  pointing: a turn doesn't drag an old error along, and letting go holds *here*
  rather than finishing the turn. A turn request is only
  good for `drive_timeout`.

  ## Why it keeps its own heading

  The `:heading` sensor integrates the raw gyro, and on this robot the gyro reads
  about -0.6°/s about the vertical while standing still: 36° a minute. Holding
  that heading would turn the robot slowly round to follow it. So this learns
  the bias whenever the robot isn't balancing and isn't being turned (the rate
  within `still_rate` of the current estimate), averaged over `bias_tau`, and
  integrates the corrected rate itself.

  The estimate starts at zero, not at the first sample. A real bias is well
  inside `still_rate` of zero, so it is learned within a few `bias_tau`; the
  first sample after boot can be garbage while the IMU starts up (500°/s has
  been seen), and taken as the bias it put every later sample outside
  `still_rate` of it, so it could never be corrected. The robot then balanced
  fine and drove in circles, fighting a turn that wasn't happening.

  What bias is left, and any that changes with temperature, is absorbed by
  letting `hold` follow the heading over `tau` seconds. That's slow enough to
  hold the direction for practical purposes while it balances, but it isn't a
  compass: over minutes the direction will wander by whatever bias the
  calibration missed.
  """

  use BB.Controller,
    options_schema: [
      gain: [type: :float, required: true, doc: "Wheel rad/s per radian of heading error"],
      damping: [type: :float, required: true, doc: "Wheel rad/s per rad/s of yaw-rate error"],
      limit: [type: :float, required: true, doc: "Largest differential, in wheel rad/s"],
      feedforward: [
        type: :float,
        default: 2.56,
        doc: "Wheel rad/s of differential per rad/s of requested turn"
      ],
      tau: [
        type: :float,
        required: true,
        doc: "Seconds for the held heading to follow the measured one. 0 never follows"
      ],
      drive_timeout: [
        type: :pos_integer,
        default: 300,
        doc: "Milliseconds a turn request stays good for"
      ],
      bias_tau: [
        type: :float,
        default: 3.0,
        doc: "Seconds the gyro bias is averaged over while learning"
      ],
      still_rate: [
        type: :float,
        default: 0.05,
        doc: "Yaw rate, in rad/s, within which the robot counts as not being turned"
      ],
      rate: [type: :pos_integer, default: 50, doc: "Loop rate, in Hz"]
    ]

  alias BB.Message
  alias BB.Message.Geometry.Twist2D
  alias BB.Message.Sensor.JointState
  alias BB.NSK.Message.Drive
  alias BB.StateMachine.Transition
  alias Goatbot.Message.Differential

  @path [:yaw]

  @doc "Where the differential is published."
  def path, do: @path

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    {:ok, ground} = BB.Robot.path_to(robot.robot(), :ground)

    {:ok, _} = BB.PubSub.subscribe(robot, [:state_machine])

    {:ok, _} =
      BB.PubSub.subscribe(robot, [:sensor | ground] ++ [:heading], message_types: [JointState])

    {:ok, _} = BB.PubSub.subscribe(robot, BB.NSK.Drive.path(), message_types: [Drive])

    state =
      %{
        robot: robot,
        mode: BB.Robot.Runtime.state(robot),
        bias: 0.0,
        raw_rate: 0.0,
        sample_at: nil,
        last: nil,
        turn: 0.0,
        turn_at: nil
      }
      |> Map.merge(fresh())
      |> configure(opts)

    schedule(state)
    {:ok, state}
  end

  # Each attempt starts facing forwards, holding where it was stood up.
  defp fresh, do: %{heading: 0.0, hold: 0.0, turning?: false, differential: 0.0}

  @impl BB.Controller
  def handle_options(opts, state), do: {:ok, configure(state, opts)}

  @impl BB.Controller
  def handle_call(:inspect, _from, state), do: {:reply, state, state}

  @impl BB.Controller
  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: to}}}, state) do
    {:noreply, Map.merge(%{state | mode: to}, fresh())}
  end

  def handle_info(
        {:bb, _path, %Message{payload: %JointState{velocities: [%Twist2D{omega: omega}]}} = m},
        state
      ) do
    {:noreply, sample(state, omega, m.monotonic_time)}
  end

  def handle_info({:bb, _path, %Message{payload: %Drive{turn: turn}} = m}, state) do
    {:noreply, %{state | turn: turn, turn_at: m.monotonic_time}}
  end

  def handle_info(:tick, state) do
    schedule(state)
    now = System.monotonic_time(:nanosecond)
    dt = if state.last, do: (now - state.last) / 1_000_000_000, else: 0.0
    state = tick(%{state | last: now}, turn_request(state, now), dt)

    if state.mode == :balancing do
      {:ok, message} = Differential.new(:yaw_hold, differential: state.differential)
      BB.PubSub.publish(state.robot, @path, message)
    end

    {:noreply, state}
  end

  def handle_info(_message, state), do: {:noreply, state}

  # Every gyro sample: integrate the heading while balancing, otherwise learn
  # the bias if nothing is turning the robot.
  defp sample(state, omega, at) do
    dt = if state.sample_at, do: (at - state.sample_at) / 1_000_000_000, else: 0.0
    state = %{state | raw_rate: omega, sample_at: at}

    cond do
      state.mode == :balancing ->
        %{state | heading: state.heading + (omega - state.bias) * dt}

      abs(omega - state.bias) < state.still_rate and dt > 0.0 ->
        %{state | bias: state.bias + (omega - state.bias) * dt / (state.bias_tau + dt)}

      true ->
        state
    end
  end

  @doc """
  One tick of the loop: move the held heading for any turn, and work out the
  differential. Pure, so it can be run against recorded data.
  """
  @spec tick(map, float, float) :: map
  def tick(%{mode: :balancing} = state, turn, dt) when dt > 0.0 do
    # A turn starts and ends from wherever the robot is pointing, so neither
    # carries an old error into the new direction.
    hold =
      cond do
        turn != 0.0 and not state.turning? -> state.heading + turn * dt
        turn != 0.0 -> state.hold + turn * dt
        state.turning? -> state.heading
        true -> state.hold
      end

    hold =
      if state.tau > 0.0, do: hold + (state.heading - hold) * dt / (state.tau + dt), else: hold

    rate = state.raw_rate - state.bias

    # `Map.get` rather than `state.feedforward`, so this loads into a running
    # controller that was started before the option existed.
    feedforward = Map.get(state, :feedforward, 2.56)

    differential =
      (feedforward * turn -
         (state.gain * wrap(state.heading - hold) + state.damping * (rate - turn)))
      |> max(-state.limit)
      |> min(state.limit)

    %{state | hold: hold, turning?: turn != 0.0, differential: differential}
  end

  def tick(state, _turn, _dt), do: state

  # An angle in (-pi, pi], so an error is always the short way round.
  defp wrap(angle) do
    wrapped = :math.fmod(angle + :math.pi(), 2 * :math.pi())
    if wrapped <= 0.0, do: wrapped + :math.pi(), else: wrapped - :math.pi()
  end

  defp turn_request(%{turn_at: nil}, _now), do: 0.0

  defp turn_request(state, now) do
    if now - state.turn_at <= state.drive_timeout * 1_000_000, do: state.turn, else: 0.0
  end

  defp schedule(state), do: Process.send_after(self(), :tick, div(1000, state.rate))

  defp configure(state, opts) do
    Map.merge(state, %{
      gain: Keyword.fetch!(opts, :gain),
      damping: Keyword.fetch!(opts, :damping),
      limit: Keyword.fetch!(opts, :limit),
      # `get`, not `fetch!`: options resolved before this option existed (a
      # running controller hot-loaded with new code) don't have it.
      feedforward: Keyword.get(opts, :feedforward, 2.56),
      tau: Keyword.fetch!(opts, :tau),
      drive_timeout: Keyword.fetch!(opts, :drive_timeout),
      bias_tau: Keyword.fetch!(opts, :bias_tau),
      still_rate: Keyword.fetch!(opts, :still_rate),
      rate: Keyword.fetch!(opts, :rate)
    })
  end
end
