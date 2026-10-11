defmodule Goatbot.VelocityController do
  @moduledoc """
  Leans the robot against its own velocity, so it slows to a stop instead of
  drifting away.

  Runs its own loop at `rate` Hz, separately from the balance loop, and talks to
  it only through PubSub: it listens to the wheel commands the balancer sends,
  and publishes a `Goatbot.Message.LeanOffset` on `[:lean_offset]` for the
  balancer to add to its setpoint.

  ## Where the velocity comes from

  There are no wheel encoders, so the only signal is the wheel command. The
  wheels behave like a speed servo: torque goes with the command minus the speed
  they're already turning at. With a large `integral_gain` in the balance loop,
  the command tracks the wheel speed, so the command, filtered, *is* the robot's
  velocity in wheel rad/s:

      average  = wheel command, averaged over `filter` seconds
      offset   = -damping * average                       (clamped to ±`limit`)

  `damping` is Kv * r: radians of lean per m/s, times the wheel radius. Moving
  forward gives a negative offset, so the robot leans back and slows. Simulated
  against a DC-motor model of this robot it balanced with the setpoint 2° off and
  through a 0.3 m/s shove; on the robot, at 0.0043 (Kv 0.2), it held still for
  over a minute with peak commands around 11 rad/s.

  ### The integrated term, off by default

      velocity += average * dt          (only while |average| < `ceiling`)
      offset   -= gain * velocity

  This is the stock `bb_nsk` trim's law, and it is right for a balance loop
  without an integral, where a held command means a held acceleration. With the
  integral in place the command already means speed, and integrating it again
  overshot: on the robot it kept pushing the old way after the robot had turned
  round. `gain` defaults to 0.

  ## Driving

  `BB.NSK.Message.Drive` on `[:drive]` (`BB.NSK.Drive.command/3`, the same
  topic the phone drive pad uses) sets a target speed of `throttle *
  drive_speed`, and the loop leans against the difference instead:

      offset = -damping * (average - speed_ref / wheel_radius)

  `speed_ref` builds towards the target at `drive_accel` and falls back at the
  much quicker `drive_decel`, and the target falls back to
  zero once no drive message has arrived for `drive_timeout`. So letting go, or
  losing whoever is driving, brings the robot to a stop. The setpoint is never
  touched. In simulation it held 10 and 20 cm/s and stopped within about a
  second of letting go.

  ## What it doesn't do

  It doesn't hold a position, so after a shove it stops wherever it ends up. The
  flip side is that a setpoint that isn't the true balance point leaves a small
  steady crawl: about 8.7 cm/s per degree of setpoint error at Kv 0.2. Raising
  `damping` shrinks the crawl, but the filtered command is the balance loop's own
  output, so `kp * damping` above about 1 starts feeding that output back into
  itself.

  Everything resets on each state transition, so each attempt starts at rest.
  """

  import BB.Unit

  use BB.Controller,
    options_schema: [
      gain: [
        type: :float,
        default: 0.0,
        doc: "Radians of lean per unit of integrated wheel command. Off by default"
      ],
      filter: [
        type: :float,
        default: 1.0,
        doc: "Seconds the wheel command is averaged over"
      ],
      limit: [
        type: BB.Unit.Option.unit_type(compatible: :degree),
        default: ~u(5 degree),
        doc: "Largest lean offset it may ask for"
      ],
      ceiling: [
        type: :float,
        default: 2.0,
        doc: "Average command, in rad/s, above which it stops integrating"
      ],
      damping: [
        type: :float,
        required: true,
        doc: "Radians of lean per rad/s of filtered wheel command: Kv times the wheel radius"
      ],
      drive_speed: [
        type: :float,
        default: 0.1,
        doc: "Target speed, in m/s, at full throttle"
      ],
      drive_accel: [
        type: :float,
        default: 0.5,
        doc: "How fast, in m/s², the target speed may build up"
      ],
      drive_decel: [
        type: :float,
        default: 3.0,
        doc: "How fast, in m/s², the target speed may fall back towards zero"
      ],
      drive_timeout: [
        type: :pos_integer,
        default: 300,
        doc: "Milliseconds a drive message stays good for before the target falls to zero"
      ],
      wheel_radius: [
        type: BB.Unit.Option.unit_type(compatible: :meter),
        default: ~u(21.5 millimeter),
        doc: "Turns a target speed into wheel rad/s"
      ],
      rate: [type: :pos_integer, default: 50, doc: "Loop rate, in Hz"],
      wheels: [
        type: {:list, :atom},
        default: [:left_wheel, :right_wheel],
        doc: "The actuators whose commands it averages. Both, so a turn cancels out"
      ]
    ]

  alias BB.Message
  alias BB.Message.Actuator.Command.Velocity
  alias BB.NSK.Message.Drive
  alias BB.Robot.Units
  alias BB.StateMachine.Transition
  alias Goatbot.Message.LeanOffset

  @path [:lean_offset]

  @doc "Where the offset is published."
  def path, do: @path

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)

    {:ok, _} = BB.PubSub.subscribe(robot, [:state_machine])

    for wheel <- Keyword.fetch!(opts, :wheels) do
      {:ok, wheel_path} = BB.Robot.actuator_path(robot.robot(), wheel)
      {:ok, _} = BB.PubSub.subscribe(robot, [:actuator | wheel_path])
    end

    {:ok, _} = BB.PubSub.subscribe(robot, BB.NSK.Drive.path(), message_types: [Drive])

    state =
      %{robot: robot, mode: BB.Robot.Runtime.state(robot), last: nil, overrides: %{}}
      |> Map.merge(fresh())
      |> configure(opts)

    schedule(state)
    {:ok, state}
  end

  defp fresh do
    %{
      sum: 0.0,
      count: 0,
      average: 0.0,
      velocity: 0.0,
      offset: 0.0,
      throttle: 0.0,
      throttle_at: nil,
      target_speed: 0.0,
      speed_ref: 0.0
    }
  end

  @impl BB.Controller
  def handle_options(opts, state), do: {:ok, configure(state, opts)}

  @impl BB.Controller
  def handle_call(:inspect, _from, state), do: {:reply, state, state}

  # Live tuning for experiments, ahead of an option becoming a parameter. Lost
  # when the process restarts.
  def handle_call({:tune, opts}, _from, state) do
    state = %{state | overrides: Map.merge(state.overrides, Map.new(opts))}
    {:reply, :ok, Map.merge(state, state.overrides)}
  end

  @impl BB.Controller
  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: to}}}, state) do
    {:noreply, Map.merge(%{state | mode: to}, fresh())}
  end

  def handle_info({:bb, [:actuator | _], %Message{payload: %Velocity{velocity: velocity}}}, state) do
    {:noreply, %{state | sum: state.sum + velocity, count: state.count + 1}}
  end

  def handle_info({:bb, _path, %Message{payload: %Drive{throttle: throttle}} = m}, state) do
    {:noreply, %{state | throttle: throttle, throttle_at: m.monotonic_time}}
  end

  # A hold or stop is the wheel at rest, which counts as a zero command.
  def handle_info({:bb, [:actuator | _], %Message{}}, state) do
    {:noreply, %{state | count: state.count + 1}}
  end

  def handle_info(:tick, state) do
    schedule(state)
    now = System.monotonic_time(:nanosecond)
    dt = if state.last, do: (now - state.last) / 1_000_000_000, else: 0.0

    state = %{
      tick(%{state | target_speed: target_speed(state, now)}, dt)
      | sum: 0.0,
        count: 0,
        last: now
    }

    if state.mode == :balancing do
      {:ok, message} = LeanOffset.new(:velocity_hold, offset: state.offset)
      BB.PubSub.publish(state.robot, @path, message)
    end

    {:noreply, state}
  end

  def handle_info(_message, state), do: {:noreply, state}

  @doc """
  One tick of the loop, given the commands collected since the last one. Pure,
  so it can be run against recorded data.
  """
  @spec tick(map, float) :: map
  def tick(%{mode: :balancing} = state, dt) when dt > 0.0 do
    mean = if state.count > 0, do: state.sum / state.count, else: 0.0
    average = state.average + (mean - state.average) * dt / (state.filter + dt)

    velocity =
      if abs(average) < state.ceiling,
        do: state.velocity + average * dt,
        else: state.velocity

    # Ramped, so a key pressed is a lean eased into rather than a step in the
    # balance loop's target. Easing off is much quicker than building up, so
    # letting go stops the robot rather than letting it coast.
    rate =
      if abs(state.target_speed) < abs(state.speed_ref) or
           state.target_speed * state.speed_ref < 0,
         do: state.drive_decel,
         else: state.drive_accel

    step = rate * dt

    speed_ref =
      if state.target_speed > state.speed_ref,
        do: min(state.speed_ref + step, state.target_speed),
        else: max(state.speed_ref - step, state.target_speed)

    # The filtered command is the wheel speed in rad/s; the reference is in m/s.
    offset =
      (-state.gain * velocity - state.damping * (average - speed_ref / state.wheel_radius))
      |> max(-state.limit)
      |> min(state.limit)

    %{state | average: average, velocity: velocity, offset: offset, speed_ref: speed_ref}
  end

  def tick(state, _dt), do: state

  # The throttle only counts while it's fresh: a driver who stops sending, or a
  # dashboard that has gone away, stops the robot.
  defp target_speed(%{throttle_at: nil}, _now), do: 0.0

  defp target_speed(state, now) do
    if now - state.throttle_at <= state.drive_timeout * 1_000_000,
      do: state.throttle * state.drive_speed,
      else: 0.0
  end

  defp schedule(state), do: Process.send_after(self(), :tick, div(1000, state.rate))

  defp configure(state, opts) do
    state
    |> Map.merge(%{
      gain: Keyword.fetch!(opts, :gain),
      filter: Keyword.fetch!(opts, :filter),
      limit: Units.to_radians(Keyword.fetch!(opts, :limit)),
      ceiling: Keyword.fetch!(opts, :ceiling),
      damping: Keyword.fetch!(opts, :damping),
      drive_speed: Keyword.fetch!(opts, :drive_speed),
      drive_accel: Keyword.fetch!(opts, :drive_accel),
      drive_decel: Keyword.fetch!(opts, :drive_decel),
      drive_timeout: Keyword.fetch!(opts, :drive_timeout),
      wheel_radius: Units.to_meters(Keyword.fetch!(opts, :wheel_radius)),
      rate: Keyword.fetch!(opts, :rate)
    })
    |> Map.merge(state.overrides)
  end
end
