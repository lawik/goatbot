defmodule Goatbot.Explore do
  @moduledoc """
  Wanders like an old robot vacuum: forward until it bumps into something,
  then back off, turn a random amount and carry on.

  Toggled with `toggle/1` (button 3, via `Goatbot.ButtonController`), and only
  while balancing. Falling or disarming switches it off, so a robot stood back
  up doesn't drive off on its own.

  It drives the same way a pilot does, with `BB.NSK.Drive.command/3` at 20 Hz,
  so speed, ramps and the dead-man timeout all stay with
  `Goatbot.VelocityController` and `Goatbot.YawController`.

  ## Bumps

  There is no bumper, so a bump is the balance being knocked: the lean leaving
  where it has been sitting by more than `bump_angle`, measured against its own
  average over the last second. Hitting something with the wheels tips the
  body forward over them, and hitting it with the body pushes it back; either
  way the balance loop has to recover, which is the moment worth reacting to.
  Speeding up shifts the lean too, so each forward run ignores the first
  `grace` seconds.

  A bump can also be gentle: close enough to something that it can't build up
  speed, it just stands there pushing and the lean never jumps. So a forward
  run that stays slower than `stall_speed` for `stall_time` counts as a bump
  too. Speed is the wheel command averaged over both wheels, times the wheel
  radius, the same estimate `Goatbot.VelocityController` and `Goatbot.Face`
  use.

  ## After a bump

      settle  standing still for `settle` seconds, to get its balance back
      back    reverse at the exploring pace for `backoff` seconds
      turn    a random angle between `min_turn` and `max_turn`, either way,
              measured on the gyro (with a timeout, in case it can't)
      forward and on again
  """

  import BB.Unit

  use BB.Controller,
    options_schema: [
      throttle: [
        type: :float,
        required: true,
        doc: "Fraction of full drive to explore at, forward and back"
      ],
      bump_angle: [
        type: BB.Unit.Option.unit_type(compatible: :degree),
        required: true,
        doc: "How far the lean must jump from its recent average to count as a bump"
      ],
      backoff: [type: :float, required: true, doc: "Seconds to reverse after a bump"],
      min_turn: [
        type: BB.Unit.Option.unit_type(compatible: :degree),
        required: true,
        doc: "Smallest turn after backing off"
      ],
      max_turn: [
        type: BB.Unit.Option.unit_type(compatible: :degree),
        required: true,
        doc: "Largest turn after backing off"
      ],
      turn_rate: [type: :float, default: 1.5, doc: "Turn rate, in rad/s"],
      settle: [type: :float, default: 1.0, doc: "Seconds to stand still after a bump"],
      grace: [
        type: :float,
        default: 1.5,
        doc: "Seconds at the start of a forward run in which bumps are ignored"
      ],
      lean_filter: [
        type: :float,
        default: 1.0,
        doc: "Seconds the lean is averaged over, for bumps to stand out from"
      ],
      stall_speed: [
        type: :float,
        required: true,
        doc: "Speed, in m/s, below which a forward run counts as stuck"
      ],
      stall_time: [
        type: :float,
        required: true,
        doc: "Seconds below `stall_speed` before it counts as a bump"
      ],
      speed_filter: [
        type: :float,
        default: 0.5,
        doc: "Seconds the speed is averaged over"
      ],
      wheel_radius: [
        type: BB.Unit.Option.unit_type(compatible: :meter),
        default: ~u(21.5 millimeter),
        doc: "Turns wheel rad/s into m/s"
      ],
      wheels: [
        type: {:list, :atom},
        default: [:left_wheel, :right_wheel],
        doc: "The actuators whose commands it averages. Both, so a turn cancels out"
      ],
      rate: [type: :pos_integer, default: 20, doc: "Drive commands per second"]
    ]

  require Logger

  alias BB.Message
  alias BB.Message.Actuator.Command.Velocity
  alias BB.Message.Geometry.Twist2D
  alias BB.Message.Sensor.JointState
  alias BB.Robot.Units
  alias BB.StateMachine.Transition

  @doc """
  Switch exploring on or off. Only switches on while the robot is balancing.
  """
  @spec toggle(module) :: {:ok, :on | :off} | {:error, :not_balancing}
  def toggle(robot), do: BB.Process.call(robot, :explorer, :toggle)

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)

    {:ok, lean_joint} = BB.Robot.path_to(robot.robot(), :lean)
    {:ok, ground} = BB.Robot.path_to(robot.robot(), :ground)

    {:ok, _} = BB.PubSub.subscribe(robot, [:state_machine])

    {:ok, _} =
      BB.PubSub.subscribe(robot, [:sensor | lean_joint] ++ [:lean_angle],
        message_types: [JointState]
      )

    {:ok, _} =
      BB.PubSub.subscribe(robot, [:sensor | ground] ++ [:heading], message_types: [JointState])

    for wheel <- Keyword.fetch!(opts, :wheels) do
      {:ok, wheel_path} = BB.Robot.actuator_path(robot.robot(), wheel)
      {:ok, _} = BB.PubSub.subscribe(robot, [:actuator | wheel_path])
    end

    state = %{
      robot: robot,
      mode: BB.Robot.Runtime.state(robot),
      phase: :off,
      phase_at: nil,
      last: nil,
      lean_avg: nil,
      lean_at: nil,
      deviation: 0.0,
      yaw_rate: 0.0,
      turned: 0.0,
      turn_target: 0.0,
      sum: 0.0,
      count: 0,
      speed: 0.0,
      slow_since: nil
    }

    {:ok, configure(state, opts)}
  end

  @impl BB.Controller
  def handle_options(opts, state), do: {:ok, configure(state, opts)}

  @impl BB.Controller
  def handle_call(:toggle, _from, %{phase: :off, mode: :balancing} = state) do
    Logger.info("Explore: on")
    send(self(), :tick)
    {:reply, {:ok, :on}, enter(state, :forward, now())}
  end

  def handle_call(:toggle, _from, %{phase: :off} = state) do
    {:reply, {:error, :not_balancing}, state}
  end

  def handle_call(:toggle, _from, state) do
    Logger.info("Explore: off")
    {:reply, {:ok, :off}, stop(state)}
  end

  def handle_call(:inspect, _from, state), do: {:reply, state, state}

  @impl BB.Controller
  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: to}}}, state) do
    state = %{state | mode: to}

    if to != :balancing and state.phase != :off do
      Logger.info("Explore: off, the robot is #{to}")
      {:noreply, stop(state)}
    else
      {:noreply, state}
    end
  end

  # The heading sensor's twist and the lean sensor's plain rate tell the two apart.
  def handle_info(
        {:bb, _path, %Message{payload: %JointState{velocities: [%Twist2D{omega: omega}]}}},
        state
      ) do
    {:noreply, %{state | yaw_rate: omega}}
  end

  def handle_info({:bb, _path, %Message{payload: %JointState{positions: [lean]}} = m}, state) do
    {:noreply, track_lean(state, lean, m.monotonic_time)}
  end

  def handle_info({:bb, [:actuator | _], %Message{payload: %Velocity{velocity: velocity}}}, state) do
    {:noreply, %{state | sum: state.sum + velocity, count: state.count + 1}}
  end

  # A hold or stop is the wheel at rest, which counts as a zero command.
  def handle_info({:bb, [:actuator | _], %Message{}}, state) do
    {:noreply, %{state | count: state.count + 1}}
  end

  def handle_info(:tick, %{phase: :off} = state), do: {:noreply, state}

  def handle_info(:tick, state) do
    Process.send_after(self(), :tick, div(1000, state.rate))
    state = step(state, now())
    {throttle, turn} = drive(state)
    BB.NSK.Drive.command(state.robot, throttle, turn)
    {:noreply, state}
  end

  def handle_info(_message, state), do: {:noreply, state}

  @doc """
  One tick of exploring: fold in the wheel commands and the turn since the last
  one, and move on to the next phase if this one is done. Pure but for the random turn, so it can be tested without a
  robot. `now` is in seconds.
  """
  @spec step(map, float) :: map
  def step(state, now) do
    dt = if state.last, do: now - state.last, else: 0.0

    state =
      %{state | last: now, turned: state.turned + state.yaw_rate * dt}
      |> sample_speed(dt)

    elapsed = now - state.phase_at

    case state.phase do
      :forward when elapsed <= state.grace ->
        state

      :forward ->
        state = track_slow(state, now)

        cond do
          abs(state.deviation) > state.bump_angle ->
            Logger.info("Explore: bump, lean off by #{Float.round(state.deviation * 57.3, 1)}°")
            enter(state, :settle, now)

          state.slow_since && now - state.slow_since >= state.stall_time ->
            Logger.info("Explore: stuck at #{Float.round(state.speed * 100, 1)} cm/s")
            enter(state, :settle, now)

          true ->
            state
        end

      :settle ->
        if elapsed >= state.settle, do: enter(state, :back, now), else: state

      :back ->
        if elapsed >= state.backoff, do: enter(state, :turn, now), else: state

      :turn ->
        # Twice as long as the turn should take, and a second, in case it
        # can't turn, so it carries on rather than spinning in place forever.
        timeout = 2 * abs(state.turn_target) / state.turn_rate + 1.0

        if abs(state.turned) >= abs(state.turn_target) or elapsed > timeout,
          do: enter(state, :forward, now),
          else: state
    end
  end

  @doc "The throttle and turn rate a phase drives with."
  @spec drive(map) :: {float, float}
  def drive(%{phase: :forward} = state), do: {state.throttle, 0.0}
  def drive(%{phase: :back} = state), do: {-state.throttle, 0.0}
  def drive(%{phase: :turn} = state), do: {0.0, sign(state.turn_target) * state.turn_rate}
  def drive(_state), do: {0.0, 0.0}

  defp enter(state, :turn, now) do
    angle = state.min_turn + :rand.uniform() * (state.max_turn - state.min_turn)
    direction = if :rand.uniform() < 0.5, do: -1.0, else: 1.0

    %{state | phase: :turn, phase_at: now, turned: 0.0, turn_target: direction * angle}
  end

  defp enter(state, phase, now), do: %{state | phase: phase, phase_at: now, slow_since: nil}

  defp sample_speed(state, dt) when dt > 0.0 do
    command = if state.count > 0, do: state.sum / state.count, else: 0.0
    k = dt / (state.speed_filter + dt)
    speed = state.speed + (command * state.wheel_radius - state.speed) * k
    %{state | speed: speed, sum: 0.0, count: 0}
  end

  defp sample_speed(state, _dt), do: %{state | sum: 0.0, count: 0}

  # When the forward run first fell below `stall_speed`, or nil while it's
  # making headway.
  defp track_slow(%{speed: speed, stall_speed: stall} = state, _now) when speed >= stall,
    do: %{state | slow_since: nil}

  defp track_slow(%{slow_since: nil} = state, now), do: %{state | slow_since: now}
  defp track_slow(state, _now), do: state

  defp stop(state) do
    BB.NSK.Drive.stop(state.robot)
    %{state | phase: :off, last: nil}
  end

  # The lean against its own recent average. While not exploring the average
  # still follows, so it is settled by the time a forward run starts.
  defp track_lean(%{lean_avg: nil} = state, lean, at),
    do: %{state | lean_avg: lean, lean_at: at, deviation: 0.0}

  defp track_lean(state, lean, at) do
    dt = (at - state.lean_at) / 1_000_000_000
    avg = state.lean_avg + (lean - state.lean_avg) * dt / (state.lean_filter + dt)
    %{state | lean_avg: avg, lean_at: at, deviation: lean - avg}
  end

  defp sign(x) when x < 0, do: -1.0
  defp sign(_x), do: 1.0

  defp now, do: System.monotonic_time(:millisecond) / 1000

  defp configure(state, opts) do
    Map.merge(state, %{
      throttle: Keyword.fetch!(opts, :throttle),
      bump_angle: Units.to_radians(Keyword.fetch!(opts, :bump_angle)),
      backoff: Keyword.fetch!(opts, :backoff),
      min_turn: Units.to_radians(Keyword.fetch!(opts, :min_turn)),
      max_turn: Units.to_radians(Keyword.fetch!(opts, :max_turn)),
      turn_rate: Keyword.fetch!(opts, :turn_rate),
      settle: Keyword.fetch!(opts, :settle),
      grace: Keyword.fetch!(opts, :grace),
      lean_filter: Keyword.fetch!(opts, :lean_filter),
      stall_speed: Keyword.fetch!(opts, :stall_speed),
      stall_time: Keyword.fetch!(opts, :stall_time),
      speed_filter: Keyword.fetch!(opts, :speed_filter),
      wheel_radius: Units.to_meters(Keyword.fetch!(opts, :wheel_radius)),
      rate: Keyword.fetch!(opts, :rate)
    })
  end
end
