defmodule Goatbot.Dashboard do
  @moduledoc """
  A terminal dashboard for the running robot, built with Breeze.

  Run it from an IEx session on the device (over SSH is fine) with
  `Goatbot.Dashboard.run/0`, or `dash()` from the IEx helpers.

  ## Keys

  | Key | Does |
  | --- | --- |
  | `a` | arm |
  | `s` | stand: hand the robot to the balancer (from idle or fallen) |
  | `d` | disarm |
  | `e` | explore on / off (while balancing), see `Goatbot.Explore` |
  | `↑` / `↓` | hold to drive forward / back; let go to stop |
  | `←` / `→` | hold to turn left / right; let go to hold the new direction |
  | `p` | pause / resume the updates |
  | `q` | quit, stopping any drive, and hand the shell back |

  Commands go through the robot's command system, so `allowed_states` still
  decides what may run; a refusal shows up in the events panel. They run in a
  task, so the screen keeps updating while one completes.

  ## Driving

  Holding `↑` or `↓` sends a full throttle forward or back with
  `BB.NSK.Drive.command/3`, every `@drive_ms`, and `Goatbot.VelocityController`
  turns it into a target speed (`drive_speed`, ramped at `drive_accel`). The
  setpoint is never touched. Holding `←` or `→` sends a turn at the robot's
  `[:yaw, :turn_rate]` instead, which `Goatbot.YawController` follows. One
  direction at a time: a terminal only repeats the last key pressed.

  A terminal reports key presses, not releases, so holding is inferred from auto
  repeat: each press keeps the drive alive, and it ends once the presses stop.
  The first press is given `@first_press_ms` to cover the keyboard's delay
  before it starts repeating; after that, `@repeat_ms` without a press is a
  release. The velocity controller has its own timeout too, so the robot stops
  even if the dashboard goes away mid-drive.

  ## Keeping it cheap

  The board has two cores, already busy running the balance loop, and a full
  redraw costs it far more than it costs a laptop. So the view redraws on a
  timer, `@refresh_ms` apart, rather than whenever something changes, and each
  section is one bordered box of pre-formatted text rather than a box per label
  and value: in Breeze every box costs, and a `panel` block costs several.

  It shows a general picture and nothing more: state and safety from
  `BB.Robot.Runtime` and `BB.Safety`, the lean from the joint configurations,
  the two controllers' state through their `:inspect` calls, and state
  transitions and parameter changes from PubSub. For detail, `params()` and
  `procs()` in IEx. If the robot isn't running it says so and keeps checking.
  """

  use Breeze.View

  alias BB.Message
  alias BB.Message.Sensor.BatteryState
  alias BB.Parameter.Changed
  alias BB.StateMachine.Transition

  @robot Goatbot.Robot
  @refresh_ms 250
  @wheel_radius 0.0215
  @deg 180 / :math.pi()
  @drive_ms 50
  @first_press_ms 600
  @repeat_ms 150
  @keys "a arm · s stand · d disarm · e explore · hold ↑↓ drive · hold ←→ turn · p pause · q quit"

  @doc "Runs the dashboard in the current terminal until `q` is pressed."
  def run, do: Breeze.Server.run(view: __MODULE__)

  @impl Breeze.View
  def mount(_opts, term) do
    send(self(), :tick)

    {:ok,
     assign(term,
       paused: false,
       subscribed: false,
       snapshot: snapshot(),
       events: [],
       battery: nil,
       held: nil,
       held_until: nil,
       pending: nil
     )}
  end

  ## Updates

  @impl Breeze.View
  def handle_info(:tick, term) do
    Process.send_after(self(), :tick, @refresh_ms)

    if term.assigns.paused do
      {:noreply, term, invalidate: false}
    else
      snap = snapshot()
      {:noreply, term |> subscribe(snap) |> assign(snapshot: snap)}
    end
  end

  def handle_info(
        {:bb, [:state_machine], %Message{payload: %Transition{from: from, to: to}}},
        term
      ) do
    {:noreply, event(term, "#{from} → #{to}", state_color(to))}
  end

  def handle_info({:bb, [:param | _], %Message{payload: %Changed{} = changed}}, term) do
    text = "#{Enum.join(changed.path, ".")} = #{show(changed.new_value)}"
    {:noreply, event(term, text, "primary")}
  end

  def handle_info({:bb, [:sensor, :power], %Message{payload: %BatteryState{} = battery}}, term) do
    {:noreply, assign(term, battery: battery), invalidate: false}
  end

  def handle_info({:command_done, name, result}, term) do
    {text, color} = describe(name, result)
    {:noreply, term |> assign(pending: nil) |> event(text, color)}
  end

  # While a key is held: keep the throttle fresh, or notice it's been let go.
  def handle_info(:drive_tick, %{assigns: %{held: nil}} = term),
    do: {:noreply, term, invalidate: false}

  def handle_info(:drive_tick, term) do
    if now_ms() > term.assigns.held_until do
      safely(fn -> BB.NSK.Drive.stop(@robot) end)
      {:noreply, assign(term, held: nil, held_until: nil)}
    else
      {throttle, turn} = request(term.assigns.held)
      safely(fn -> BB.NSK.Drive.command(@robot, throttle, turn) end)
      Process.send_after(self(), :drive_tick, @drive_ms)
      {:noreply, term, invalidate: false}
    end
  end

  def handle_info(_message, term), do: {:noreply, term, invalidate: false}

  ## Keys

  @impl Breeze.View
  def handle_event(:input, %{"key" => "p"}, term) do
    {:noreply, assign(term, paused: !term.assigns.paused)}
  end

  def handle_event(:input, %{"key" => "a"}, term), do: {:noreply, command(term, :arm)}
  def handle_event(:input, %{"key" => "s"}, term), do: {:noreply, command(term, :stand)}
  def handle_event(:input, %{"key" => "d"}, term), do: {:noreply, command(term, :disarm)}
  def handle_event(:input, %{"key" => "e"}, term), do: {:noreply, command(term, :explore)}
  def handle_event(:input, %{"key" => "ArrowUp"}, term), do: {:noreply, press(term, :forward)}
  def handle_event(:input, %{"key" => "ArrowDown"}, term), do: {:noreply, press(term, :back)}
  def handle_event(:input, %{"key" => "ArrowLeft"}, term), do: {:noreply, press(term, :left)}
  def handle_event(:input, %{"key" => "ArrowRight"}, term), do: {:noreply, press(term, :right)}

  # Stop explicitly on the way out rather than leaning on the timeout.
  def handle_event(:input, %{"key" => "q"}, term) do
    safely(fn -> BB.NSK.Drive.stop(@robot) end)
    {:stop, term}
  end

  def handle_event(_event, _payload, term), do: {:noreply, term}

  ## Rendering

  @impl Breeze.View
  def render(assigns) do
    assigns =
      assign(assigns,
        robot_text: robot_text(assigns)
      )

    ~H"""
    <box class="w-screen">
      <box class="inline w-full h-1 bg-panel overflow-hidden">
        <box class="font-bold text-primary px-1">goatbot</box>
        <box class={"font-bold px-1 text-#{state_color(@snapshot.state)}"}>
          {header_state(@snapshot)}
        </box>
        <box class={"px-1 text-#{battery_color(@battery)}"}>{battery_text(@battery)}</box>
        <box class="text-muted px-1">{header_info(@snapshot)}</box>
        <box :if={@paused} class="text-warning font-bold px-1">PAUSED</box>
      </box>
      <box class={"w-full h-1 overflow-hidden px-1 " <> banner_class(@snapshot)}>
        {banner_text(@snapshot, @pending)}
      </box>
      <box class="grid grid-cols-2 gap-x-1 w-full">
        <box class="border rounded h-11 overflow-hidden">
          <box class="font-bold text-primary">Robot</box>
          <box>{@robot_text}</box>
        </box>
        <box class="border rounded h-11 overflow-hidden">
          <box class="font-bold text-primary">Events</box>
          <box :if={@events == []} class="text-muted">nothing yet</box>
          <box :for={{line, color} <- @events} class={"overflow-hidden text-#{color}"}>{line}</box>
        </box>
      </box>
      <box class="h-1 bg-panel text-muted overflow-hidden px-1">{keys()}</box>
    </box>
    """
  end

  defp keys, do: @keys

  defp header_state(%{running: false}), do: "robot not running"
  defp header_state(snap), do: "● #{snap.state}"

  # A single-cell LiPo: worth knowing below 3.5 V, worrying below 3.3 V.
  defp battery_text(nil), do: "battery -"

  defp battery_text(%BatteryState{} = b) do
    usb = if b.power_supply_status in [:charging, :not_charging], do: " · USB", else: ""
    "battery #{num(b.voltage, 2)} V #{num(b.current, 2)} A" <> usb
  end

  defp battery_color(%BatteryState{voltage: v}) when v < 3.3, do: "error"
  defp battery_color(%BatteryState{voltage: v}) when v < 3.5, do: "warning"
  defp battery_color(%BatteryState{}), do: "success"
  defp battery_color(nil), do: "muted"

  defp header_info(%{running: false} = snap), do: system_info(snap)
  defp header_info(snap), do: "safety #{snap.safety} · " <> system_info(snap)

  defp system_info(snap),
    do: "up #{uptime(snap.uptime)} · #{snap.memory_mb} MB · #{snap.processes} procs"

  defp robot_text(%{snapshot: %{balancer: nil} = snap}),
    do: rows([{"lean", deg(snap.lean)}, {"balancer", "not running"}])

  defp robot_text(%{snapshot: snap, held: held}) do
    b = snap.balancer
    v = snap.velocity
    y = snap.yaw

    rows([
      {"lean", deg(snap.lean)},
      {"setpoint", deg(b.setpoint)},
      {"target", "#{deg(b[:target] || b.setpoint)}  (setpoint + velocity offset)"},
      {"command", "#{num(b.last_command, 2)} rad/s"},
      {"speed", if(v, do: "#{num(v.average * @wheel_radius * 100, 1)} cm/s", else: "-")},
      {"offset", if(v, do: deg(v.offset), else: "-")},
      {"heading", heading_text(y)},
      {"drive", drive_text(held, v, snap.explore)}
    ])
  end

  defp rows(pairs),
    do: Enum.map_join(pairs, "\n", fn {k, v} -> String.pad_trailing(k, 10) <> v end)

  ## Snapshot

  # Everything else the view shows, read once per redraw. Every read is
  # guarded: the robot may not be running, or may be restarting underneath us.
  @doc false
  def snapshot do
    running = Process.whereis(@robot) != nil

    base = %{
      running: running,
      uptime: :erlang.statistics(:wall_clock) |> elem(0),
      memory_mb: div(:erlang.memory(:total), 1_048_576),
      processes: :erlang.system_info(:process_count),
      state: nil,
      safety: nil,
      lean: nil,
      balancer: nil,
      velocity: nil,
      yaw: nil,
      explore: nil
    }

    if running do
      %{
        base
        | state: safely(fn -> BB.Robot.Runtime.state(@robot) end),
          safety: safely(fn -> BB.Safety.state(@robot) end),
          lean: safely(fn -> Map.get(BB.Robot.Runtime.configurations(@robot), :lean) end),
          balancer: safely(fn -> BB.Process.call(@robot, :pid_balancer, :inspect, 200) end),
          velocity: safely(fn -> BB.Process.call(@robot, :velocity_hold, :inspect, 200) end),
          yaw: safely(fn -> BB.Process.call(@robot, :yaw_hold, :inspect, 200) end),
          explore: safely(fn -> BB.Process.call(@robot, :explorer, :inspect, 200) end)
      }
    else
      base
    end
  end

  defp safely(fun) do
    fun.()
  rescue
    _ -> nil
  catch
    :exit, _ -> nil
  end

  # Subscribed once the robot is first seen running: state changes and
  # parameter changes for the events panel, and the battery for the header.
  defp subscribe(%{assigns: %{subscribed: true}} = term, _snap), do: term
  defp subscribe(term, %{running: false}), do: term

  defp subscribe(term, _snap) do
    for path <- [[:state_machine], [:param], [:sensor, :power]],
        do: {:ok, _} = BB.PubSub.subscribe(@robot, path)

    assign(term, subscribed: true)
  rescue
    _ -> term
  end

  defp event(term, text, color) do
    time = Calendar.strftime(NaiveDateTime.local_now(), "%H:%M:%S")
    assign(term, events: Enum.take([{"#{time} #{text}", color} | term.assigns.events], 8))
  end

  ## Commands and driving

  # Await in a task: arming can take a while, and the screen should keep
  # updating meanwhile. The result comes back as a message.
  defp command(%{assigns: %{pending: pending}} = term, _name) when not is_nil(pending), do: term

  defp command(term, name) do
    view = self()

    Task.start(fn ->
      result =
        try do
          case apply(@robot, name, []) do
            {:ok, pid} -> BB.Command.await(pid, 60_000)
            other -> other
          end
        catch
          kind, reason -> {:error, {kind, reason}}
        end

      send(view, {:command_done, name, result})
    end)

    assign(term, pending: name)
  end

  defp describe(name, {:ok, result}), do: {"#{name}: #{result}", "success"}
  defp describe(name, {:ok, result, _opts}), do: {"#{name}: #{result}", "success"}

  defp describe(name, {:error, %{__exception__: true} = error}),
    do: {"#{name} refused: #{Exception.message(error)}", "error"}

  defp describe(name, other), do: {"#{name}: #{inspect(other, limit: 5)}", "error"}

  # A press starts or continues a hold. Repeats of the key being held keep it
  # alive for `@repeat_ms`; a new press (or another arrow) gets
  # `@first_press_ms` to cover the keyboard's delay before it starts repeating.
  defp press(term, direction) do
    window = if term.assigns.held == direction, do: @repeat_ms, else: @first_press_ms
    starting? = is_nil(term.assigns.held)
    changed? = term.assigns.held != direction

    term = assign(term, held: direction, held_until: now_ms() + window)
    if starting?, do: send(self(), :drive_tick)
    if changed?, do: event(term, "#{direction}", "primary"), else: term
  end

  # What a held arrow asks for: a throttle, or a turn rate in rad/s (positive
  # left).
  defp request(:forward), do: {1.0, 0.0}
  defp request(:back), do: {-1.0, 0.0}
  defp request(:left), do: {0.0, turn_rate()}
  defp request(:right), do: {0.0, -turn_rate()}

  defp turn_rate, do: safely(fn -> BB.Parameter.get!(@robot, [:yaw, :turn_rate]) end) || 1.0

  defp now_ms, do: System.monotonic_time(:millisecond)

  # Exploring drives through the same messages as the arrows, so it takes the
  # same row.
  defp drive_text(nil, v, %{phase: phase}) when phase != :off,
    do: "◆ exploring: #{phase}" <> speed_ref_text(v)

  defp drive_text(nil, v, _explore), do: "— hold an arrow" <> speed_ref_text(v)
  defp drive_text(:forward, v, _explore), do: "▲ forward" <> speed_ref_text(v)
  defp drive_text(:back, v, _explore), do: "▼ back" <> speed_ref_text(v)
  defp drive_text(:left, _v, _explore), do: "◀ turning left"
  defp drive_text(:right, _v, _explore), do: "▶ turning right"

  defp heading_text(nil), do: "-"

  defp heading_text(y) do
    "#{num(y.heading * @deg, 1)}°  holding #{num(y.hold * @deg, 1)}°  differential #{num(y.differential, 2)}"
  end

  defp speed_ref_text(%{speed_ref: ref, target_speed: target}) when ref != 0.0 or target != 0.0,
    do: "  (target #{num(target * 100, 1)} cm/s, ramping #{num(ref * 100, 1)})"

  defp speed_ref_text(_v), do: ""

  ## Banner

  defp banner_text(snap, nil), do: banner(snap)
  defp banner_text(snap, pending), do: "#{banner(snap)} · #{pending}…"

  defp banner(%{running: false}), do: "start_robot() to start the robot"
  defp banner(%{state: :disarmed}), do: "DISARMED · a to arm"
  defp banner(%{state: :idle}), do: "ARMED · hold it upright and still, then s to balance"
  defp banner(%{state: :executing}), do: "running a command"
  defp banner(%{state: :disarming}), do: "disarming"

  defp banner(%{state: :balancing}),
    do: "BALANCING · hold arrows to drive and turn · e explore · d disarm"

  defp banner(%{state: :fallen}),
    do: "FALLEN · wheels braked · stand it up, hold it still, then s"

  defp banner(%{state: :error}),
    do: "SAFETY ERROR · a disarm callback failed · BB.Safety.force_disarm(Goatbot.Robot) to clear"

  defp banner(%{state: state}), do: to_string(state)

  defp banner_class(%{state: :fallen}), do: "bg-error text-bg font-bold"
  defp banner_class(%{state: :error}), do: "bg-error text-bg font-bold"
  defp banner_class(%{state: :balancing}), do: "bg-panel text-success"

  defp banner_class(%{state: state}) when state in [:idle, :executing],
    do: "bg-panel text-warning"

  defp banner_class(_snap), do: "bg-panel text-muted"

  ## Formatting

  defp state_color(:balancing), do: "success"
  defp state_color(:fallen), do: "error"
  defp state_color(:error), do: "error"
  defp state_color(:idle), do: "warning"
  defp state_color(:executing), do: "warning"
  defp state_color(nil), do: "error"
  defp state_color(_), do: "muted"

  defp deg(nil), do: "-"
  defp deg(radians), do: "#{num(radians * @deg, 2)}°"

  defp num(nil, _), do: "-"
  defp num(value, decimals), do: :erlang.float_to_binary(value / 1, decimals: decimals)

  defp show(%Localize.Unit{value: value, name: name}), do: "#{value} #{name}"
  defp show(value) when is_float(value), do: :erlang.float_to_binary(value, [:short])
  defp show(value), do: inspect(value)

  defp uptime(ms) do
    s = div(ms, 1000)
    minutes = s |> rem(3600) |> div(60) |> Integer.to_string() |> String.pad_leading(2, "0")
    "#{div(s, 3600)}h#{minutes}m"
  end
end
