defmodule GoatbotWeb.DriveLive do
  @moduledoc """
  A thumb on a phone, driving the robot.

  The pad is *absolute*: the centre of it is neutral, and the command is where
  the thumb is relative to that. The edges are full deflection, so the whole
  screen means something and a thumb can be put straight where it means instead
  of dragged there — which matters most at the edges, where a relative pad has
  nowhere left to drag into and can only ever ask for a fraction of a turn.

  **It commands whatever the thumb lands on, immediately.** A thumb arriving at
  the edge asks for full deflection from a standing start, where a relative pad
  would have begun at zero by construction. What keeps that from throwing the
  robot is `drive_slew` in the balance controller, which ramps the drive lean
  rather than stepping it — so this trades a guarantee in the interface for one
  in the loop.

  Up is forwards and right turns right, mapped to `BB.NSK.Drive.command/3`.

  ## What stops the robot

  Three things, deliberately overlapping, because this is a robot being driven
  over wifi by a browser.

  Lifting a finger sends a zero. Closing the tab or walking out of range sends
  nothing at all, and the balance controller forgets a drive command after
  `drive_timeout`. And `terminate/2` sends a zero as the LiveView goes down, which
  covers a tab closed cleanly while the robot is mid-drive.

  The socket itself is not a safety mechanism: LiveView will happily reconnect and
  resume, so the timeout in the controller is the thing actually keeping this
  honest.
  """

  use GoatbotWeb, :live_view

  alias BB.StateMachine.Transition
  alias BB.NSK.Drive

  @robot Goatbot.Robot

  # Radians a second of turn at full deflection. A quarter turn a second, which
  # is brisk enough to be useful and slow enough to be aimed.
  @full_turn 1.5

  @impl Phoenix.LiveView
  def mount(_params, _session, socket) do
    if connected?(socket) do
      Process.flag(:trap_exit, true)
      BB.PubSub.subscribe(@robot, [:state_machine])
    end

    {:ok,
     socket
     |> assign(throttle: 0.0, turn: 0.0, driving?: false, error: nil)
     |> assign(name: name(), state: robot_state()), layout: false}
  end

  @impl Phoenix.LiveView
  def handle_event("drive", %{"x" => x, "y" => y}, socket) do
    # Up the screen is forwards, and the browser's Y grows downwards.
    throttle = clamp(y) * -1.0
    turn = clamp(x) * -@full_turn

    Drive.command(@robot, throttle, turn)

    {:noreply, assign(socket, throttle: throttle, turn: turn, driving?: true)}
  end

  def handle_event("release", _params, socket) do
    Drive.stop(@robot)

    {:noreply, assign(socket, throttle: 0.0, turn: 0.0, driving?: false)}
  end

  def handle_event("arm", _params, socket), do: {:noreply, command(socket, :arm)}

  # Disarming stops the wheels and drops a balancing robot, which is the point:
  # it is the button you hit when something is going wrong, so it stays on screen
  # and reachable whatever else is happening.
  def handle_event("disarm", _params, socket) do
    Drive.stop(@robot)

    {:noreply, command(socket, :disarm)}
  end

  @impl Phoenix.LiveView
  def handle_info({:bb, _path, %BB.Message{payload: %Transition{to: to}}}, socket) do
    {:noreply, assign(socket, state: to, throttle: 0.0, turn: 0.0, driving?: false, error: nil)}
  end

  def handle_info(_message, socket), do: {:noreply, socket}

  @impl Phoenix.LiveView
  def terminate(_reason, _socket) do
    Drive.stop(@robot)
  end

  defp command(socket, name) do
    case apply(@robot, name, []) do
      {:ok, _command} -> assign(socket, error: nil)
      {:error, reason} -> assign(socket, error: inspect(reason))
    end
  end

  defp clamp(value) when is_number(value), do: value |> max(-1.0) |> min(1.0)
  defp clamp(_value), do: 0.0

  # Read from the OTP application rather than written down, so a robot built from
  # this wears its own name. It is the way back to the setup page, which is the
  # root, so it is a link rather than a label.
  defp name, do: __MODULE__ |> Application.get_application() |> to_string()

  # One value covering both halves: a disarmed robot reports `:disarmed` here
  # whatever its operational state is. The rescue is for a page served while the
  # robot itself is not running, which on the host is most of the time.
  defp robot_state do
    BB.Robot.Runtime.state(@robot)
  rescue
    _ -> :unknown
  catch
    :exit, _ -> :unknown
  end

  @impl Phoenix.LiveView
  def render(assigns) do
    ~H"""
    <div class="fixed inset-0 flex touch-none select-none flex-col bg-zinc-900 text-zinc-100">
      <div class="flex items-center justify-between px-4 py-3 text-sm font-mono">
        <a href="/" class="underline decoration-zinc-600 underline-offset-4">{@name}</a>
        <div class="flex items-center gap-3">
          <span
            class={["rounded px-2 py-1", @driving? && "bg-emerald-600", !@driving? && "bg-zinc-700"]}
          >
            {if @driving? do
              "driving"
            else
              @state
            end}
          </span>
          <button
            :if={@state != :disarmed}
            phx-click="disarm"
            class="rounded border border-red-500/60 px-2 py-1 text-red-400"
          >
            disarm
          </button>
        </div>
      </div>
      <p :if={@error} class="border-t border-zinc-700 px-4 py-2 text-sm text-amber-500">{@error}</p>
      <div
        :if={@state == :disarmed}
        class="flex flex-1 flex-col items-center justify-center gap-6 border-t border-zinc-700 p-8"
      >
        <p class="max-w-xs px-4 text-center text-balance text-zinc-400">
          The robot is disarmed and will ignore anything you ask of it.
        </p>
        <button phx-click="arm" class="w-full max-w-xs rounded bg-emerald-600 px-4 py-4 text-lg">
          arm
        </button>
      </div>
      <div
        :if={@state != :disarmed}
        id="pad"
        phx-hook="DrivePad"
        class="relative flex-1 touch-none border-t border-zinc-700"
      >
        <div
          id="pad-origin"
          class="pointer-events-none absolute left-1/2 top-1/2 h-24 w-24 -translate-x-1/2 -translate-y-1/2 rounded-full border-2 border-zinc-600"
        >
        </div>
        <div
          id="pad-thumb"
          class="pointer-events-none absolute hidden h-12 w-12 -translate-x-1/2 -translate-y-1/2 rounded-full bg-emerald-500/80"
        >
        </div>
        <p
          :if={!@driving?}
          class="pointer-events-none absolute inset-0 flex items-end justify-center pb-10 text-zinc-500"
        >
          touch anywhere to drive — the ring is neutral
        </p>
      </div>
      <div class="grid grid-cols-2 gap-px bg-zinc-700 font-mono text-sm">
        <div class="bg-zinc-900 px-4 py-3">
          throttle 
          <span class="float-right">{fmt(@throttle)}</span>
        </div>
        <div class="bg-zinc-900 px-4 py-3">
          turn 
          <span class="float-right">{fmt(@turn)} rad/s</span>
        </div>
      </div>
    </div>
    """
  end

  defp fmt(value), do: :erlang.float_to_binary(value * 1.0, decimals: 2)
end
