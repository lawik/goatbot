defmodule Goatbot.Leds do
  @moduledoc """
  Shows the robot's state on the NeoPixels, every state breathing.

  A copy of `BB.NSK.Leds.Controller` with its patterns changed, since those
  are fixed inside it. The stock one flashes when fallen, hard on and off four
  times a second, which is a lot to have going on while picking the robot up.
  Here the colour says what's wrong and nothing shouts: every state breathes
  on the same calm `pulse_period`.

  `:lifted` gets a colour of its own; the stock controller doesn't know it.

  As in the stock controller, one timer drives all four pixels, brightness is
  worked out from the monotonic clock so a late tick costs a frame rather than
  the phase, and the pixels are turned off when the robot shuts down.
  """

  use BB.Controller,
    options_schema: [
      brightness: [
        type: :non_neg_integer,
        default: 40,
        doc: "Brightness at the top of a breath, 0 to the device's maximum"
      ],
      pulse_period: [
        type: :pos_integer,
        default: 2_000,
        doc: "Milliseconds for a full breath, in and out"
      ],
      pulse_interval: [
        type: :pos_integer,
        default: 50,
        doc: "Milliseconds between brightness updates"
      ]
    ]

  alias BB.{Message, PubSub}
  alias BB.NSK.Leds
  alias BB.Robot.Runtime
  alias BB.StateMachine.Transition

  require Logger

  # The stock controller's colours, plus `:lifted`.
  @colours %{
    disarmed: {0, 0, 255},
    idle: {0, 255, 0},
    executing: {0, 255, 0},
    balancing: {0, 255, 0},
    fallen: {255, 0, 180},
    lifted: {0, 200, 160},
    disarming: {255, 120, 0},
    error: {255, 0, 0}
  }

  @unknown {80, 0, 80}

  @doc "The colour a state is shown in."
  @spec colour(atom) :: Leds.colour()
  def colour(state), do: Map.get(@colours, state, @unknown)

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    start(robot, opts, Leds.open_all())
  end

  @impl BB.Controller
  def handle_continue(:show_state, state), do: {:noreply, show(state)}

  @impl BB.Controller
  def handle_info({:bb, _path, %Message{payload: %Transition{to: to}}}, state) do
    {:noreply, show(%{state | operational: to})}
  end

  def handle_info(:breathe, state) do
    elapsed = System.monotonic_time(:millisecond) - state.started_at

    level =
      BB.NSK.Leds.Controller.breath(
        elapsed,
        state.brightness,
        dimmest(state),
        state.pulse_period
      )

    {:noreply, %{state | pixels: Leds.set_brightness_all(state.pixels, level)}}
  end

  def handle_info(_message, state), do: {:noreply, state}

  @impl BB.Controller
  def handle_options(opts, state) do
    {:ok,
     show(%{
       state
       | brightness: brightness(opts, state.max_brightness),
         pulse_period: Keyword.fetch!(opts, :pulse_period)
     })}
  end

  @impl BB.Controller
  def terminate(_reason, _state) do
    Leds.off()
    :ok
  end

  defp start(_robot, _opts, []) do
    Logger.info("No NeoPixels found, so #{inspect(__MODULE__)} is not starting.")
    :ignore
  end

  defp start(robot, opts, [pixel | _rest] = pixels) do
    Process.flag(:trap_exit, true)
    {:ok, _subscription} = PubSub.subscribe(robot, [:state_machine])

    max = Leds.max_brightness(pixel.path) || 255

    # Brightness is ours to drive, so make sure nothing in the kernel is too.
    Enum.each(pixels, &Leds.release(&1.path))

    :timer.send_interval(Keyword.fetch!(opts, :pulse_interval), :breathe)

    state = %{
      pixels: pixels,
      operational: Runtime.state(robot),
      max_brightness: max,
      brightness: brightness(opts, max),
      pulse_period: Keyword.fetch!(opts, :pulse_period),
      started_at: System.monotonic_time(:millisecond)
    }

    {:ok, state, {:continue, :show_state}}
  end

  defp brightness(opts, max), do: opts |> Keyword.fetch!(:brightness) |> min(max) |> max(0)

  # Dimming to nothing would read as an LED that has gone out, so a breath
  # bottoms out at a glow.
  defp dimmest(state), do: div(state.brightness, 8)

  defp show(state) do
    Leds.set_all(state.pixels, colour(state.operational))
    state
  end
end
