defmodule Goatbot.ButtonController do
  @moduledoc """
  Turns the board's buttons into robot commands.

  | button | does                                  |
  | ------ | ------------------------------------- |
  | 1      | arm, or disarm if armed               |
  | 2      | stand                                 |
  | 3      | explore on or off (`Goatbot.Explore`) |

  Acts on the press, not the release. Commands go through the robot's command
  system as they would from IEx, so a press in a state that doesn't allow the
  command (stand while disarmed, say) is refused and logged rather than acted
  on. Nothing waits for a command to finish, so a slow one doesn't hold up the
  next press.

  The buttons come from the STM32 via `Goatbot.Sensor.Power` on `[:buttons]`,
  so on the host this never hears anything.
  """

  use BB.Controller,
    options_schema: [
      arm_toggle: [type: {:in, [1, 2, 3]}, default: 1, doc: "Button that arms and disarms"],
      stand: [type: {:in, [1, 2, 3]}, default: 2, doc: "Button that stands the robot up"],
      explore: [type: {:in, [1, 2, 3]}, default: 3, doc: "Button that toggles exploring"]
    ]

  require Logger

  alias BB.Message
  alias Goatbot.Message.Button

  @doc """
  The command a button press asks for, given whether the robot is armed.
  Pure, so it can be tested without a robot.
  """
  @spec command(1..3, boolean, keyword) :: :arm | :disarm | :stand | :explore | nil
  def command(button, armed?, buttons) do
    cond do
      button == buttons[:arm_toggle] and armed? -> :disarm
      button == buttons[:arm_toggle] -> :arm
      button == buttons[:stand] -> :stand
      button == buttons[:explore] -> :explore
      true -> nil
    end
  end

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)

    {:ok, _} =
      BB.PubSub.subscribe(robot, Goatbot.Sensor.Power.buttons_path(), message_types: [Button])

    {:ok, %{robot: robot, buttons: Keyword.take(opts, [:arm_toggle, :stand, :explore])}}
  end

  @impl BB.Controller
  def handle_info({:bb, _path, %Message{payload: %Button{button: button, pressed: true}}}, state) do
    case command(button, BB.Safety.armed?(state.robot), state.buttons) do
      nil -> :ok
      name -> run(state.robot, name, button)
    end

    {:noreply, state}
  end

  def handle_info(_message, state), do: {:noreply, state}

  defp run(robot, :explore, button) do
    case Goatbot.Explore.toggle(robot) do
      {:ok, on_or_off} ->
        Logger.info("Button #{button}: explore #{on_or_off}")

      {:error, reason} ->
        Logger.warning("Button #{button}: explore refused: #{inspect(reason)}")
    end
  end

  defp run(robot, name, button) do
    case apply(robot, name, []) do
      {:ok, _pid} ->
        Logger.info("Button #{button}: #{name}")

      {:error, reason} ->
        Logger.warning("Button #{button}: #{name} refused: #{inspect(reason)}")
    end
  end
end
