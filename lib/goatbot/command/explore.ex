defmodule Goatbot.Command.Explore do
  @moduledoc """
  Switches `Goatbot.Explore` on or off, as button 3 does.

  A command so that the web dashboard, which lists the robot's commands, has a
  button for it. Only while balancing: exploring switches itself off whenever
  the robot stops balancing, so there is nothing to toggle otherwise.
  """

  use BB.Command

  @impl BB.Command
  def handle_command(_goal, context, state) do
    {:stop, :normal, %{state | result: Goatbot.Explore.toggle(context.robot_module)}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end
