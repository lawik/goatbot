defmodule Goatbot.Command.Lift do
  @moduledoc """
  Gives up balancing because the robot has been picked up.

  Issued by `Goatbot.Balancer` when the wheel command runs away: in the air the
  wheels can't move the body, so the lean error never shrinks and the integral
  winds the wheels up to full speed. The balancer brakes the wheels itself; this
  only moves the state machine, so the face and everything else watching
  `[:state_machine]` know.

  Like `BB.NSK.Command.Fall`, not a safety transition: the robot stays armed,
  to be put down and stood back up.
  """

  use BB.Command

  @impl BB.Command
  def handle_command(_goal, _context, state), do: {:stop, :normal, state}

  @impl BB.Command
  def result(_state), do: {:ok, :lifted, next_state: :lifted}
end
