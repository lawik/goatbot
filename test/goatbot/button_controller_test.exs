defmodule Goatbot.ButtonControllerTest do
  use ExUnit.Case, async: true

  import Goatbot.ButtonController, only: [command: 3]

  @buttons [arm_toggle: 1, stand: 2]

  test "the arm button toggles" do
    assert command(1, false, @buttons) == :arm
    assert command(1, true, @buttons) == :disarm
  end

  test "the stand button stands, armed or not" do
    assert command(2, true, @buttons) == :stand
    assert command(2, false, @buttons) == :stand
  end

  test "the third button does nothing" do
    assert command(3, true, @buttons) == nil
  end
end
