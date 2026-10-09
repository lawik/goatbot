defmodule Goatbot.FaceTest do
  use ExUnit.Case, async: true

  import Goatbot.Face, only: [face: 3]

  # Arguments are speed and yaw rate as multiples of their thresholds.
  describe "balancing" do
    test "still, or not past either threshold" do
      assert face(:balancing, 0.0, 0.0) == "-_-"
      assert face(:balancing, 0.9, -0.9) == "-_-"
    end

    test "forward and backward" do
      assert face(:balancing, 2.4, 0.0) == ">.<"
      assert face(:balancing, -1.5, 0.0) == "O_O"
    end

    test "right is a negative yaw rate, left a positive one" do
      assert face(:balancing, 0.0, -1.2) == "O.o"
      assert face(:balancing, 0.0, 1.2) == "o.O"
    end

    test "whichever is further past its threshold wins" do
      assert face(:balancing, 3.0, 1.5) == ">.<"
      assert face(:balancing, -1.2, -2.0) == "O.o"
    end
  end

  test "not balancing ignores motion" do
    assert face(:fallen, 3.0, 0.0) == "x_x"
    assert face(:lifted, 3.0, 0.0) == "^.^"
    assert face(:disarmed, 3.0, 0.0) == "-_-"
    assert face(:idle, 0.0, 3.0) == "-_-"
  end
end
