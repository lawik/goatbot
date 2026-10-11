defmodule Goatbot.BalancerTest do
  use ExUnit.Case, async: true

  import Goatbot.Balancer, only: [held_upright?: 3]

  @deg :math.pi() / 180
  @state %{setpoint: -2.5 * @deg, catch_angle: 2 * @deg, catch_rate: 4 * @deg}

  test "upright and still near the setpoint is held" do
    assert held_upright?(@state, -2.5 * @deg, 0.0)
    assert held_upright?(@state, -1.0 * @deg, 3 * @deg)
  end

  test "too far from the setpoint is not" do
    refute held_upright?(@state, 0.0, 0.0)
    refute held_upright?(@state, -20 * @deg, 0.0)
  end

  test "swinging through upright is not" do
    refute held_upright?(@state, -2.5 * @deg, 30 * @deg)
  end
end
