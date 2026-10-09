defmodule Goatbot.ExploreTest do
  use ExUnit.Case, async: true

  alias Goatbot.Explore

  defp state(overrides) do
    Map.merge(
      %{
        phase: :forward,
        phase_at: 0.0,
        last: nil,
        deviation: 0.0,
        yaw_rate: 0.0,
        turned: 0.0,
        turn_target: 0.0,
        throttle: 0.5,
        bump_angle: 0.05,
        backoff: 1.2,
        min_turn: 0.8,
        max_turn: 3.1,
        turn_rate: 1.5,
        settle: 1.0,
        grace: 1.5,
        sum: 0.0,
        count: 0,
        speed: 0.1,
        slow_since: nil,
        stall_speed: 0.03,
        stall_time: 1.5,
        speed_filter: 0.5,
        wheel_radius: 0.0215
      },
      Map.new(overrides)
    )
  end

  describe "forward" do
    test "drives at the exploring throttle" do
      assert Explore.drive(state(phase: :forward)) == {0.5, 0.0}
    end

    test "ignores a lean jump during the grace period" do
      assert Explore.step(state(deviation: 0.1), 1.0).phase == :forward
    end

    test "a lean jump after it is a bump" do
      assert Explore.step(state(deviation: -0.1), 2.0).phase == :settle
    end

    test "a wobble inside bump_angle is not" do
      assert Explore.step(state(deviation: 0.03), 5.0).phase == :forward
    end
  end

  describe "stuck" do
    test "slower than stall_speed for stall_time is a bump" do
      s = state(speed: 0.01, last: 2.0)
      s = Explore.step(s, 2.0)
      assert s.phase == :forward and s.slow_since == 2.0

      s = Explore.step(s, 3.0)
      assert s.phase == :forward

      assert Explore.step(s, 3.6).phase == :settle
    end

    test "picking up speed again resets it" do
      s = Explore.step(state(speed: 0.01, last: 2.0), 2.0)
      # Both wheels at 5 rad/s is about 11 cm/s.
      s = Explore.step(%{s | sum: 10.0, count: 2, speed: 0.1}, 2.1)
      assert s.slow_since == nil
      assert Explore.step(s, 4.0).phase == :forward
    end

    test "is not judged during the grace period" do
      s = Explore.step(state(speed: 0.0, last: 0.0), 1.0)
      assert s.slow_since == nil
    end
  end

  test "settles, then backs off, then turns" do
    s = Explore.step(state(phase: :settle, phase_at: 2.0), 2.5)
    assert s.phase == :settle and Explore.drive(s) == {0.0, 0.0}

    s = Explore.step(s, 3.0)
    assert s.phase == :back and Explore.drive(s) == {-0.5, 0.0}

    s = Explore.step(s, 4.3)
    assert s.phase == :turn
    assert abs(s.turn_target) >= 0.8 and abs(s.turn_target) <= 3.1
  end

  describe "turn" do
    test "turns towards the target until the gyro says it got there" do
      s = state(phase: :turn, phase_at: 0.0, turn_target: -1.0, yaw_rate: -1.5, last: 0.0)
      assert Explore.drive(s) == {0.0, -1.5}

      s = Explore.step(s, 0.5)
      assert s.phase == :turn

      assert Explore.step(s, 0.7).phase == :forward
    end

    test "gives up on a turn that isn't happening" do
      s = state(phase: :turn, phase_at: 0.0, turn_target: 1.5, yaw_rate: 0.0, last: 0.0)
      assert Explore.step(s, 2.5).phase == :turn
      assert Explore.step(s, 3.1).phase == :forward
    end
  end
end
