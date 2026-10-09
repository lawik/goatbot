defmodule Goatbot.YawControllerTest do
  use ExUnit.Case, async: true

  alias BB.Message
  alias BB.Message.Geometry.Twist2D
  alias BB.Message.Sensor.JointState
  alias Goatbot.YawController

  @bias -0.6 * :math.pi() / 180

  defp state do
    %{
      mode: :disarmed,
      bias: 0.0,
      raw_rate: 0.0,
      sample_at: nil,
      heading: 0.0,
      still_rate: 0.05,
      bias_tau: 3.0
    }
  end

  # A gyro sample `at` seconds in.
  defp gyro(omega, at) do
    payload = %JointState{
      names: [:ground],
      positions: [0.0],
      velocities: [%Twist2D{vx: 0.0, vy: 0.0, omega: omega}],
      efforts: []
    }

    {:bb, [:sensor, :heading], %Message{payload: payload, monotonic_time: round(at * 1.0e9)}}
  end

  defp feed(state, samples) do
    Enum.reduce(samples, state, fn {omega, at}, state ->
      {:noreply, state} = YawController.handle_info(gyro(omega, at), state)
      state
    end)
  end

  test "learns the bias while standing still" do
    state = feed(state(), for(i <- 0..1000, do: {@bias, i / 100}))
    assert_in_delta state.bias, @bias, 0.0005
  end

  # The estimate starts at zero (see `init/1`), so a garbage sample is just one
  # outside `still_rate` of it, and is ignored rather than becoming the bias.
  test "a garbage sample from a freshly booted IMU is ignored" do
    # 500°/s, as seen on the robot.
    garbage = 500 * :math.pi() / 180
    state = feed(state(), [{garbage, 0.0} | for(i <- 1..1000, do: {@bias, i / 100})])

    assert_in_delta state.bias, @bias, 0.0005
  end
end
