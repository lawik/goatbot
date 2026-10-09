defmodule Goatbot.Application do
  # See https://elixir.hexdocs.pm/Application.html
  # for more information on OTP Applications
  @moduledoc false

  use Application

  @impl true
  def start(_type, _args) do
    # Hardware links come up before the robot, so its components find them.
    # The endpoint comes last: children stop in reverse order, so the robot is
    # disarmed before the dashboard watching it goes away.
    children =
      target_children() ++
        [
          GoatbotWeb.Telemetry,
          {Phoenix.PubSub, name: Goatbot.PubSub},
          {Goatbot.Robot, robot_opts()},
          # Drive commands from the badge, over distribution.
          Goatbot.Remote,
          GoatbotWeb.Endpoint
        ]

    # See https://elixir.hexdocs.pm/Supervisor.html
    # for other strategies and supported options
    opts = [strategy: :one_for_one, name: Goatbot.Supervisor]
    Supervisor.start_link(children, opts)
  end

  # List all child processes to be supervised
  if Mix.target() == :host do
    defp target_children() do
      [
        # Children that only run on the host during development or test.
        # In general, prefer using `config/host.exs` for differences.
        #
        # Starts a worker by calling: Host.Worker.start_link(arg)
        # {Host.Worker, arg},
      ]
    end
  else
    defp target_children() do
      [
        # Children for all targets except host
        # Starts a worker by calling: Target.Worker.start_link(arg)
        # {Target.Worker, arg},

        # Joins a configured wifi network, or brings up the robot's own access
        # point when there is none. Ahead of everything so the board is
        # reachable even when the robot fails to start.
        BB.NSK.Network.Monitor,

        # The STM32 power-management coprocessor on UART2: battery telemetry
        # and the three buttons. `Goatbot.Sensor.Power` puts both on PubSub.
        %{
          id: Goatbot.STM32,
          start: {__MODULE__, :start_stm32, [[port: "ttyS2"]]}
        },

        # The Erlang node the badge drives the robot through, named for the wifi address.
        Goatbot.Distribution
      ]
    end
  end

  require Logger

  @doc false
  # Battery telemetry is a nice-to-have: if the UART won't open, log it and
  # carry on rather than failing the application and taking the robot with it.
  def start_stm32(opts) do
    case Goatbot.STM32.start_link(opts) do
      {:ok, pid} ->
        {:ok, pid}

      {:error, reason} ->
        Logger.warning("STM32 coprocessor not available: #{inspect(reason)}")
        :ignore
    end
  end

  defp robot_opts do
    if System.get_env("SIMULATE") do
      [simulation: :kinematic]
    else
      Application.get_env(:goatbot, Goatbot.Robot, [])
    end
  end
end
