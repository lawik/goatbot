defmodule Goatbot.MixProject do
  use Mix.Project

  @app :goatbot
  @version "0.1.0"
  @all_targets [:trellis]

  def project do
    [
      app: @app,
      version: @version,
      elixir: "~> 1.20",
      elixirc_paths: elixirc_paths(Mix.env()),
      archives: [nerves_bootstrap: "~> 1.17"],
      listeners: listeners(Mix.target(), Mix.env()),
      start_permanent: Mix.env() == :prod,
      deps: deps(),
      releases: [{@app, release()}]
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger, :runtime_tools],
      mod: {Goatbot.Application, []}
    ]
  end

  # Test fakes (e.g. Goatbot.STM32.FakeUART) are compiled only for tests.
  defp elixirc_paths(:test), do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]

  def cli do
    [preferred_targets: [run: :host, test: :host]]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:rustler, ">= 0.0.0", optional: true},
      {:circuits_i2c, "~> 2.1"},
      {:circuits_gpio, "~> 2.1"},
      # The STM32 power-management coprocessor on ttyS2: battery and buttons.
      {:circuits_uart, "~> 1.5"},
      {:nsk, github: "protolux-electronics/nsk", targets: :host, runtime: false},
      {:bb_nsk, "~> 0.1"},
      {:igniter, "~> 0.6", only: [:dev, :test]},
      # Dependencies for all targets
      {:nerves, "~> 1.13", runtime: false},
      {:shoehorn, "~> 0.9.1"},
      {:logger_backends, "~> 1.0"},
      {:ring_logger, "~> 0.11.0"},
      {:toolshed, "~> 0.5.0"},
      {:breeze, "~> 0.5.1"},

      # Allow Nerves.Runtime on host to support development, testing and CI.
      # See config/host.exs for usage.
      {:nerves_runtime, "~> 0.13.12"},

      # Dependencies for all targets except :host
      {:nerves_pack, "~> 0.7.1", targets: @all_targets},
      # Dependencies for specific targets
      # NOTE: It's generally low risk and recommended to follow minor version
      # bumps to Nerves systems. Since these include Linux kernel and Erlang
      # version updates, please review their release notes in case
      # changes to your application are needed.
      {:nerves_system_trellis, "~> 0.5", runtime: false, targets: :trellis}
    ]
  end

  def release do
    [
      overwrite: true,
      # Erlang distribution is not started automatically.
      # See https://nerves-pack.hexdocs.pm/readme.html#erlang-distribution
      cookie: "#{@app}_cookie",
      include_erts: &Nerves.Release.erts/0,
      steps: [&Nerves.Release.init/1, :assemble],
      strip_beams: Mix.env() == :prod or [keep: ["Docs"]]
    ]
  end

  # Uncomment the following line if using Phoenix > 1.8.
  # defp listeners(:host, :dev), do: [Phoenix.CodeReloader]
  defp listeners(_, _), do: []
end
