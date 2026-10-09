# This file is responsible for configuring your application and its
# dependencies.
#
# This configuration file is loaded before any dependency and is restricted to
# this project.
import Config

config :tailwind,
  version: "4.1.12",
  goatbot: [
    args: ~w(
    --input=assets/css/app.css
    --output=priv/static/assets/css/app.css
  ),
    cd: Path.expand("..", __DIR__)
  ]

config :esbuild,
  version: "0.25.4",
  goatbot: [
    args:
      ~w(js/app.js --bundle --target=es2022 --outdir=../priv/static/assets/js --external:/fonts/* --external:/images/*),
    cd: Path.expand("../assets", __DIR__),
    env: %{"NODE_PATH" => [Path.expand("../deps", __DIR__), Mix.Project.build_path()]}
  ]

config :phoenix, json_library: Jason

config :logger,
  default_formatter: [format: "$time $metadata[$level] $message\n", metadata: [:request_id]]

config :goatbot, GoatbotWeb.Endpoint,
  url: [host: "localhost"],
  adapter: Bandit.PhoenixAdapter,
  render_errors: [
    formats: [json: GoatbotWeb.ErrorJSON],
    layout: false
  ],
  pubsub_server: Goatbot.PubSub,
  live_view: [signing_salt: "lWkClIY5"]

config :bb_nsk, name: "goatbot"

# Breeze (the `Goatbot.Dashboard` TUI) sizes its render cache from :os_mon when
# it's running. It isn't here, so give it a fixed budget that suits the board.
config :back_breeze, render_cache_max_memory_bytes: 16 * 1_024 * 1_024

# Enable the Nerves integration with Mix
Application.start(:nerves_bootstrap)

# Customize non-Elixir parts of the firmware. See
# https://nerves.hexdocs.pm/advanced-configuration.html for details.

config :nerves, :firmware,
  rootfs_overlay: "rootfs_overlay",
  provisioning: "config/provisioning.conf"

# Set the SOURCE_DATE_EPOCH date for reproducible builds.
# See https://reproducible-builds.org/docs/source-date-epoch/ for more information

config :nerves, source_date_epoch: "1791461140"

if Mix.target() == :host do
  import_config "host.exs"
else
  import_config "target.exs"
end

import_config "#{config_env()}.exs"
