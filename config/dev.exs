import Config

config :phoenix, stacktrace_depth: 20, plug_init_mode: :runtime
config :logger, default_formatter: [format: "[$level] $message\n"]
config :goatbot, dev_routes: true

config :goatbot, GoatbotWeb.Endpoint,
  http: [ip: {0, 0, 0, 0}, port: String.to_integer(System.get_env("PORT", "4000"))],
  check_origin: false,
  secret_key_base: "qMTWJal0iiE5eWzrPzx9x+Ke/jPxe4p7OjDcimIzrhQjBu+V4onCfDIA3dWGqqED",
  watchers: [],
  live_reload: []

# Only on a developer's machine. A Nerves firmware is built out of `:dev`,
# and a board has no asset toolchain to run and no source tree to watch.
if Mix.target() == :host do
  config :goatbot, GoatbotWeb.Endpoint,
    code_reloader: true,
    debug_errors: true,
    watchers: [
      esbuild: {Esbuild, :install_and_run, [:goatbot, ["--sourcemap=inline", "--watch"]]},
      tailwind: {Tailwind, :install_and_run, [:goatbot, ["--watch"]]}
    ],
    live_reload: [
      web_console_logger: true,
      patterns: [
        ~r"priv/static/(?!uploads/).*\.(js|css|png|jpeg|jpg|gif|svg)$"E,
        ~r"lib/.*_web/router\.ex$"E,
        ~r"lib/.*_web/(controllers|live|components)/.*\.(ex|heex)$"E
      ]
    ]
end
