import Config

config :phoenix, plug_init_mode: :runtime
config :logger, level: :warning

config :goatbot, GoatbotWeb.Endpoint,
  http: [ip: {127, 0, 0, 1}, port: 4002],
  secret_key_base: "62543H00STxPR2qbEG5Np9w160ydG58Xu+aWddfiSWZLo9CR/b+5VNFIOuLXqd9v",
  server: false
