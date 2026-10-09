import Config

config :logger, level: :info

# No `force_ssl` here, unlike a server: the robot serves plain HTTP on port 80,
# reached by IP or on its own access point, and has no certificate to offer.
# The secret is generated at boot; see `config/runtime.exs`.
