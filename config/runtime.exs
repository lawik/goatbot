import Config

Application.ensure_all_started(:logger_backends)
LoggerBackends.add(RingLogger)

# On the board the endpoint serves the drive pad on port 80 and starts
# itself — there is no `mix phx.server` on a device. `check_origin` is off
# because the robot is reached by IP, on its own access point as often as
# not, so there is no origin to check against.
if config_target() != :host do
  config :goatbot, GoatbotWeb.Endpoint,
    http: [ip: {0, 0, 0, 0, 0, 0, 0, 0}, port: 80],
    server: true,
    check_origin: false,
    # A fresh one each boot, so no secret lives in the repository. It only
    # signs session cookies, which costs nothing more than a reconnect.
    secret_key_base: Base.encode64(:crypto.strong_rand_bytes(48))
end
