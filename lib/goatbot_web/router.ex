defmodule GoatbotWeb.Router do
  use GoatbotWeb, :router

  pipeline :browser do
    plug(:accepts, ["html"])
    plug(:fetch_session)
    plug(:fetch_live_flash)
    plug(:put_root_layout, html: {GoatbotWeb.Layouts, :root})
    plug(:protect_from_forgery)
    plug(:put_secure_browser_headers)
  end

  scope "/", GoatbotWeb do
    pipe_through([:browser])
    live("/", WifiLive)
    live("/drive", DriveLive)
  end

  scope "/" do
    pipe_through([:browser])
    import BB.LiveView.Router
    bb_dashboard("/dashboard", robot: Goatbot.Robot)
  end
end
