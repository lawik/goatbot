defmodule GoatbotWeb.WifiLive do
  @moduledoc """
  Telling the robot about a network, from the network the robot made.

  This is the root page, because it is the first thing anybody needs and often
  the only thing that works: a robot fresh out of a burn is reachable over its
  own access point and nothing else. Driving and the robot dashboard are a link
  away from here rather than the other way round, so the panel only ever has to
  show a bare address.

  ## Joining cuts the ground from under this page

  There is one radio, so the moment a network is configured the access point
  goes away and takes this connection with it — whether or not the credentials
  were any good. The page cannot report what happened next, because it will not
  be there.

  So it says what is about to happen *before* it happens: the confirmation is
  rendered first and the configuration applied a beat later, which is the only
  ordering where the browser receives the message at all. What to expect
  afterwards is on screen, including how to get back if it doesn't work, which
  is to wait a minute for `BB.NSK.Network.Monitor` to give up and put the
  access point back.

  ## Scanning is a convenience, not the mechanism

  The radio does scan while it is an access point — confirmed on hardware — but
  the SSID is a text field first and the results are shortcuts that fill it in,
  because a hidden network never turns up in a scan and somebody has to be able
  to type one.
  """

  use GoatbotWeb, :live_view

  alias BB.NSK.Network

  @refresh :timer.seconds(2)

  # Long enough for LiveView to flush the confirmation to the browser before the
  # interface is reconfigured out from under it.
  @settle 500

  @impl Phoenix.LiveView
  def mount(_params, _session, socket) do
    if connected?(socket) do
      Network.scan()
      Process.send_after(self(), :refresh, @refresh)
    end

    {:ok, socket |> assign(ssid: "", passphrase: "", joining: nil) |> refresh(), layout: false}
  end

  @impl Phoenix.LiveView
  def handle_event("pick", %{"ssid" => ssid}, socket) do
    {:noreply, assign(socket, ssid: ssid)}
  end

  def handle_event("scan", _params, socket) do
    Network.scan()

    {:noreply, refresh(socket)}
  end

  def handle_event("join", %{"ssid" => ""}, socket), do: {:noreply, socket}

  def handle_event("join", %{"ssid" => ssid, "passphrase" => passphrase}, socket) do
    Process.send_after(self(), {:join, ssid, passphrase}, @settle)

    {:noreply, assign(socket, joining: ssid)}
  end

  def handle_event("forget", _params, socket) do
    Network.forget()

    {:noreply, refresh(socket)}
  end

  @impl Phoenix.LiveView
  def handle_info({:join, ssid, passphrase}, socket) do
    Network.join(ssid, passphrase)

    {:noreply, socket}
  end

  def handle_info(:refresh, socket) do
    Process.send_after(self(), :refresh, @refresh)

    {:noreply, refresh(socket)}
  end

  def handle_info(_message, socket), do: {:noreply, socket}

  defp refresh(socket) do
    assign(socket,
      mode: Network.mode(),
      connected?: Network.connected?(),
      configured_ssid: Network.configured_ssid(),
      own_ssid: Network.ssid(),
      passphrase_hint: Network.passphrase(),
      url: Network.url(),
      access_points: Network.access_points()
    )
  end

  @impl Phoenix.LiveView
  def render(assigns) do
    ~H"""
    <div class="min-h-screen bg-zinc-900 px-4 py-6 font-mono text-zinc-100">
      <div class="mx-auto max-w-md space-y-6">
        <header class="space-y-3">
          <h1 class="text-lg">{@own_ssid}</h1>
          <nav class="grid grid-cols-2 gap-3 text-center text-sm">
            <a href="/drive" class="rounded border border-zinc-700 px-3 py-2 text-emerald-400">
              drive
            </a>
            <a href="/dashboard" class="rounded border border-zinc-700 px-3 py-2 text-emerald-400">
              dashboard
            </a>
          </nav>
        </header>
        <div :if={@joining} class="rounded border border-amber-600 bg-amber-950/40 p-4 text-sm">
          <p class="font-bold">Joining {@joining}.</p>
          <p class="mt-2">
            This page is about to go away — the robot has one radio, so its own network stops the moment it joins yours. Reconnect your phone or laptop to {@joining} and look for the robot there.
          </p>
          <p class="mt-2">
            If it never turns up, wait a minute. The robot gives the network a minute to work, then puts 
            <span class="font-bold">{@own_ssid}</span>
            back so you can try again.
          </p>
        </div>
        <div :if={!@joining} class="space-y-6">
          <section class="rounded border border-zinc-700 p-4 text-sm">
            <p>
              <span class="text-zinc-400">state</span>
              <span class="float-right">{state(assigns)}</span>
            </p>
            <p :if={@configured_ssid} class="mt-1">
              <span class="text-zinc-400">network</span>
              <span class="float-right">{@configured_ssid}</span>
            </p>
            <p :if={@mode == :access_point} class="mt-1">
              <span class="text-zinc-400">passphrase</span>
              <span class="float-right">{@passphrase_hint}</span>
            </p>
            <p :if={@mode == :access_point} class="mt-1">
              <span class="text-zinc-400">address</span>
              <span class="float-right">{@url}</span>
            </p>
          </section>
          <form phx-submit="join" class="space-y-3">
            <label class="block text-sm text-zinc-400" for="ssid">network name</label>
            <input
              id="ssid"
              name="ssid"
              value={@ssid}
              autocapitalize="off"
              autocorrect="off"
              spellcheck="false"
              class="w-full rounded border border-zinc-700 bg-zinc-800 px-3 py-2"
            />
            <label class="block text-sm text-zinc-400" for="passphrase">
              passphrase 
              <span class="text-zinc-600">(blank for an open network)</span>
            </label>
            <input
              id="passphrase"
              name="passphrase"
              type="password"
              value={@passphrase}
              class="w-full rounded border border-zinc-700 bg-zinc-800 px-3 py-2"
            />
            <button
              type="submit"
              disabled={@ssid == ""}
              class="w-full rounded bg-emerald-600 px-3 py-2 disabled:bg-zinc-700 disabled:text-zinc-500"
            >
              join
            </button>
          </form>
          <section class="space-y-2">
            <div class="flex items-baseline justify-between text-sm text-zinc-400">
              <span>networks nearby</span>
              <button phx-click="scan" class="text-emerald-400 underline">scan</button>
            </div>
            <p :if={@access_points == []} class="text-sm text-zinc-500">
              Nothing yet. Scan again, or type the name in above — a hidden network won't appear here however long you wait.
            </p>
            <button
              :for={access_point <- @access_points}
              phx-click="pick"
              phx-value-ssid={access_point.ssid}
              class="flex w-full items-baseline justify-between rounded border border-zinc-700 px-3 py-2 text-left text-sm"
            >
              <span>{access_point.ssid}</span>
              <span class="text-zinc-500">{access_point.signal_percent}%</span>
            </button>
          </section>
          <button
            :if={@configured_ssid}
            phx-click="forget"
            class="w-full rounded border border-zinc-700 px-3 py-2 text-sm text-zinc-400"
          >
            forget {@configured_ssid} and go back to {@own_ssid}
          </button>
        </div>
      </div>
    </div>
    """
  end

  defp state(%{mode: :access_point}), do: "its own network"
  defp state(%{mode: :unconfigured}), do: "starting up"
  defp state(%{connected?: true}), do: "connected"
  defp state(_connecting), do: "connecting"
end
