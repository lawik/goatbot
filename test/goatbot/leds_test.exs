defmodule Goatbot.LedsTest do
  use ExUnit.Case, async: true

  alias Goatbot.Leds

  test "keeps the stock colours" do
    for state <- [:disarmed, :idle, :balancing, :fallen, :disarming, :error] do
      assert Leds.colour(state) == BB.NSK.Leds.Controller.colour(state)
    end
  end

  test "lifted has a colour of its own" do
    refute Leds.colour(:lifted) == Leds.colour(:some_unknown_state)
    refute Leds.colour(:lifted) == Leds.colour(:fallen)
  end
end
