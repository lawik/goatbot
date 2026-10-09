# Emerge only exists on the target (see `mix.exs`), so on the host this module
# isn't compiled and `Goatbot.Face` has nothing to draw with.
if Code.ensure_loaded?(Emerge.UI) do
  defmodule Goatbot.Face.Screen do
    @moduledoc """
    The face, as an Emerge tree the size of the panel: one expression in big
    text, centred.

    Spleen's 16x32 outline font, so at 192 px each glyph is the bitmap's pixels
    drawn six times over and stays crisp on a one-bit panel. Three characters at
    96 px apiece fill 288 of the panel's 400 px, and 192 of its 300.
    """

    use Emerge.UI

    @size 192

    @spec screen(String.t()) :: Emerge.tree()
    def screen(face) do
      el(
        [width(fill()), height(fill()), Font.color(color_rgb(0, 0, 0))],
        el([center_x(), center_y(), Font.family("Spleen Title"), Font.size(@size)], text(face))
      )
    end
  end
end
