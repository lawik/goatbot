# Used by "mix format"
[
  inputs: [
    "{mix,.formatter}.exs",
    "{config,lib,test}/**/*.{ex,exs}",
    "rootfs_overlay/etc/iex.exs"
  ],
  plugins: [Breeze.HTMLFormatter],
  import_deps: [:bb_liveview, :phoenix, :bb_parameter_store_cubdb, :bb, :bb_nsk, :breeze]
]
