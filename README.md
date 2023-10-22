# imxrt-enet

An Ethernet driver for i.MX RT MCUs with support for `smoltcp`.

## Development

To build the driver in this repo, enable

- an `imxrt-ral` feature.
- a `smoltcp` socket feature.

For example,

```
cargo build --features=imxrt-ral/imxrt1062,smoltcp/socket-raw
```

If you're depending on this driver, you're expected to enable similar features
somewhere in your dependency graph.

To test the driver on hardware, see the various ENET examples maintained with
[`imxrt-hal`].

[`imxrt-hal`]: https://github.com/imxrt-rs/imxrt-hal
