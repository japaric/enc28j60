# `enc28j60`

> A platform agnostic driver to interface with the [ENC28J60][] (Ethernet controller)

[ENC28J60]: http://www.microchip.com/wwwproducts/en/en022889

## [API documentation](https://docs.rs/enc28j60)

## Examples

You should find some examples in the [`stm32f103xx-hal`] crate.

[`stm32f103xx-hal`]: https://github.com/japaric/stm32f103xx-hal/tree/master/examples

The smoltcp example can be flashed & run with [probe-run](https://github.com/knurling-rs/probe-run/)
via `cargo r --release --target thumbv7m-none-eabi --example smoltcp --features=smoltcp`
using the configuration in `.cargo/config.toml`.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
