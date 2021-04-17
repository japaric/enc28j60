# `enc28j60`

> A platform agnostic driver to interface with the [ENC28J60][] (Ethernet controller)

[ENC28J60]: http://www.microchip.com/wwwproducts/en/en022889

## [API documentation](https://docs.rs/enc28j60)

## Example

Example program on a STM32F1xx (bluepill) using a ST-Link V2.  
See the [Rust Embedded Book](https://docs.rust-embedded.org/book/intro/install.html) or [STM32F1xx-hal](https://github.com/stm32-rs/stm32f1xx-hal) for setup details.

Attach OpenOCD to dongle:

```sh
openocd -f interface\stlink.cfg -f target\stm32f1x.cfg
```

Flash and Run:
```sh
cargo run --features=smoltcp --example smoltcp
```

Ping or navigate a web-browser to the IP address in `examples/smoltcp.rs`.

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
