# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- An `Enc28j60::next_packet` API to process new packets in two stages.
  `next_packet` returns a `Packet` proxy; the proxy can be dropped (e.g. if
  it's too big for the available memory) or read into memory.

### Changed

- `Enc28j60::transmit` now immediately flushes the packet. Meaning that
  `transmit` will block until the packet has been sent.

### Removed

- [breaking-change] `Enc28j60::receive` has been removed in favor of the
  `Enc28j60::next_packet` API.

- [breaking-change] `Enc28j60::flush` has been removed. It was found that
  starting a transmission prevents EPKTCNT from increasing; as EPKTCNT indicates
  whether a packet is ready to be read this results in a deadlock: `transmit` ->
  `flush` -> `receive` was OK, but `transmit` -> `receive` would always
  deadlock.

- [breaking-change] `IntPin` and `ResetPin` have been removed from the public
  API; they were implementation details.

## [v0.2.1] - 2019-01-30

### Fixed

- Properly work around silicon Erratum #1. Some devices would get stuck while
  executing `Enc28j60::new` due to a silicon bug because the documented
  workaround was not properly applied; this has been fixed.

## [v0.2.0] - 2018-05-16

### Changed

- [breaking-change] moved the `embedded-hal` v0.2.x

## v0.1.0 - 2018-03-13

Initial release

[Unreleased]: https://github.com/japaric/enc28j60/compare/v0.2.1...HEAD
[v0.2.1]: https://github.com/japaric/enc28j60/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/japaric/enc28j60/compare/v0.1.0...v0.2.0
