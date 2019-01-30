# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

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

[Unreleased]: https://github.com/japaric/enc28j60/compare/v0.2.0...HEAD
[v0.2.0]: https://github.com/japaric/enc28j60/compare/v0.1.0...v0.2.0
