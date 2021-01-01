# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2020-12-31
This entry will contain recent major changes since it is the start of the changelog file. 

### Added

- This changelog file
- Launch files for packages that did not have them, and added added command line arguments to launch certain configurations
- `spot_micro_rviz` package, which includes a urdf file defining the spot micro robot geometry. Currently this file and package will only be used for visualization. Not yet functional, reserved for future capability. 

### Changed
- Converted `i2c-pwmboard` package from a code copy to a git submodule

### Deprecated
- `spot_micro_simple_command` and `spot_micro_walk` python packages will be deprecated soon