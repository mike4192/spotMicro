# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2020-12-31
This report will break order and capture other recent changes in last few days for the sake of capturing major recent changes.

### Added
- Launch files for packages that did not have them
- `spot_micro_rviz` package, which includes a urdf file defining the spot micro robot geometry. Currently this file and package will only be used for visualization. Not yet functional, reserved for future capability. 

### Changed
- Changed robot velocity command from a Vector3 message on a `speed_cmd` topic, to the more ROS conventional Twist message on a `cmd_vel` topic._ This affected `spot_micro_motion_cmd`, `spot_micro_keyboard_command`, and `lcd_monitor` packages
- Changed launch file for `spot_micro_motion_cmd` and added command line arguments to launch certain configurations
- Converted `i2c-pwmboard` package from a code copy to a git submodule

### Removed
- Removed deprecated `spot_micro_walk` and `spot_micro_simple_command` python pacakges, as they are obsolete

## [0.1.0] - 2020-12-31

### Added
- This changelog file

### Deprecated
- `spot_micro_simple_command` and `spot_micro_walk` python packages will be deprecated soon
