# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2021-1-07

### Added
- Publishing of robot state and joints via tf2
- Open loop calculated odometry by integrating rate commands 
- Additional documentation about hardware
- Links to 3d printed parts for mounting lidar scanner

### Changed
- Merged alternate gait into master, configurable by parameters

#### Deprecated
- Deprecating alternate-gait branch

## [0.2.0] - 2021-1-01

### Changed
- Changed robot velocity command from a Vector3 message on a `speed_cmd` topic, to the more ROS conventional Twist message on a `cmd_vel` topic._ This affected `spot_micro_motion_cmd`, `spot_micro_keyboard_command`, and `lcd_monitor` packages

### Removed
- Removed deprecated `spot_micro_walk` and `spot_micro_simple_command` python pacakges, as they are obsolete

## [0.1.0] - 2020-12-31

### Added
- This changelog file
- Launch files for packages that did not have them, and added added command line arguments to launch certain configurations
- `spot_micro_rviz` package, which includes a urdf file defining the spot micro robot geometry. Currently this file and package will only be used for visualization. Not yet functional, reserved for future capability. 

### Changed
- Converted `i2c-pwmboard` package from a code copy to a git submodule

### Deprecated
- `spot_micro_simple_command` and `spot_micro_walk` python packages will be deprecated soon
