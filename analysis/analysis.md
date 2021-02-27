# Analysis

This directory contains analysis work related to the spot micro quadruped robot project, such as plotting and analysis scripts.

This file documents various analysis efforts.

## Reprocessing a Log
Basic idea of reprocessing is playback a recorded dataset while running another ROS package that does something with the data. Complicating details are that certain ROS channels may need to be filtered out of playback.

A command like the one below will playback a ros bag while filtering certain channels and messages that meet certain criteria. In the example below speciifcally, all messages other than `tf` will be played back, while `tf` messages that have a parent frame ID of odom or map will not be published.

```
rosbag filter <original_bag_file.bag> <output_bag_filename.bag> "topic!='/tf' or m.transforms[0].header.frame_id!='odom' or m.transforms[0].header.frame_id!='map
```

## Autonomous Navigation 

### Odometry
Autonomous Navigation packages in ROS require reasonable odometry for localization and other purposes. My outputted robot odometry is merely an integration of velocity commands and is totally bogus.

I need to try out some potential ROS packages that could effectively output odometry, primarily derived from lidar scans. An example is using ROS packages that do lidar scan matching to calculate odometry.

There are several potential ROS packages. To evaluate which may work best, want to reprocess a data log which originally had hector slam running, and run it through some of these packages and save off their calculated odometry. Then analyze and compare teh results and choose the best.


**Possible Packages to Evaluate:**
- rf2o_laser_odometry




#### rf2o_laser_odometry
Instructions to reprocess:
Install rf2o_laser_odometry package into ros workspace.
Run below to create a filtered bagfile. 
```
rosbag filter <original_bag_file.bag> <output_bag_filename.bag> "topic!='/tf' or m.transforms[0].header.frame_id!='odom' or m.transforms[0].header.frame_id!='map
```
Playback filtered bagfile, and launch laser odometry package:
`rosbag play <path/to/bagfile>`
`roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch`

Visualize results in RVIZ
`roslaunch spot_micro_rviz slam.launch`

TODO: Create python script to extract odometry tracks from seperate logs and plot them together. See remove_tf.py for example code to interace with bag files through python
(https://github.com/srv/srv_tools/blob/kinetic/bag_tools/scripts/remove_tf.py). Note, don't use remove_tf to try and remove tf messages from a bag file as it seems to screw up static transforms. 


