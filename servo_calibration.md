# Servo Calibration Guide

This document provides a comprehensive guide to calibrate servos on a spot micro frame and create a cooresponding servo configuration ditionary the ROS control software requires. A spreadhseet named `servo_calibration_spreadsheet.ods` is included in this repo as an aid for calculating the servo configuration values.

The servo configuration dictionary is contained within the configuration file `spot_micro_motion_cmd.yaml` and holds servo configuration values as shown below:
```yaml
num_servos: 12
servo_max_angle_deg: 82.5
RF_3: {num: 1,  center: 306,range: 385,direction:  1, center_angle_deg:  84.0}
RF_2: {num: 2,  center: 306,range: 385,direction:  1, center_angle_deg: -27.9}
RF_1: {num: 3,  center: 306,range: 396,direction: -1, center_angle_deg:  -5.4}
RB_3: {num: 4,  center: 306,range: 394,direction:  1, center_angle_deg:  90.4}
.
.
.
```

Servo's are defined by a abbreviation and number referring to their location and position within a leg. "RF" cooresponds to right front, "RB" to right back, similarly "LF" and "LB" to the left front and left back legs. A number 1 cooresponds to link 1 (the hip joint), 2 to link 2 (the upper leg, or shoulder joint), and 3 to link 3 (the lower leg or knee joint).

### Description of Servo Configuration Values
* **num_servos**: Fixed at 12 for this spot micro platform

* **servo_max_angle_deg**: Maximum permissible command angle in a single direction for every servo. Robotic servos usually have a total movement range of 180 deg (i.e. +/- 90 deg in each direction from center). The maximum value for this limit is usually 90 deg, however it is useful to slightly constrain it to account for errors in servo center position, potential performance loss near the edges of a servo's travel, or to avoid mechanical limits or restrictions. A value of 82.5 deg works for my robot.

* **num:** Port on the PCA9685 board (numbered 1-16) which that servo is connected to

* **center:** The "raw" pwm value the PCA9685 node recieves representing the servo center position. A servo's position is generally commanded by a 1 to 2 ms pulse in a 50 hz cycle (20 ms time period). The PCA9685 board and cooresponding ros node controls the length of this pulse via a 12 bit pwm value. Assuming a 20ms total cycle length, 0 means no pulse, 2^12=4096 means a constant high signal, and 2048 for example would mean a pulse length of 10ms. A servo's center position is usually positioned with a 1.5 ms pulse which cooresponds to a pwm value of about 307. This value can be fine tuned by moving a sample servo with a horn on it via the servo keyboard move node and finding the center point from where the servo moves equally in opposite directions (say +/- 90 deg equally from the center. The value of 307 is a good starting point, and the value should be the same for all the servos in a set  of the same kind.

* **range:** The "raw" pwm value that cooresponds to the max end to end movement range for a servo. This will be related to the servo_max_angle_deg defined in the config file. As an example, the servo range value would be the raw pwm value for a servo position at +80 deg  minus the raw pwm value for a servo position at -80 deg . The spreadsheet in this repo calculates this range value.

* **direction:** A value of 1 or -1. Reverses the direction or servo rotation if needed, dependent on the coordinate axes of a specific leg joint. Computed in the servo calibration spreadsheet.

* **center_angle_deg:** The value in degrees, in a leg joint's specific coordinate system, that cooresponds to a servo's position when at it's "center" pwm value. Value calculated in the servo calibration spreadsheet.


### Servo Installation
It is reccomended to install servos in the spot micro frame when they are powered and commanded to their center position. The joint which a servo is installed in should be approximately positioned at it's "nuetral" stance, the position about which most leg motion will occur. This ensures maximum servo travel will be available around the typical joint command angles. The two figures below roughly depict the joint orientations for which servo's should be installed when at their center position. 

![Side View Neutral Positions](assets/1_robot_right_links.png)
![Back View Neutral Positions](assets/12_robot_back_overview.png)


### Commanding Individual Servos for Calibration
Individual servo's can be commanded via the servo_move_keyboard ROS node for the calibration procedure. After one or more servos are connected to the PCA9685, the steps for commanding a single servo are as follows:

1. On the raspberry pi, start the i2cpwm_board node:
`rosrun i2cpwm_board i2cpwm_board`
2. On another terminal on the raspberry pi, or on a linux machine with ROS installed and capabile of communicating with the raspberry pi, run the servo_move_keyboard ros node:
`rosrun servo_move_keyboard servoMoveKeyboard.py`
3. After the descriptive prompt appears, type `oneServo` to enter the one servo command node.
4. Select the servo to command by entering the integer number cooresponding to the PCA9685 port to command, for example, `2`. After a servo is selected, all other servos are commanded to idle (or freewheel) such that they can be moved by hand. The following prompt will appear:

![Servo move prompt](assets/servo_move_prompt.png)
5. Use the key `y` to command the default center value of the selected servo (pwm = 306). Use the keys `g` and `j` to decrease or increase the servo pwm command value by 1, respectively. This moves the servo in fine increments. The current pwm value is printed in the terminal. Use the `f` and `k` keys to move the servo in coarser increments. 
    * Key's `t` and `u` can be used to quickly command the servo to it's min and max values, respectively (by default 83 and 520, but these can be updated per servo by the keys specified in the instrucitonal prompt).
6. After commanding a servo to desired calibration positions and noting down values in the calibration spreadsheet, exit the one servo control mode by pressing `q`
7. Go back to step 3 to repeat the process for another servo.

### Guide to Creating Servo Calibration Values
Adequate kinematic performance can be achieved through calibration of links by eye, as depicted in the diagrams below. However, I also used a smartphone inclinometer app as an aid to measure angles of 45 deg for the link 1 calibration. It is reccomended to prop up the robot on a box or similar test stand so the legs can hang freely for calibration.

**Leg links should be visualized as straight lines segments from reference point to reference point (e.g. axis of rotation to axis of rotation).** When positioning joints for calibration, use the wireframe representation of each link for orientation reference rather than the the 3d printed part itself.

The servo calibration can be built up with aid of the servo_calibration_spreadsheet, shown in the figure below. Generally the procedure is the position each link to two reference positions (angles) and note down the cooresponding pwm values for those positions. I used gross angles of 0 and 90 deg for ease of placement by eye. While the slope value is not used, it is useful to keep an eye on it as all servos should have similar calculated slopes. Large discrepencies in this value could be indicative of errors.

![Servo Calibration Spreadsheet](assets/servo_calibration_spreadsheet.png)

#### Right Side Legs Links 2 and 3 Calibration

**Consistency in the following steps is more important than accuracy.**

Start with calibrating links 2 and 3 for all legs, starting with the right side of the robot. The figure below depicts the coordinate systems of the links of the right side legs positioned at 0 degrees, and defines the positive and negative angle directions.

![Right side zero degree positions](assets/2_right_straight_links.png)

Note that link 3's angles are relative to link 2, as exemplified by the figure below:
![Example right side link 3 angles ](assets/3_right_link_angles_example.png)

Starting with link 2 (ignoring the orientation of link 3), command link 2 to positions of 0 and -90 degrees, and note the cooresponding PWM values in the spreadsheet. These two positions are shown in the following two figures.

![Right side link 2 step 1 ](assets/4_right_link2_config_step_1.png)
![Right side link 2 step 2 ](assets/4_right_link2_config_step_2.png)

Next, move onto link 3. For ease of visualizing reference orientation, manually position link 2 in a vertical orientation. Command link 3 to 0 and +90 degree positions and record the cooresponding pwm values. The two positions of link 3 are shown in the following two figures.

![Right side link 3 step 1 ](assets/5_right_link3_config_step_1.png)
![Right side link 3 step 2 ](assets/6_right_link3_config_step_2.png)

Repeat this process for the other right side leg.

#### Left Side Legs Links 2 and 3 Calibration
Next repeat this process for the legs on the left side of the robot. **Note that** the coordinate systems for the left legs have different directions for positive and negative angles. These are depicted in the figure below for completeness.

![Left side overview ](assets/7_robot_left_overview.png)

Repeat the calibration process for link 2 on the left side legs. Position link 2 to 0 and +90 degree positions, and record the PWM values in the spreadsheet. The two positions are shown in the following two figures.

![Left side link 2 step 1 ](assets/8_left_link2_config_step_1.png)
![Left side link 2 step 2 ](assets/9_left_link2_config_step_2.png)

Repeat the calibration process for link 3 on the left side legs. Manually position link 2 vertically, then position link 3 at 0 and -90 degrees and record the cooresponding pwm values in the spreadsheet. The two positions are shown below.

![Left side link 3 step 1 ](assets/10_left_link3_config_step_1.png)
![Left side link 3 step 2 ](assets/11_left_link3_config_step_2.png)

Repeat the process for both left side legs.

#### Left and Right Legs Link 1 Calibration

Finally, calibrate link 1 for the left and right legs in a similar pattern. The coordinate systems for link 1 of the left and right legs is shown in the figures below.

![Back view Coordinate Systems link 1 ](assets/12_robot_back_overview.png)
![Back view Coordinate Systems link 1 ](assets/13_robot_back_angle_directions.png)

Start with the right legs, and command link 1 to 0 and -45 degrees, and record the cooresponding PWM values in the spreadsheet. A reference value of 45 degrees is used instead of 90 due to mechanical limits. Repeat the process for the left legs. These positions are shown in the four figures below. 


![Right leg link 1 step 1](assets/14_right_link1_config_step_1.png)
![Right leg link 1 step 2](assets/15_right_link1_config_step_2.png)

![Left leg link 1 step 1](assets/16_left_link1_config_step_1.png)
![Left leg link 1 step 2](assets/17_left_link1_config_step_2.png)

This complets the leg servo calibration process. Take the **bold** values from the servo calibration spreadsheet and copy them to the servo configuration dictionaries in the `spot_micro_motion_cmd.yaml` config file.