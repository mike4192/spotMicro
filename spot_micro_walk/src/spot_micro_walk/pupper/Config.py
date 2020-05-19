import numpy as np
#from pupper.HardwareConfig import MICROS_PER_RAD, NEUTRAL_ANGLE_DEGREES, PS4_COLOR, PS4_DEACTIVATED_COLOR
from enum import Enum

class Configuration:
    def __init__(self):

        #################### COMMANDS ####################
        self.max_x_velocity = 0.4
        self.max_y_velocity = 0.3
        self.max_yaw_rate = 2.0
        #self.max_pitch = 30.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.03  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 2.0
        
        self.body_shift_x = 0.035 # Distance body shifts forward in x direction during stance phase
        self.body_shift_y = 0.025 # Distance body shifts sideways during stance phase

        #################### STANCE ####################
        self.delta_x = 0.1
        self.delta_y = 0.09
        self.x_shift_front = 0.02
        self.x_shift_back = -0.03
        self.default_z_ref = -0.18

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.045
        self.alpha = (
            1.0  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            1.0  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        # I think the order is:
        # rightfront, leftfront, rightback, leftback
        
        # Contact phases:
        # 2: Leg stationary
        # 1: Moving stance forward
        # 0: leg swing
        #-1: Moving stance backwards

        self.dt = 0.02
        self.num_phases = 8
        self.contact_phases = np.array(
            [[1, 2, -1, 0, 1, 2, -1, 2],
             [1, 2, -1, 2, 1, 2, -1, 0],
             [1, 0, -1, 2, 1, 2, -1, 2],
             [1, 2, -1, 2, 1, 0, -1, 2]]
        )
        self.overlap_time = (
            1.2  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            1.2  # duration of the phase when only two feet are on the ground
        )

    @property
    def default_stance(self):
        # I think the order is:
        # rightfront, leftfront, rightback, leftback
        return np.array(
            [
                [
                    self.delta_x + self.x_shift_front,
                    self.delta_x + self.x_shift_front,
                    -self.delta_x + self.x_shift_back,
                    -self.delta_x + self.x_shift_back,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z
        # b_z = np.array([0, 0, 0, 0, self.__z_clearance])
        # A_z = np.array(
        #     [
        #         [0, 0, 0, 0, 1],
        #         [1, 1, 1, 1, 1],
        #         [0, 0, 0, 1, 0],
        #         [4, 3, 2, 1, 0],
        #         [0.5 ** 4, 0.5 ** 3, 0.5 ** 2, 0.5 ** 1, 0.5 ** 0],
        #     ]
        # )
        # self.z_coeffs = solve(A_z, b_z)

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 1 * self.overlap_ticks 

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 4 * self.overlap_ticks + 4 * self.swing_ticks

