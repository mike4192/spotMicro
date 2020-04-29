import numpy as np
#from transforms3d.euler import euler2mat
from ..spot_micro_kinematics.utilities.transformations import rotz\

class SwingController:
    def __init__(self, config):
        self.config = config

    def raibert_touchdown_location(
        self, leg_index, command, shifted_forward, shifted_left
    ):
        delta_p_2d = (
            self.config.alpha
            * self.config.swing_ticks
            * self.config.dt
            * (command.horizontal_velocity * (float(self.config.phase_length)/self.config.swing_ticks))
        )
        delta_p = np.array([delta_p_2d[0], delta_p_2d[1], 0])

        theta = (
            self.config.beta
            * self.config.stance_ticks
            * self.config.dt
            * command.yaw_rate
        )
        #R = euler2mat(0, 0, theta)
        #return R @ self.config.default_stance[:, leg_index] + delta_p
        R = rotz(theta)
        step_dist_x = command.horizontal_velocity[0] * (float(self.config.phase_length)/self.config.swing_ticks)
        if shifted_forward == True: 
            shift_correction = np.array([-self.config.body_shift_x - step_dist_x/2.0 ,0,0])
        else:
            shift_correction = np.array([-step_dist_x + self.config.body_shift_x + step_dist_x/4.0 ,0,0])

        if shifted_left == True:
            shift_correction[1] = -self.config.body_shift_y
        else:
            shift_correction[1] = self.config.body_shift_y
        return np.matmul(R, self.config.default_stance[:, leg_index]) + delta_p + shift_correction 
        
    def swing_height(self, swing_phase, triangular=True):
        if triangular:
            if swing_phase < 0.5:
                swing_height_ = swing_phase / 0.5 * self.config.z_clearance
            else:
                swing_height_ = self.config.z_clearance * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_


    def next_foot_location(
        self,
        swing_prop,
        leg_index,
        state,
        command,
        shifted_forward,
        shifted_left
    ):
        assert swing_prop >= 0 and swing_prop <= 1
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, shifted_forward, shifted_left)

        time_left = self.config.dt * self.config.swing_ticks * (1.0 - swing_prop)
        
        v = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])

        delta_foot_location = v * self.config.dt
        z_vector = np.array([0, 0, swing_height_ + command.height])
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
