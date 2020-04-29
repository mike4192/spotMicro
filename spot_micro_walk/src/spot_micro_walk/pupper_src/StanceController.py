import numpy as np
#from transforms3d.euler import euler2mat
from ..spot_micro_kinematics.utilities.transformations import rotz

class StanceController:
    def __init__(self, config):
        self.config = config


    def position_delta(self, leg_index, state, command, move_fwd, first_cycle, move_sideways, move_left):
        """Calculate the difference between the next desired body location and the current body location
        
        Parameters
        ----------
        z_measured : float
            Z coordinate of the feet relative to the body.
        stance_params : StanceParams
            Stance parameters object.
        movement_reference : MovementReference
            Movement reference object.
        gait_params : GaitParams
            Gait parameters object.

        Returns
        -------
        (Numpy array (3), Numpy array (3, 3))
            (Position increment, rotation matrix increment)
        """
        z = state.foot_locations[2, leg_index]
        step_dist_x = command.horizontal_velocity[0] * (float(self.config.phase_length)/self.config.swing_ticks)

        if first_cycle == True:
            shift_factor = 1
        else:
            shift_factor = 2

        side_vel = 0
        if move_sideways == True:
            if move_left == True:
                side_vel = -(self.config.body_shift_y*shift_factor)/(float(self.config.dt)*self.config.stance_ticks)
            else:
                side_vel = (self.config.body_shift_y*shift_factor)/(float(self.config.dt)*self.config.stance_ticks)
        else:
            side_enable = 0.0
            
        if move_fwd == True:

            v_xy = np.array(
                [
                    -(self.config.body_shift_x*shift_factor + step_dist_x/4.0)/(float(self.config.dt)*self.config.stance_ticks), 
                    side_vel, 
                    1.0
                    / self.config.z_time_constant
                    * (state.height - z),
                ]
            )
        else:
            v_xy = np.array(
                [
                    (self.config.body_shift_x*shift_factor - step_dist_x/4.0)/(float(self.config.dt)*self.config.stance_ticks), 
                    side_vel,
                    1.0
                    / self.config.z_time_constant
                    * (state.height - z),
                ]
            )
        delta_p = v_xy * self.config.dt
        #delta_R = euler2mat(0, 0, -command.yaw_rate * self.config.dt)
        delta_R = rotz(-command.yaw_rate * self.config.dt)
        return (delta_p, delta_R)


    # TODO: put current foot location into state
    def next_foot_location(self, leg_index, state, command, move_fwd, first_cycle, move_sideways, move_left):
        foot_location = state.foot_locations[:, leg_index]
        (delta_p, delta_R) = self.position_delta(leg_index, state, command, move_fwd, first_cycle, move_sideways, move_left)
        #incremented_location = delta_R @ foot_location + delta_p
        incremented_location = np.matmul(delta_R, foot_location) + delta_p
        
        return incremented_location
