from .Gaits import GaitController
from .StanceController import StanceController
from .SwingLegController import SwingController
from .Utilities import clipped_first_order_filter
from .State import BehaviorState, State
from ..spot_micro_kinematics.utilities.transformations import rotz

import numpy as np
#from transforms3d.euler import euler2mat, quat2euler
#from transforms3d.quaternions import qconjugate, quat2axangle
#from transforms3d.axangles import axangle2mat


class Controller:
    """Controller and planner object
    """

    def __init__(self, config):
        self.config = config
        self.first_cycle = True
        self.smoothed_yaw = 0.0  # for REST mode only
        #self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        #self.hop_transition_mapping = {BehaviorState.REST: BehaviorState.HOP, BehaviorState.HOP: BehaviorState.FINISHHOP, BehaviorState.FINISHHOP: BehaviorState.REST, BehaviorState.TROT: BehaviorState.HOP}
        self.trot_transition_mapping = {BehaviorState.REST: BehaviorState.TROT, BehaviorState.TROT: BehaviorState.REST, BehaviorState.HOP: BehaviorState.TROT, BehaviorState.FINISHHOP: BehaviorState.TROT}
        self.activate_transition_mapping = {BehaviorState.DEACTIVATED: BehaviorState.REST, BehaviorState.REST: BehaviorState.DEACTIVATED}


    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        prev_contact_modes = self.config.contact_phases[:,self.gait_controller.phase_index(state.ticks)-1]
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            phase_idx = self.gait_controller.phase_index(state.ticks)
            contact_mode = contact_modes[leg_index]
            prev_contact_mode = prev_contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if (contact_mode == 1) or (contact_mode == -1):
                if contact_mode == 1:
                    move_fwd = True
                else:
                    move_fwd = False

                if phase_idx in (0,4):
                    move_sideways = True
                    if phase_idx == 0:
                        move_left = True
                    else:
                        move_left = False
                else:
                    move_sideways = False
                    move_left = False



                new_location = self.stance_controller.next_foot_location(leg_index, state, command, move_fwd, self.first_cycle, move_sideways, move_left)
            elif contact_mode == 2:
                new_location = foot_location
            else:
                swing_proportion = (
                   float(self.gait_controller.subphase_ticks(state.ticks)) / float(self.config.swing_ticks)
                )
                if prev_contact_mode == 1:
                    shifted_forward = True
                else:
                    shifted_forward = False

                if phase_idx in (1,3):
                    # Body is shifted left
                    shifted_left = True
                else:
                    shifted_left = False

                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command,
                    shifted_forward,
                    shifted_left
                )

            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes


    def run(self, state, command):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """

        ########## Update operating state based on command ######
        if command.activate_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
            self.first_cycle = True
        # elif command.hop_event:
        #     state.behavior_state = self.hop_transition_mapping[state.behavior_state]

        if state.behavior_state == BehaviorState.TROT:
            state.foot_locations, contact_modes = self.step_gait(
                state,
                command,
            )
            if (self.gait_controller.phase_index(state.ticks) > 0) and (self.first_cycle == True):
                self.first_cycle = False

            # Apply the desired body rotation
            #rotated_foot_locations = (
            #    euler2mat(
            #        0, 0, 0.0
            #    )
            #    @ state.foot_locations
            #)

            # Construct foot rotation matrix to compensate for body tilt
            # (roll, pitch, yaw) = quat2euler(state.quat_orientation)
            # correction_factor = 0.8
            # max_tilt = 0.4
            # roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            # pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
            #rmat = euler2mat(0, 0, 0)

            #rotated_foot_locations = rmat.T @ rotated_foot_locations

            #state.joint_angles = self.inverse_kinematics(
            #    rotated_foot_locations, self.config
            #)

        # elif state.behavior_state == BehaviorState.HOP:
        #     state.foot_locations = (
        #         self.config.default_stance
        #         + np.array([0, 0, -0.09])[:, np.newaxis]
        #     )
        #     state.joint_angles = self.inverse_kinematics(
        #         state.foot_locations, self.config
        #     )

        # elif state.behavior_state == BehaviorState.FINISHHOP:
        #     state.foot_locations = (
        #         self.config.default_stance
        #         + np.array([0, 0, -0.22])[:, np.newaxis]
        #     )
        #     state.joint_angles = self.inverse_kinematics(
        #         state.foot_locations, self.config
        #     )

        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = float(command.yaw_rate) / float(self.config.max_yaw_rate)
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )
            # Set the foot locations to the default stance plus the standard height
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            # Apply the desired body rotation
            #rotated_foot_locations = (
            #    euler2mat(
            #        command.roll,
            #        command.pitch,
            #        self.smoothed_yaw,
            #    )
            #    @ state.foot_locations
            #)
            # rotated_foot_locations = np.matmul(rotz(self.smoothed_yaw),state.foot_locations)
            # state.joint_angles = self.inverse_kinematics(
                # rotated_foot_locations, self.config
            # )

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

        return state.foot_locations

    # def set_pose_to_default(self):
    #     state.foot_locations = (
    #         self.config.default_stance
    #         + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
    #     )
    #     state.joint_angles = controller.inverse_kinematics(
    #         state.foot_locations, self.config
    #     )
