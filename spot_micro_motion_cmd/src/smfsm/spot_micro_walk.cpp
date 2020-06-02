#include "spot_micro_walk.h"

#include <eigen3/Eigen/Geometry>

#include "spot_micro_transition_stand.h"
#include "spot_micro_motion_cmd.h"
#include "rate_limited_first_order_filter.h"

SpotMicroWalkState::SpotMicroWalkState() {
  contact_feet_states_.right_back_in_swing = false;
  contact_feet_states_.right_front_in_swing = false;
  contact_feet_states_.left_front_in_swing = false;
  contact_feet_states_.left_back_in_swing = false;

  ticks_ = 0;
  phase_index_ = 0;
  subphase_ticks_ = 0;
}

SpotMicroWalkState::~SpotMicroWalkState() {
  //std::cout << "SpotMicroWalkState Dtor" << std::endl;
}

void SpotMicroWalkState::handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc, 
                                   smk::BodyState* body_state_cmd) {
  // Debug output
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Walk State" << std::endl;
  }


  // If stand command received, change to transition to stand state
  if (cmd.getStandCmd() == true) {
    // Call parent class's change state method
    changeState(smmc, std::make_unique<SpotMicroTransitionStandState>());

  } else {
    // Update gate phasing data
    updatePhaseData();

    // Step the gait controller
    body_state_cmd->leg_feet_pos = stepGait(body_state, cmd, smnc, smmc->getNeutralStance()); 

    // Set body state command values
    //body_state_cmd->xyz_pos = cmd_state_.xyz_pos;

    //body_state_cmd->leg_feet_pos = cmd_state_.leg_feet_pos;

    // Set servo data and publish command
    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();

    // Increment ticks
    ticks_ += 1;
  }
}



void SpotMicroWalkState::init(const smk::BodyState& body_state,
                               const SpotMicroNodeConfig& smnc,
                               const Command& cmd,
                               SpotMicroMotionCmd* smmc) {

  // Save off config struct for convenience
  smnc_ = smnc;

  // Set default stance
  cmd_state_.leg_feet_pos = smmc->getNeutralStance();

  // End body state position and angles
  cmd_state_.euler_angs.phi = 0.0f;
  cmd_state_.euler_angs.theta = 0.0f;
  cmd_state_.euler_angs.psi = 0.0f;

  cmd_state_.xyz_pos.x = 0.0f;
  cmd_state_.xyz_pos.y = smnc.default_stand_height;
  cmd_state_.xyz_pos.z = 0.0f;

}


void SpotMicroWalkState::updatePhaseData() {
  int phase_time = ticks_ % smnc_.phase_length;
  int phase_sum = 0;

  // Update phase index and subphase ticks
  for (int i = 0; i < smnc_.num_phases; i++) {
    phase_sum += smnc_.phase_ticks[i];
    if (phase_time < phase_sum) {
      phase_index_ = i;
      subphase_ticks_ = phase_time - phase_sum + smnc_.phase_ticks[i];
      break;
    }
  }

  // Update contact feet states
  if (smnc_.rb_contact_phases[phase_index_] == 0) {
    contact_feet_states_.right_back_in_swing = true;
  } else {
    contact_feet_states_.right_back_in_swing = false;
  }

  if (smnc_.rf_contact_phases[phase_index_] == 0) {
    contact_feet_states_.right_front_in_swing = true;
  } else {
    contact_feet_states_.right_front_in_swing = false;
  }

  if (smnc_.lf_contact_phases[phase_index_] == 0) {
    contact_feet_states_.left_front_in_swing = true;
  } else {
    contact_feet_states_.left_front_in_swing = false;
  }

  if (smnc_.lb_contact_phases[phase_index_] == 0) {
    contact_feet_states_.left_back_in_swing = true;
  } else {
    contact_feet_states_.left_back_in_swing = false;
  }
}


smk::LegsFootPos SpotMicroWalkState::stepGait(
    const smk::BodyState& body_state,
    const Command& cmd,
    const SpotMicroNodeConfig& smnc, 
    const smk::LegsFootPos& default_stance_feet_pos) {

  bool contact_mode;
  float swing_proportion;
  smk::Point foot_pos;
  smk::LegsFootPos new_feet_pos;
  smk::Point default_stance_foot_pos;

  for (int i = 0; i < 4; i++) {
    if (i == 0) { // right back
      contact_mode = contact_feet_states_.right_back_in_swing;
      foot_pos = body_state.leg_feet_pos.right_back;
      default_stance_foot_pos = default_stance_feet_pos.right_back;

    } else if (i == 1) { // right front
      contact_mode = contact_feet_states_.right_front_in_swing;
      foot_pos = body_state.leg_feet_pos.right_front;
      default_stance_foot_pos = default_stance_feet_pos.right_front;

    } else if (i == 2) { // left front
      contact_mode = contact_feet_states_.left_front_in_swing;
      foot_pos = body_state.leg_feet_pos.left_front;
      default_stance_foot_pos = default_stance_feet_pos.left_front;

    } else { // left back
      contact_mode = contact_feet_states_.left_back_in_swing;
      foot_pos = body_state.leg_feet_pos.left_back;
      default_stance_foot_pos = default_stance_feet_pos.left_back;
    }

    if (contact_mode == false) { // Stance controller
      foot_pos = stanceController(foot_pos, cmd, smnc); 

    } else { // swing leg controller
      swing_proportion = (float)subphase_ticks_ / (float)smnc.swing_ticks;
      foot_pos = swingLegController(foot_pos, cmd, smnc, swing_proportion, default_stance_foot_pos);
    }

    if (i == 0) { // right back
      new_feet_pos.right_back = foot_pos;

    } else if (i == 1) { // right front
      new_feet_pos.right_front = foot_pos;

    } else if (i == 2) { // left front
      new_feet_pos.left_front = foot_pos;

    } else { // left back
      new_feet_pos.left_back = foot_pos;
    }
  }

  // Return new feet positions
  return new_feet_pos;
}


smk::Point SpotMicroWalkState::stanceController(
    const smk::Point& foot_pos,
    const Command& cmd,
    const SpotMicroNodeConfig& smnc) {

  using namespace Eigen;

  // Declare return value
  smk::Point new_foot_pos;

  // Convenience variables
  float dt = smnc.dt;
  float h_tau = smnc.foot_height_time_constant;

  // Calculate position deltas due to speed and rotation commands
  // Create vector to hold current foot position
  Vector3f foot_pos_vec(foot_pos.x, foot_pos.y, foot_pos.z);
  Vector3f new_foot_pos_vec;

  // Create delta position vector, which is the commanded velocity times the
  // timestep. Note that y speed command is really sideways velocity command, which 
  // is in the z direction of robot coordinate system. Stance foot position hard
  // coded to 0 height here in second equation
  Vector3f delta_pos(-cmd.getXSpeedCmd() * dt,
                     (1.0f/h_tau)*(0.0f - foot_pos.y) * dt,
                     -cmd.getYSpeedCmd() * dt);

  // Create rotation matrix for yaw rate
  Matrix3f rot_delta;
  rot_delta = AngleAxisf(cmd.getYawRateCmd() * dt, Vector3f::UnitY());

  // Move foot by rotation and linear translation deltas
  new_foot_pos_vec = (rot_delta * foot_pos_vec) + delta_pos;

  // Assign values to return structure
  new_foot_pos.x = new_foot_pos_vec[0];
  new_foot_pos.y = new_foot_pos_vec[1];
  new_foot_pos.z = new_foot_pos_vec[2];

  // Return value
  return new_foot_pos;
}

smk::Point SpotMicroWalkState::swingLegController(
    const smk::Point& foot_pos,
    const Command& cmd,
    const SpotMicroNodeConfig& smnc,
    float swing_proportion,
    const smk::Point& default_stance_foot_pos) {
  
  using namespace Eigen;

  // declare return value
  smk::Point new_foot_pos;

  // Convenience variables
  float dt = smnc.dt;
  //float h_tau = smnc.foot_height_time_constant;
  float swing_height;
  float alpha = smnc.alpha;
  float beta = smnc.beta;
  float stance_ticks = smnc.stance_ticks;
  Vector3f default_stance_foot_pos_vec(default_stance_foot_pos.x,
                                       default_stance_foot_pos.y,
                                       default_stance_foot_pos.z);

  // Calculate swing height based on triangular profile
  if (swing_proportion < 0.5f) {
    swing_height = (swing_proportion/ 0.5f) * smnc.z_clearance;
  } else {
    swing_height = smnc.z_clearance * (1.0f - (swing_proportion - 0.5f) / 0.5f);
  }


  // Calculate position deltas due to speed and rotation commands
  // Create vector to hold current foot position
  Vector3f foot_pos_vec(foot_pos.x, foot_pos.y, foot_pos.z);
  Vector3f new_foot_pos_vec;

  // Create delta position vector for touchdown location
  Vector3f delta_pos(alpha * stance_ticks * dt * cmd.getXSpeedCmd(),
                     0.0f, 
                     alpha * stance_ticks * dt * cmd.getYSpeedCmd());

  // Create rotation matrix for yaw rate
  float theta = beta * stance_ticks * dt * -cmd.getYawRateCmd();
  Matrix3f rot_delta;
  rot_delta = AngleAxisf(theta, Vector3f::UnitY());

  // Calculate touchdown location
  Vector3f touchdown_location = (rot_delta * default_stance_foot_pos_vec) + delta_pos;

  float time_left = dt * smnc.swing_ticks * (1.0f - swing_proportion);

  Vector3f delta_pos2 = ((touchdown_location - foot_pos_vec) / time_left) * dt;

  new_foot_pos_vec = foot_pos_vec + delta_pos2;
  new_foot_pos_vec[1] = swing_height;
  
  // Assign values to return structure
  new_foot_pos.x = new_foot_pos_vec[0];
  new_foot_pos.y = new_foot_pos_vec[1];
  new_foot_pos.z = new_foot_pos_vec[2];

  // Return value
  return new_foot_pos;
  
}

