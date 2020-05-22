#include "spot_micro_transition_stand.h"
#include "spot_micro_idle.h"
#include "spot_micro_stand.h"
#include "spot_micro_motion_cmd.h"

SpotMicroTransitionStandState::SpotMicroTransitionStandState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroTransitionStandState Ctor" << std::endl;

  // Need to create 6 first order filter object, ignore transiting feet poition
  // for now 
  //rlfof = RateLmtdFirstOrderFilter(0.02, 1.0f, 0.0f, 1000.0f);

}

SpotMicroTransitionStandState::~SpotMicroTransitionStandState() {
  std::cout << "SpotMicroTransitionStandState Dtor" << std::endl;
}


void SpotMicroTransitionStandState::init(SpotMicroMotionCmd* smmc, 
                    const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd) {
  // Set initial state and end state
  // Get starting body state
  start_body_state_ = body_state;
 
  // Create end state 
  // Create end state feet positions, a default foot stance
  float len = smnc.smc.body_length; // body length
  float width = smnc.smc.body_width; // body width
  float l1 = smnc.smc.hip_link_length; // liength of the hip link
  // TODO: add stance offset parameters

  end_body_state_.leg_feet_pos.right_back  = {.x = -len/2, .y = 0.0f, .z =  width/2 + l1};
  end_body_state_.leg_feet_pos.right_front = {.x =  len/2, .y = 0.0f, .z =  width/2 + l1};
  end_body_state_.leg_feet_pos.left_front  = {.x =  len/2, .y = 0.0f, .z = -width/2 - l1};
  end_body_state_.leg_feet_pos.left_back   = {.x = -len/2, .y = 0.0f, .z = -width/2 - l1};

  // End body state position and angles
  end_body_state_.euler_angs.phi = 0.0f;
  end_body_state_.euler_angs.theta = 0.0f;
  end_body_state_.euler_angs.psi = 0.0f;

  end_body_state_.xyz_pos.x = 0.0f;
  end_body_state_.xyz_pos.y = smnc.default_stand_height;
  end_body_state_.xyz_pos.z = 0.0f;

  // Initialize filters
  float dt = smnc.dt;
  float tau = smnc.transit_tau;
  float rl = smnc.transit_rl;
  float rl_ang = smnc.transit_angle_rl;
  typedef RateLmtdFirstOrderFilter rlfof;

  body_state_filters_.leg_right_front = 
      {.x = rlfof(dt, tau, start_body_state_.leg_feet_pos.right_front.x, rl),
       .y = rlfof(dt, tau, start_body_state_.leg_feet_pos.right_front.y, rl),
       .z = rlfof(dt, tau, start_body_state_.leg_feet_pos.right_front.z, rl)};

  body_state_filters_.leg_right_back = 
      {.x = rlfof(dt, tau, start_body_state_.leg_feet_pos.right_back.x, rl),
       .y = rlfof(dt, tau, start_body_state_.leg_feet_pos.right_back.y, rl),
       .z = rlfof(dt, tau, start_body_state_.leg_feet_pos.right_back.z, rl)};

  body_state_filters_.leg_left_back = 
      {.x = rlfof(dt, tau, start_body_state_.leg_feet_pos.left_back.x, rl),
       .y = rlfof(dt, tau, start_body_state_.leg_feet_pos.left_back.y, rl),
       .z = rlfof(dt, tau, start_body_state_.leg_feet_pos.left_back.z, rl)};

  body_state_filters_.leg_left_front = 
      {.x = rlfof(dt, tau, start_body_state_.leg_feet_pos.left_front.x, rl),
       .y = rlfof(dt, tau, start_body_state_.leg_feet_pos.left_front.y, rl),
       .z = rlfof(dt, tau, start_body_state_.leg_feet_pos.left_front.z, rl)};

  body_state_filters_.body_pos = 
      {.x = rlfof(dt, tau, start_body_state_.xyz_pos.x, rl),
       .y = rlfof(dt, tau, start_body_state_.xyz_pos.y, rl),
       .z = rlfof(dt, tau, start_body_state_.xyz_pos.z, rl)};

  body_state_filters_.body_angs = 
      {.x = rlfof(dt, tau, start_body_state_.euler_angs.phi, rl_ang),
       .y = rlfof(dt, tau, start_body_state_.euler_angs.theta, rl_ang),
       .z = rlfof(dt, tau, start_body_state_.euler_angs.psi, rl_ang)};

  // Set destination commands for all filters
  // Right front leg
  body_state_filters_.leg_right_front.x.setCommand(
      end_body_state_.leg_feet_pos.right_front.x);

  body_state_filters_.leg_right_front.y.setCommand(
      end_body_state_.leg_feet_pos.right_front.y);

  body_state_filters_.leg_right_front.z.setCommand(
      end_body_state_.leg_feet_pos.right_front.z);

  // Right Back leg
  body_state_filters_.leg_right_back.x.setCommand(
      end_body_state_.leg_feet_pos.right_back.x);

  body_state_filters_.leg_right_back.y.setCommand(
      end_body_state_.leg_feet_pos.right_back.y);

  body_state_filters_.leg_right_back.z.setCommand(
      end_body_state_.leg_feet_pos.right_back.z);

  // Left Back leg
  body_state_filters_.leg_left_back.x.setCommand(
      end_body_state_.leg_feet_pos.left_back.x);

  body_state_filters_.leg_left_back.y.setCommand(
      end_body_state_.leg_feet_pos.left_back.y);

  body_state_filters_.leg_left_back.z.setCommand(
      end_body_state_.leg_feet_pos.left_back.z);

  // Left front leg
  body_state_filters_.leg_left_front.x.setCommand(
      end_body_state_.leg_feet_pos.left_front.x);

  body_state_filters_.leg_left_front.y.setCommand(
      end_body_state_.leg_feet_pos.left_front.y);

  body_state_filters_.leg_left_front.z.setCommand(
      end_body_state_.leg_feet_pos.left_front.z);

  // Body pos
  body_state_filters_.body_pos.x.setCommand(end_body_state_.xyz_pos.x);
  body_state_filters_.body_pos.y.setCommand(end_body_state_.xyz_pos.y);
  body_state_filters_.body_pos.z.setCommand(end_body_state_.xyz_pos.z);

  body_state_filters_.body_angs.x.setCommand(end_body_state_.euler_angs.phi);
  body_state_filters_.body_angs.y.setCommand(end_body_state_.euler_angs.theta);
  body_state_filters_.body_angs.z.setCommand(end_body_state_.euler_angs.psi);
}


void SpotMicroTransitionStandState::handleInputCommands(
                                   const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc,
                                   smk:: BodyState* body_state_cmd) {
  std::cout << "In Spot Micro Idle State" << std::endl;
  
  // Check if desired end state reached, if so, change to stand state
  float cur_height = body_state.xyz_pos.y;
  float des_height = end_body_state_.xyz_pos.y;
  if (std::fabs(cur_height - des_height) < 0.001f) {
    changeState(smmc, std::make_unique<SpotMicroStandState>());
  
  } else {
    // Otherwise, otherwise, set transitory body/feet position/orientation, and
    // set and send servo proportional command
    body_state_cmd->xyz_pos.x = body_state_filters_.body_pos.x.runTimestepAndGetOutput();
    body_state_cmd->xyz_pos.y = body_state_filters_.body_pos.y.runTimestepAndGetOutput();
    body_state_cmd->xyz_pos.z = body_state_filters_.body_pos.z.runTimestepAndGetOutput();

    body_state_cmd->euler_angs.phi = 
        body_state_filters_.body_angs.x.runTimestepAndGetOutput();
    body_state_cmd->euler_angs.theta = 
        body_state_filters_.body_angs.y.runTimestepAndGetOutput();
    body_state_cmd->euler_angs.psi = 
        body_state_filters_.body_angs.z.runTimestepAndGetOutput();

    // right back leg
    body_state_cmd->leg_feet_pos.right_back.x = 
        body_state_filters_.leg_right_back.x.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.right_back.y = 
        body_state_filters_.leg_right_back.y.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.right_back.z = 
        body_state_filters_.leg_right_back.z.runTimestepAndGetOutput();

    // right front leg
    body_state_cmd->leg_feet_pos.right_front.x = 
        body_state_filters_.leg_right_front.x.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.right_front.y = 
        body_state_filters_.leg_right_front.y.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.right_front.z = 
        body_state_filters_.leg_right_front.z.runTimestepAndGetOutput();

    // right front leg
    body_state_cmd->leg_feet_pos.left_front.x = 
        body_state_filters_.leg_left_front.x.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.left_front.y = 
        body_state_filters_.leg_left_front.y.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.left_front.z = 
        body_state_filters_.leg_left_front.z.runTimestepAndGetOutput();

    // right front leg
    body_state_cmd->leg_feet_pos.left_back.x = 
        body_state_filters_.leg_left_back.x.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.left_back.y = 
        body_state_filters_.leg_left_back.y.runTimestepAndGetOutput();
    body_state_cmd->leg_feet_pos.left_back.z = 
        body_state_filters_.leg_left_back.z.runTimestepAndGetOutput();

    // Send command
    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();

  }

}


