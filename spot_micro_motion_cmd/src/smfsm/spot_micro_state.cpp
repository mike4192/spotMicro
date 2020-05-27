#include "spot_micro_state.h"

#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "spot_micro_motion_cmd.h"

using namespace smk;

// Constructor
SpotMicroState::SpotMicroState() {
  // Nothing needs to be done
  //std::cout << "SpotMicroState Ctor" << std::endl;
}


SpotMicroState::~SpotMicroState() {
  //std::cout << "SpotMicroState Dtor" << std::endl;
}


void SpotMicroState::changeState(SpotMicroMotionCmd* smmc, std::unique_ptr<SpotMicroState> sms) {
  smmc->changeState(std::move(sms));
}


void SpotMicroState::initBodyStateFilters(
    float dt, float tau, float rl, float rl_ang,
     const smk::BodyState& body_state,
     BodyStateFilters* body_state_filters) {

  // typedef for convenience
  typedef RateLmtdFirstOrderFilter rlfof;

  // Create and initialize filter objects
  body_state_filters->leg_right_front = 
      {.x = rlfof(dt, tau, body_state.leg_feet_pos.right_front.x, rl),
       .y = rlfof(dt, tau, body_state.leg_feet_pos.right_front.y, rl),
       .z = rlfof(dt, tau, body_state.leg_feet_pos.right_front.z, rl)};

  body_state_filters->leg_right_back = 
      {.x = rlfof(dt, tau, body_state.leg_feet_pos.right_back.x, rl),
       .y = rlfof(dt, tau, body_state.leg_feet_pos.right_back.y, rl),
       .z = rlfof(dt, tau, body_state.leg_feet_pos.right_back.z, rl)};

  body_state_filters->leg_left_back = 
      {.x = rlfof(dt, tau, body_state.leg_feet_pos.left_back.x, rl),
       .y = rlfof(dt, tau, body_state.leg_feet_pos.left_back.y, rl),
       .z = rlfof(dt, tau, body_state.leg_feet_pos.left_back.z, rl)};

  body_state_filters->leg_left_front = 
      {.x = rlfof(dt, tau, body_state.leg_feet_pos.left_front.x, rl),
       .y = rlfof(dt, tau, body_state.leg_feet_pos.left_front.y, rl),
       .z = rlfof(dt, tau, body_state.leg_feet_pos.left_front.z, rl)};

  body_state_filters->body_pos = 
      {.x = rlfof(dt, tau, body_state.xyz_pos.x, rl),
       .y = rlfof(dt, tau, body_state.xyz_pos.y, rl),
       .z = rlfof(dt, tau, body_state.xyz_pos.z, rl)};

  body_state_filters->body_angs = 
      {.x = rlfof(dt, tau, body_state.euler_angs.phi, rl_ang),
       .y = rlfof(dt, tau, body_state.euler_angs.theta, rl_ang),
       .z = rlfof(dt, tau, body_state.euler_angs.psi, rl_ang)};
}


void SpotMicroState::setBodyStateFilterCommands(
    const smk::BodyState& body_state,
    BodyStateFilters* body_state_filters) {
  
  // Set commands for all filters
  // Right front leg
  body_state_filters->leg_right_front.x.setCommand(
      body_state.leg_feet_pos.right_front.x);

  body_state_filters->leg_right_front.y.setCommand(
      body_state.leg_feet_pos.right_front.y);

  body_state_filters->leg_right_front.z.setCommand(
      body_state.leg_feet_pos.right_front.z);

  // Right Back leg
  body_state_filters->leg_right_back.x.setCommand(
      body_state.leg_feet_pos.right_back.x);

  body_state_filters->leg_right_back.y.setCommand(
      body_state.leg_feet_pos.right_back.y);

  body_state_filters->leg_right_back.z.setCommand(
      body_state.leg_feet_pos.right_back.z);

  // Left Back leg
  body_state_filters->leg_left_back.x.setCommand(
      body_state.leg_feet_pos.left_back.x);

  body_state_filters->leg_left_back.y.setCommand(
      body_state.leg_feet_pos.left_back.y);

  body_state_filters->leg_left_back.z.setCommand(
      body_state.leg_feet_pos.left_back.z);

  // Left front leg
  body_state_filters->leg_left_front.x.setCommand(
      body_state.leg_feet_pos.left_front.x);

  body_state_filters->leg_left_front.y.setCommand(
      body_state.leg_feet_pos.left_front.y);

  body_state_filters->leg_left_front.z.setCommand(
      body_state.leg_feet_pos.left_front.z);

  // Body pos
  body_state_filters->body_pos.x.setCommand(body_state.xyz_pos.x);
  body_state_filters->body_pos.y.setCommand(body_state.xyz_pos.y);
  body_state_filters->body_pos.z.setCommand(body_state.xyz_pos.z);

  body_state_filters->body_angs.x.setCommand(body_state.euler_angs.phi);
  body_state_filters->body_angs.y.setCommand(body_state.euler_angs.theta);
  body_state_filters->body_angs.z.setCommand(body_state.euler_angs.psi);
}


void SpotMicroState::runFilters(BodyStateFilters* body_state_filters) {

  // xyz body position
  body_state_filters->body_pos.x.runTimestep();
  body_state_filters->body_pos.y.runTimestep();
  body_state_filters->body_pos.z.runTimestep();
 
  // Phi, theta psi body angles 
  body_state_filters->body_angs.x.runTimestep();
  body_state_filters->body_angs.y.runTimestep();
  body_state_filters->body_angs.z.runTimestep();

  // right back leg
  body_state_filters->leg_right_back.x.runTimestep();
  body_state_filters->leg_right_back.y.runTimestep();
  body_state_filters->leg_right_back.z.runTimestep();

  // right front leg
  body_state_filters->leg_right_front.x.runTimestep();
  body_state_filters->leg_right_front.y.runTimestep();
  body_state_filters->leg_right_front.z.runTimestep();

  // right front leg
  body_state_filters->leg_left_front.x.runTimestep();
  body_state_filters->leg_left_front.y.runTimestep();
  body_state_filters->leg_left_front.z.runTimestep();

  // right front leg
  body_state_filters->leg_left_back.x.runTimestep();
  body_state_filters->leg_left_back.y.runTimestep();
  body_state_filters->leg_left_back.z.runTimestep();
}

void SpotMicroState::assignFilterValuesToBodyState(
    const BodyStateFilters& body_state_filters,
    smk::BodyState* body_state) {

    body_state->xyz_pos.x = body_state_filters.body_pos.x.getOutput();
    body_state->xyz_pos.y = body_state_filters.body_pos.y.getOutput();
    body_state->xyz_pos.z = body_state_filters.body_pos.z.getOutput();

    body_state->euler_angs.phi =   body_state_filters.body_angs.x.getOutput();
    body_state->euler_angs.theta = body_state_filters.body_angs.y.getOutput();
    body_state->euler_angs.psi =   body_state_filters.body_angs.z.getOutput();

    // right back leg
    body_state->leg_feet_pos.right_back.x = 
        body_state_filters.leg_right_back.x.getOutput();
    body_state->leg_feet_pos.right_back.y = 
        body_state_filters.leg_right_back.y.getOutput();
    body_state->leg_feet_pos.right_back.z = 
        body_state_filters.leg_right_back.z.getOutput();

    // right front leg
    body_state->leg_feet_pos.right_front.x = 
        body_state_filters.leg_right_front.x.getOutput();
    body_state->leg_feet_pos.right_front.y = 
        body_state_filters.leg_right_front.y.getOutput();
    body_state->leg_feet_pos.right_front.z = 
        body_state_filters.leg_right_front.z.getOutput();

    // right front leg
    body_state->leg_feet_pos.left_front.x = 
        body_state_filters.leg_left_front.x.getOutput();
    body_state->leg_feet_pos.left_front.y = 
        body_state_filters.leg_left_front.y.getOutput();
    body_state->leg_feet_pos.left_front.z = 
        body_state_filters.leg_left_front.z.getOutput();

    // right front leg
    body_state->leg_feet_pos.left_back.x = 
        body_state_filters.leg_left_back.x.getOutput();
    body_state->leg_feet_pos.left_back.y = 
        body_state_filters.leg_left_back.y.getOutput();
    body_state->leg_feet_pos.left_back.z = 
        body_state_filters.leg_left_back.z.getOutput();
}

bool SpotMicroState::checkBodyStateEquality(
    const smk::BodyState& body_state1,
    const smk::BodyState& body_state2,
    float tol) {
  // Initialize return value true, run checks and set false if any fail
  bool ret_val = true;

  // right back leg
  if (std::fabs(body_state1.leg_feet_pos.right_back.x - 
                body_state2.leg_feet_pos.right_back.x) > tol) {
    ret_val = false; 
  } else if (std::fabs(body_state1.leg_feet_pos.right_back.y - 
                       body_state2.leg_feet_pos.right_back.y) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.leg_feet_pos.right_back.z - 
                       body_state2.leg_feet_pos.right_back.z) > tol) {
    ret_val = false;

  // right front leg
  } else if (std::fabs(body_state1.leg_feet_pos.right_front.x - 
                       body_state2.leg_feet_pos.right_front.x) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.leg_feet_pos.right_front.y - 
                       body_state2.leg_feet_pos.right_front.y) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.leg_feet_pos.right_front.z - 
                       body_state2.leg_feet_pos.right_front.z) > tol) {
    ret_val = false;

    // left front leg
  } else if (std::fabs(body_state1.leg_feet_pos.left_front.x - 
                       body_state2.leg_feet_pos.left_front.x) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.leg_feet_pos.left_front.y - 
                       body_state2.leg_feet_pos.left_front.y) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.leg_feet_pos.left_front.z - 
                       body_state2.leg_feet_pos.left_front.z) > tol) {
    ret_val = false;

    // left back leg
  } else if (std::fabs(body_state1.leg_feet_pos.left_back.x - 
                       body_state2.leg_feet_pos.left_back.x) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.leg_feet_pos.left_back.y - 
                       body_state2.leg_feet_pos.left_back.y) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.leg_feet_pos.left_back.z - 
                       body_state2.leg_feet_pos.left_back.z) > tol) {
    ret_val = false;

  // body xyz pos
  } else if (std::fabs(body_state1.xyz_pos.x -
                       body_state2.xyz_pos.x) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.xyz_pos.y -
                       body_state2.xyz_pos.y) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.xyz_pos.z -
                       body_state2.xyz_pos.z) > tol) {
    ret_val = false;

  // body angles
  } else if (std::fabs(body_state1.euler_angs.phi -
                       body_state2.euler_angs.phi) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.euler_angs.theta -
                       body_state2.euler_angs.theta) > tol) {
    ret_val = false;
  } else if (std::fabs(body_state1.euler_angs.psi -
                       body_state2.euler_angs.psi) > tol) {
    ret_val = false;
  }

  return ret_val;
}
