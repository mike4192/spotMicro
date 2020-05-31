#include "spot_micro_motion_cmd.h"





struct GaitConfig {
  SpotMicroNodeConfig smnc;
  float max_fwd_velocity;
  float max_side_velocity;
  float max_yaw_rate;
  float z_clearance;
  float alpha;
  float beta;
  
};
