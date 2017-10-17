#ifndef HEBI_KINEMATIC_PARAMETERS_H
#define HEBI_KINEMATIC_PARAMETERS_H

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

#include <math.h>

/**
 * A structure which stores the parameters for a 1-DOF rotary actuator
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
struct HebiKinematicParametersActuator
{
  float com[3];
  float input_to_joint[16];
  float joint_rotation_axis[3];
  float joint_to_output[16];
};

/**
 * A structure which stores the parameters for a 1 output static body.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
struct HebiKinematicParametersStaticBody
{
  float com[3];
  float output[16];
};

/**
 * The kinematic parameters for an X5-series actuator.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static const struct HebiKinematicParametersActuator hebiKinematicParametersX5 = 
{
  { // com
    0, 0, 0.0155
  },
  { // input_to_joint
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.03105,
    0, 0, 0, 1
  },
  { // joint_rotation_axis
    0, 0, 1
  },
  { // joint_to_output
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  }
};

/**
 * The kinematic parameters for an X5-series shoulder bracket.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static const struct HebiKinematicParametersStaticBody hebiKinematicParametersX5Bracket =
{
  { // com
    0, -0.02, 0.02
  },
  { // output (rotation is pi/2 about x axis)
    1, 0, 0, 0,
    0, 0, -1, -.04,
    0, 1, 0, 0.04,
    0, 0, 0, 1
  }
};

/**
 * The kinematic parameters for an X5-series tube link.
 *
 * \param length The length, from center of the previous actuator's output to
 * the center of the subsequent actuator's input, in meters.
 * \param twist The twist about the central axis of the tube.  A twist of zero
 * refers to an input and output frame that are aligned in rotation, but offset
 * in the z-direction.
 * 
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static struct HebiKinematicParametersStaticBody hebiKinematicParametersX5Link(float length, float twist)
{
  struct HebiKinematicParametersStaticBody tmp =
  {
    { // com
      length * 0.5f,
      -((float)sin(twist)) * 0.005f,
      ((float)cos(twist)) * 0.005f
    },
    { // output (rotation is twist about x axis)
      1, 0, 0, length,
      0, (float)cos(twist), -(float)sin(twist), -((float)sin(twist)) * 0.0175f,
      0, (float)sin(twist), (float)cos(twist), 0.0175f + ((float)cos(twist)) * 0.0175f,
      0, 0, 0, 1
    }
  };
  return tmp;
}

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_KINEMATIC_PARAMETERS_H
