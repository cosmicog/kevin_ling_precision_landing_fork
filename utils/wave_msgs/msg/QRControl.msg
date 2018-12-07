# QRControl.msg
Header header

# Control command type. Valid types are defined at the bottom of this message
# definition and are prefixed with CMD_.
int8 type


#
# Controller reference signals. The interpretation of these fields is based on
# the message type. SI units are used for all control message types.
#
float32 x # m
float32 y # m
float32 z # m
# Yaw is the direction that the front of the quadrotor faces.
float32 yaw # rad

float32 dx # m/s
float32 dy # m/s
float32 dz # m/s
float32 dyaw # rad/s

# Heading is the direction that the quadrotor is moving in. This is independent
# of the yaw.
float32 heading # rad


#
# Valid types for the QRControl message.
# 

# CMD_INERTIAL_POSITION - Inertial Position Control:
#   - x, y, z, and yaw specify the desired position and yaw in the inertial
#     frame
#   - desired velocities (dx, dy, and dz), yaw rate (dyaw), and heading fields
#     are unused.
int8 CMD_INERTIAL_POSITION = 1

# CMD_INERTIAL_TRAJECTORY - Inertial Trajectory Control:
#   - x, y, z, and yaw specify the desired position and yaw
#   - dx, dy, dz, and dyaw specify the desired velocity and yaw rate commands
#
#   It is up to the implementer to publish QRControl messages at the appropriate
#   times for the trajectory. The heading field is unused.
int8 CMD_INERTIAL_TRAJECTORY = 2

# CMD_RELATIVE_POSITION - Relative Position Control:
#   The relative state estimates are used in this mode for x and y position
#   control. Commands are specified in the relative frame as well. Estimators
#   are free to define the relative frame however they want.
#
#   Height and yaw control are still performed in the inertial frame.
#
#   - x and y are the desired position in the relative frame
#   - dx and dy specify the desired velocities in the relative
#   - z and yaw specify commands in the inertial field
#   - dz, dyaw, and heading are unused
#
#   An example of where this would be used is to track a target observed with
#   relative measurements. The x and y fields would be set to the position of
#   the quadrotor relative to the target and the dx and dy fields would be
#   set to the x and y velocities of the quadrotor relative to the target.
int8 CMD_RELATIVE_POSITION = 3

# CMD_CONSTANT_VELOCITY - Constant Velocity Along Track Control:
#   This is a placeholder. This controller has not been implemented yet.
#int8 CMD_CONSTANT_VELOCITY = 

