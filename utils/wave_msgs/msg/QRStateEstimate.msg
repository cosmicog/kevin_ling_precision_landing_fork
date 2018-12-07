# QRStateEstimate.msg
Header header

# Type of the state estimate message. The type determines which fields are
# valid. Valid types are listed at the bottom of this message defintiion
# and are prefixed with EST_.
int8 type

# True if there was a position measurement. If this is false for a long time,
# then the x, y, dx, and dy fields are dead reckoned and likely invalid.
bool position_measurement_update

#
# State estimate variables.
#

# Position
float32 x # m
float32 y # m
float32 z # m

# Attitude, in 3-2-1 Euler angles
float32 roll  # rad
float32 pitch # rad
float32 yaw   # rad

# Linear Velocity
float32 dx # m/s
float32 dy # m/s
float32 dz # m/s

# Angular Velocity in Euler Angles
float32 droll   # rad/s
float32 dpitch  # rad/s
float32 dyaw    # rad/s

# Angular Velocity in Body Rates
float32 p       # rad/s
float32 q       # rad/s
float32 r       # rad/s

# Dynamic extension states for the full-state feedback linearized model.
float32 xi1 # unitless
float32 xi2 # unitless

#
# Valid message types for the QRStateEstimate message.
#

# EST_6_STATE - Six State Estimates:
#   Position (x, y, z) and linear velocity (dx, dy, dz) estimates are valid.
int8 EST_6_STATE = 1


# EST_9_STATE - Nine State Estimates:
#   Position (x, y, z), linear velocity (dx, dy, dz), and attitude (roll, pitch,
#   yaw) estimates are valid.
int8 EST_9_STATE = 2

# EST_12_STATE_EULER - Twelve State Estimates:
#   Position (x, y, z), linear velocity (dx, dy, dz), attitude (roll, pitch,
#   yaw), and angular velocity Euler angle (droll, dpitch, dyaw) estimates are
#   valid.
int8 EST_12_STATE_EULER = 3

# EST_12_STATE_PQR - Twelve State Estimates:
#   Position (x, y, z), linear velocity (dx, dy, dz), attitude (roll, pitch,
#   yaw), and angular velocity body rate (p, q, r) estimates are valid.
int8 EST_12_STATE_PQR = 4

# EST_12_STATE - Twelve State Estimates:
#   Position (x, y, z), linear velocity (dx, dy, dz), attitude (roll, pitch,
#   yaw), angular velocity Euler angle (droll, dpitch, dyaw), and angular
#   velocity body rate (p, q, r) estimates are valid.
int8 EST_12_STATE = 5

# EST_14_STATE - Fourteen State Estimates:
#   Position (x, y, z), linear velocity (dx, dy, dz), attitude (roll, pitch,
#   yaw), angular velocity (droll, dpitch, dyaw), and dynamic extension state
#   (xi1, xi2) estimates are valid.
int8 EST_14_STATE = 6
