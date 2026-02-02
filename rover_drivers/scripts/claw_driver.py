# Subscribes to /claw/command (gripper, multiple arm joints, extension)
# Sends complex multi-motor commands to Pico over serial
# Receives encoder/position feedback from claw motors
# Publishes claw state (joint angles, gripper pressure, fully extended, etc.)
# Handles claw-specific safety logic (force limits, position bounds)