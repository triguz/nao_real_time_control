Header header

# A list of joint names, corresponding to their names in the Nao docs.
# This must be either the same lenght of joint_angles or 1 if it's a
# keyworord such as 'body' (for all angles)
string[] joint_names
float32[] joint_angles

# Fraction of max joint velocity [0:1]
float32 speed
