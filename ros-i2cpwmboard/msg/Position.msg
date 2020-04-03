# the position message is used when configuring drive mode to
# assign a  servo to a specific wheel positon in the drive system
# postions are specific toe the desired drive mode
# ackerman has only one position
# 1 = drive
# differential has two positons
# 1 = left, 2 = right
# mecanum has four positions
# 1 = front left, 2 = front right, 3 = rear left, 4 = rear right
# multiple servos/motors may be used for each positon

int16 servo
int16 position
