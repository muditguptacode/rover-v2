from roboclaw_3 import Roboclaw
import time

# leftRoboclaw = Roboclaw("/dev/ttyACM1", 38400)
# leftRoboclaw.Open()

roboclaw = Roboclaw("COM9", 38400)
roboclaw.Open()

rightAddress = 0x80
leftAddress = 0x81

MAX_SPEED_LIMIT = 5000
PPR = 4096
time.sleep(1)
# roboclaw.SpeedM1M2(leftAddress, int(MAX_SPEED_LIMIT / 10), int(MAX_SPEED_LIMIT / 10))
# roboclaw.SpeedM1M2(rightAddress, int(MAX_SPEED_LIMIT / 10), int(MAX_SPEED_LIMIT / 10))
# time.sleep(2)
# roboclaw.SpeedM1M2(leftAddress, 0, 0)
# roboclaw.SpeedM1M2(rightAddress, 0, 0)
# time.sleep(1)
for i in range(0, 1):
    roboclaw.SpeedDistanceM1M2(rightAddress, MAX_SPEED_LIMIT, PPR, MAX_SPEED_LIMIT, PPR, 0)
    roboclaw.SpeedDistanceM1M2(rightAddress, 0, 0, 0, 0, 0)
    roboclaw.SpeedDistanceM1M2(rightAddress, -MAX_SPEED_LIMIT, PPR, -MAX_SPEED_LIMIT, PPR, 0)
    roboclaw.SpeedDistanceM1M2(rightAddress, 0, 0, 0, 0, 0)