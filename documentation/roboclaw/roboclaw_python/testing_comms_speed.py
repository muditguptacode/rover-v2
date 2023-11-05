from roboclaw_3 import Roboclaw
import time

roboclaw = Roboclaw("/dev/ttyACM1", 38400)
roboclaw.Open()

rightAddress = 0x80
leftAddress = 0x81

MAX_SPEED_LIMIT = 5000
PPR = 4096
time.sleep(1)

for i in range(0, 1):
    roboclaw.SpeedDistanceM1M2(leftAddress, MAX_SPEED_LIMIT, PPR, MAX_SPEED_LIMIT, PPR, 0)
    roboclaw.SpeedDistanceM1M2(rightAddress, MAX_SPEED_LIMIT, PPR, MAX_SPEED_LIMIT, PPR, 0)
    roboclaw.SpeedDistanceM1M2(leftAddress, 0, 0, 0, 0, 0)
    roboclaw.SpeedDistanceM1M2(rightAddress, 0, 0, 0, 0, 0)
    roboclaw.SpeedDistanceM1M2(leftAddress, -MAX_SPEED_LIMIT, PPR, -MAX_SPEED_LIMIT, PPR, 0)
    roboclaw.SpeedDistanceM1M2(rightAddress, -MAX_SPEED_LIMIT, PPR, -MAX_SPEED_LIMIT, PPR, 0)
    roboclaw.SpeedDistanceM1M2(leftAddress, 0, 0, 0, 0, 0)
    roboclaw.SpeedDistanceM1M2(rightAddress, 0, 0, 0, 0, 0)