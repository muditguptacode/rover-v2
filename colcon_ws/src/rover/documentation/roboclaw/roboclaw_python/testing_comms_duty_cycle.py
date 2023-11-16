from roboclaw_3 import Roboclaw
import time

# leftRoboclaw = Roboclaw("/dev/ttyACM1", 38400)
# leftRoboclaw.Open()

roboclaw = Roboclaw("COM9", 38400)
roboclaw.Open()

rightAddress = 0x80
leftAddress = 0x81

MAX_SPEED_LIMIT = 32767
time.sleep(1)
roboclaw.DutyM1M2(leftAddress, int(MAX_SPEED_LIMIT / 10), int(MAX_SPEED_LIMIT / 10))
roboclaw.DutyM1M2(rightAddress, int(MAX_SPEED_LIMIT / 10), int(MAX_SPEED_LIMIT / 10))
time.sleep(2)
roboclaw.DutyM1M2(leftAddress, 0, 0)
roboclaw.DutyM1M2(rightAddress, 0, 0)
time.sleep(1)
