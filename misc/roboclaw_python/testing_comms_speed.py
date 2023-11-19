from roboclaw_3 import Roboclaw
import time

roboclaw = Roboclaw("/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00", 38400)
roboclaw.Open()

rightAddress = 0x80
leftAddress = 0x81

DIRECTION = 1
MAX_SPEED_LIMIT = 500 * DIRECTION
PPR = int(4096/2)
BUFFER = 0
time.sleep(1)

roboclaw.SpeedM1M2(rightAddress, MAX_SPEED_LIMIT, MAX_SPEED_LIMIT)
# roboclaw.SpeedDistanceM1M2(leftAddress, MAX_SPEED_LIMIT, PPR, MAX_SPEED_LIMIT, PPR, BUFFER)
# roboclaw.SpeedDistanceM1M2(leftAddress, 0, 0, 0, 0, 0)
# roboclaw.SpeedDistanceM1M2(rightAddress, 0, 0, 0, 0, 0)
# roboclaw.SpeedDistanceM1M2(leftAddress, -MAX_SPEED_LIMIT, PPR, -MAX_SPEED_LIMIT, PPR, BUFFER)
# roboclaw.SpeedDistanceM1M2(rightAddress, -MAX_SPEED_LIMIT, PPR, -MAX_SPEED_LIMIT, PPR, BUFFER)
# roboclaw.SpeedDistanceM1M2(leftAddress, 0, 0, 0, 0, 0)
# roboclaw.SpeedDistanceM1M2(rightAddress, 0, 0, 0, 0, 0)
while True:
    time.sleep(0.1)
    lbf, lbm1, lbm2 = roboclaw.ReadBuffers(leftAddress)
    rbf, rbm1, rbm2 = roboclaw.ReadBuffers(rightAddress)
    print("Buffers: ", lbm1, lbm2, rbm1, rbm2)
    if lbm1 == 128 and lbm2 == 128 and rbm1 == 128 and rbm2 == 128:
        break
print("ALL DONE!")

while True:
    print(time.localtime(), "Main Battery Left Voltage: ", roboclaw.ReadMainBatteryVoltage(leftAddress))
    print(time.localtime(),"Main Battery Right Voltage: ", roboclaw.ReadMainBatteryVoltage(rightAddress))
    time.sleep(1)