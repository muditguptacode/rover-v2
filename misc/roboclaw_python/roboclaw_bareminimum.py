from roboclaw import Roboclaw
import time
#Windows comport name
# rc = Roboclaw("COM3",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM1",115200)

rightAddress = 0x80
leftAddress = 0x81

rc.Open()
 
while True:
    time.sleep(0.1)
    enc1 = rc.ReadEncM1(address)
    enc2 = rc.ReadEncM2(address)
    print(enc1 +"     ", enc2)