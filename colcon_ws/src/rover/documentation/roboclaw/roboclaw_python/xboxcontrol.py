import xbox
import time
import roboclaw

# Windows comport name
# roboclaw.Open("COM5",115200)
# Linux comport name
# roboclaw.Open("/dev/ttyACM0",115200)
roboclaw.Open("COM5", 115200)
roboclaw.Open("COM6", 115200)
address = 0x80
# Setup joystick
joy = xbox.Joystick()
scale = 32
try:
    # Valid connect may require joystick input to occur
    print("Waiting for Joystick to connect")
    while not joy.connected():
        time.sleep(0.10)

    # Show misc inputs until Back button is pressed
    while not joy.Back() and joy.connected():
        forwardBackVal = joy.leftY()
        leftRightVal = joy.rightX()
        if leftRightVal == 0:
            if forwardBackVal > 0:  
                val = int(forwardBackVal*scale)
                roboclaw.ForwardM1(address, val)
                roboclaw.ForwardM2(address, val)
            elif forwardBackVal < 0:
                val = int(-forwardBackVal*scale)
                roboclaw.BackwardM1(address, val)
                roboclaw.BackwardM2(address, val)
            elif forwardBackVal == 0:
                val = int(forwardBackVal*scale)
                roboclaw.ForwardM1(address, val)
                roboclaw.ForwardM2(address, val)
        elif leftRightVal > 0:
            val = int(leftRightVal*scale)
            roboclaw.BackwardM1(address, val)
            roboclaw.ForwardM2(address, val)
        elif leftRightVal < 0:
            val = int(-leftRightVal*scale)
            roboclaw.BackwardM2(address, val)
            roboclaw.ForwardM1(address, val)
        time.sleep(0.10)

finally:
    # Always close out so that xboxdrv subprocess ends
    joy.close()
    print("Done.")
