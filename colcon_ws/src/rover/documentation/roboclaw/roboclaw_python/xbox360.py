import signal
from xbox360controller import Xbox360Controller
from roboclaw_3 import Roboclaw

# leftRoboclaw = Roboclaw("/dev/ttyACM1", 38400)
# leftRoboclaw.Open()

roboclaw = Roboclaw("/dev/ttyACM0", 115200)
roboclaw.Open()

rightAddress = 0x80
leftAddress = 0x81

MAX_SPEED_LIMIT = 120000*3
MAX_SPEED = MAX_SPEED_LIMIT/2
DEAD_BAND = 0.01


def on_A_button_pressed(button):
    print('Button {0} was pressed'.format(button.name))


def on_A_button_released(button):
    print('Button {0} was released'.format(button.name))


def on_right_trigger_pressed(button):
    global MAX_SPEED
    global MAX_SPEED_LIMIT
    print('Button {0} was pressed'.format(button.name))
    MAX_SPEED = MAX_SPEED_LIMIT


def on_right_trigger_released(button):
    global MAX_SPEED
    global MAX_SPEED_LIMIT
    print('Button {0} was released'.format(button.name))
    MAX_SPEED = MAX_SPEED_LIMIT/2


def on_B_button_pressed(button):
    print('Button {0} was pressed'.format(button.name))
    roboclaw.SpeedM1(leftAddress, 0)
    roboclaw.SpeedM2(leftAddress, 0)
    roboclaw.SpeedM1(rightAddress, 0)
    roboclaw.SpeedM2(rightAddress, 0)


def on_B_button_released(button):
    print('Button {0} was released'.format(button.name))


def on_axis_moved(axis):
    global MAX_SPEED
    global MAX_SPEED_LIMIT
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    if axis.name == "axis_l":
        if axis.y > DEAD_BAND or axis.y < -DEAD_BAND:
            print(int(MAX_SPEED*axis.y))

            roboclaw.SpeedM1(leftAddress, int(MAX_SPEED*axis.y))
            roboclaw.SpeedM2(leftAddress, int(MAX_SPEED*axis.y))
        else:
            print("STOPPING")

            roboclaw.SpeedM1(leftAddress, 0)
            roboclaw.SpeedM2(leftAddress, 0)

    if axis.name == "axis_r":
        if axis.y > DEAD_BAND or axis.y < -DEAD_BAND:
            print(int(MAX_SPEED*axis.y))

            roboclaw.SpeedM1(rightAddress, int(-MAX_SPEED*axis.y))
            roboclaw.SpeedM2(rightAddress, int(-MAX_SPEED*axis.y))
        else:
            print("STOPPING")
            roboclaw.SpeedM1(rightAddress, 0)
            roboclaw.SpeedM2(rightAddress, 0)


try:
    with Xbox360Controller(0, axis_threshold=0.005) as controller:
        # Button A events
        controller.button_a.when_pressed = on_A_button_pressed
        controller.button_a.when_released = on_A_button_released

        # Button B events
        controller.button_b.when_pressed = on_B_button_pressed
        controller.button_b.when_released = on_B_button_released

        # Button B events
        controller.button_trigger_r.when_pressed = on_right_trigger_pressed
        controller.button_trigger_r.when_released = on_right_trigger_released

        # Left and right axis move event
        controller.axis_l.when_moved = on_axis_moved
        controller.axis_r.when_moved = on_axis_moved

        signal.pause()
except KeyboardInterrupt:
    pass
