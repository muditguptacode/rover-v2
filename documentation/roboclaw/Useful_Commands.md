#Launch Gazebo Sim with saved workd
ros2 launch rover launch_sim.launch.py world:=src/rover/worlds/obstacles.world 

#Launch teleop Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

#Launch roboclaw hardware interface
ros2 launch diffdrive_roboclaw diffbot.launch.py use_mock_hardware:=false gui:=false