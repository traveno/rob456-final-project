# Final Project for ROB456
### Compiling and sourcing
* `catkin_make` (from root level repo directory)
* `source devel/setup.bash`

### To get things running (in the context of lab 2)
Ensure `devel/setup.bash` is sourced for any terminals running these commands.
1. `roscore` -- start ROS master node
2. `roslaunch stage_osu empty.launch` -- launch an environment
3. `rosrun lab2 send_points.py` -- code related to lab 2
4. `rosrun lab2 driver.py` -- code related to lab 2
5. `rviz -d src/lab2/config/driver.rviz` -- rviz with lab 2 config