# Final Project for ROB456
### Project Partners
[TC Drury](https://github.com/druryt)

[Kira Kopcho](https://github.com/kira-the-engineer)

[Leo Klotz](https://github.com/Leopold-Klotz)

[Stephen Fike](https://github.com/traveno)

### Environment
* ROS Noetic

### Compiling and sourcing
* `source /opt/ros/noetic/setup.bash` (if not already sourced)
* `catkin_make` (from root level repo directory)
* `source ./devel/setup.bash` (from root level repo directory)

### To get things running (in the context of lab 2)
Ensure `devel/setup.bash` is sourced for any terminals running these commands.
1. `roscore` -- start ROS master node
2. `roslaunch stage_osu empty.launch` -- launch an environment
3. `rosrun lab2 send_points.py` -- code related to lab 2
4. `rosrun lab2 driver.py` -- code related to lab 2
5. `rviz -d src/lab2/config/driver.rviz` -- rviz with lab 2 config