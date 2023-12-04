# Final Project for ROB456
## Project Partners
[TC Drury](https://github.com/druryt)

[Kira Kopcho](https://github.com/kira-the-engineer)

[Leo Klotz](https://github.com/Leopold-Klotz)

[Stephen Fike](https://github.com/traveno)

## Environment
* ROS Noetic

## Compiling and sourcing
* `source /opt/ros/noetic/setup.bash` (if not already sourced)
* `catkin_make` (from root level repo directory)
* `source ./devel/setup.bash` (from root level repo directory)

## To get things running 
Ensure `devel/setup.bash` is sourced for any terminals running these commands.
* `roslaunch walle walle.launch`

## Docker based setup (via ROS BOX)
* `docker dev create traveno/rob456-final-project`
* You're done!
* Access the source code by attaching VSCode to the `dev_environment` container. Or use `docker dev open`.
* Access the window server at https://localhost:8080/. Or use a native VNC client such as TigerVNC (port 5900).

### Notes

* ROS BOX offers Catmux, use `cmux walle` to launch the package. The tmux session is mouse enabled. Use `C-a` `x` to
stop. Use `cmux-kill` to kill a background catmux session.
* `docker dev open` sometimes causes the containers to bump up port numbers, so if 8080 doesn't work, try 8081 (5901 for VNC clients).

![](https://raw.githubusercontent.com/traveno/ros2-docker-dev/noetic-tigervnc/.github/wallebot.png)