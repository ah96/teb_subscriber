# teb_subscriber ROS package

The teb_subscriber package implements a subscriber to the teb_local_planner of the 2D navigation stack. The underlying method of the teb_local_planner called Timed Elastic Band locally optimizes the robot's trajectory with respect to trajectory execution time, separation from obstacles and compliance with kinodynamic constraints at runtime.

Refer to http://wiki.ros.org/teb_local_planner for more information and tutorials about the local planner.

To run the teb_subscriber please type: rosrun teb_subscriber teb_listener.

Data will be saved as .csv files.

