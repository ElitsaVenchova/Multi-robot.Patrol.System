# patrol_sim
 Patrol Simulation using Gazebo, Turtlebot3, ROS2 and Python
 
 Notes:
 
	 ISO:
		24.04 amd: https://releases.ubuntu.com/noble/
		22.04 amd: https://releases.ubuntu.com/22.04/

	Fix terminal not open: (22.04 amd)
		https://www.youtube.com/watch?v=fokpWH94OxY //change language and restart

	Fix missing sudo rights: (22.04 amd)
		https://www.youtube.com/watch?v=ZxOwFOtcaaA

	ROS2:
		Jazzy (24.04 amd): https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
		Humble (22.04 amd): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
		
	https://automaticaddison.com/how-to-install-ros-2-jazzy/
		echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
		new terminal or source ~/.bashrc
		
	TurtleBot3 simulation: //select Humble / Jazzy
		https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
		https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
		
	Create packege:
		cd ~/turtlebot3_ws/src
		ros2 pkg create patrol_sim --build-type ament_python --dependencies rclpy geometry_msgs nav_msgs gazebo_ros
		cd patrol_sim
		mkdir scripts launch
		touch scripts/__init__.py
		chmod +x scripts/*.py
		cd ~/turtlebot3_ws
		colcon build --symlink-install
		source install/setup.bash
		
		
	rqt_graph
	ros2 launch turtlebot3_gazebo multi_robot.launch.py

	rebuild:
		cd ~/turtlebot3_ws
		rm -rf build install log
		colcon build --symlink-install
		source install/setup.bash
		
	ros2 launch patrol_sim multi_robot.launch.py
	ros2 launch patrol_sim multi_behavior.launch.py
