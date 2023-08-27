# gazebo-robotraconteur-sim-drivers

This package contains simulated drivers for use with the Gazebo Robot Raconteur Server Plugin. The drivers
connect to the server, and provide standard Robot Raconteur service types. The following drivers are available:

* `robot`: Simulated robot by kinematically commanding a robot model
* `gripper`: Simulated gripper using a link attacher (closed will attach to the object, open will release it)
* `camera`: Simulated camera using a Gazebo camera sensor

These drivers are primarily intended for use with the Robot Raconteur Training simulator, but may also be
useful for other purposes.

See als:

* [Robot Raconteur](https://robotraconteur.com)
* [Robot Raconteur Gazebo Server Plugin](https://github.com/robotraconteur-contrib/RobotRaconteur_Gazebo_Server_Plugin)
* [Robot Raconteur Training Simulator](https://github.com/robotraconteur-contrib/robotraconteur_training_sim)
* [Robot Raconteur Standard Types](https://github.com/robotraconteur/robotraconteur_standard_robdef)
