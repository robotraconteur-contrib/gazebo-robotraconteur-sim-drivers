from contextlib import suppress
import sys
import time
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import numpy as np
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
import drekar_launch_process
import threading
import traceback

import argparse
import general_robotics_toolbox as rox

from robotraconteur_abstract_robot import AbstractRobot

class GazeboRobotImpl(AbstractRobot):
    def __init__(self, robot_info, gazebo_url, world_name, model_name, robot_op_mode):
        # Call AbstractRobot __init__
        super().__init__(robot_info, 6)

        self._gazebo_operational_mode = robot_op_mode
        self._gazebo_url = gazebo_url
        self._model_name = model_name
        self._world_name = world_name

        # This driver does not home the robot
        self._uses_homing = False
        # Streaming position command is available
        self._has_position_command = True
        
        self._has_velocity_command = False
        # Use 100 Hz update loop (10 ms timestep)
        self._update_period = 10e-3
       
        self._base_set_controller_state = True
        self._base_set_operational_mode = True
        # Override device capabilities from RobotInfo
        self.robot_info.robot_capabilities &= self._robot_capabilities["jog_command"] \
             & self._robot_capabilities["position_command"] & self._robot_capabilities["trajectory_command"] \
        # Set trajectory_error_tol to a large number.
        self._trajectory_error_tol = 1000

        self._egm = None

        self._gazebo_sub = None

        self._position_wire = None
        self._position_command_wire = None
        self._velocity_wire = None
        self._velocity_command_wire = None

    def _on_sub_connect(self, sub, subscription_id, client):
        self._world = client.get_worlds(self._world_name)
        self._model = self._world.get_models(self._model_name)

        self._model_rr_path = RRN.GetObjectServicePath(self._model)

        with suppress(Exception):
            self._model.destroy_joint_controller()

        with suppress(Exception):
            self._model.destroy_kinematic_joint_controller()

        self._model.create_kinematic_joint_controller()
        controller = self._model.get_kinematic_joint_controller()

        for joint_name in self._joint_names:
            controller.add_joint(joint_name)

        current_position, _ = controller.joint_position.PeekInValue()
        controller.joint_position_command.PokeOutValue(current_position)

        self._controller_rr_path = RRN.GetObjectServicePath(controller)

        if self._position_wire is None:
            self._position_wire = self._gazebo_sub.SubscribeWire("joint_position", self._controller_rr_path)
            self._position_wire.WireValueChanged += self._on_joint_position
            self._position_wire.InValueLifespan = 0.1
        if self._position_command_wire is None:
            self._position_command_wire = self._gazebo_sub.SubscribeWire("joint_position_command", self._controller_rr_path)
        if self._velocity_wire is None:
            self._velocity_wire = self._gazebo_sub.SubscribeWire("joint_velocity", self._controller_rr_path)
            self._velocity_wire.WireValueChanged += self._on_joint_velocity
        if self._velocity_command_wire is None:
            self._velocity_command_wire = self._gazebo_sub.SubscribeWire("joint_velocity_command", self._controller_rr_path)

        print("Connect complete")
        
    def _verify_communication(self, now):
        connected = False
        if self._position_wire is not None:
            with suppress(Exception):
                connected, _, _ = self._position_wire.TryGetInValue()

        if connected:
            self._last_robot_state = self._stopwatch_ellapsed_s()
            self._ready = True
            self._enabled = True
            self._homed = True
            self._error = False
            self._operational_mode = self._gazebo_operational_mode
        else:
            self._last_robot_state = 0


        return super()._verify_communication(now)

    def _on_joint_position(self, c, value, ts):
        if not value:
            return

        pos = np.zeros((len(self._joint_names),))
        for i in range(len(self._joint_names)):
            pos[i] = value[self._joint_names[i]]

        ep_pose = np.zeros((len(self._rox_robots),),dtype=self._pose_dtype)
        try:
            for i in range(len(self._rox_robots)):
                joint_numbers = self._robot_info.chains[i].joint_numbers
                pos1 = pos[joint_numbers]
                rox_ep_pos = rox.fwdkin(self._rox_robots[i], pos1, True)
                ep_pose[i] = self._geometry_util.rox_transform_to_pose(rox_ep_pos)
        except:
            ep_pose = None
            traceback.print_exc()

        with self._lock:
            self._joint_position = pos
            self._endpoint_pose = ep_pose
            self._last_joint_state = self._stopwatch_ellapsed_s()
            self._last_endpoint_state = self._stopwatch_ellapsed_s()


    def _on_joint_velocity(self, c, value, ts):
        if value is None:
            return
        vel = np.zeros((len(self._joint_names),))
        for i in range(len(self._joint_names)):
            vel[i] = value[self._joint_names[i]]

        ep_vel = np.zeros((len(self._rox_robots),),dtype=self._spatial_velocity_dtype)

        try:
            for i in range(len(self._rox_robots)):
                joint_numbers = self._robot_info.chains[i].joint_numbers
                vel1 = vel[joint_numbers]
                pos1 = self._joint_position[joint_numbers]
                rox_ep_vel = rox.robotjacobian(self._rox_robots[i], pos1) @ vel1
                ep_vel[i] = self._geometry_util.array_to_spatial_velocity(rox_ep_vel)
        except:
            ep_vel = None
            traceback.print_exc()

        with self._lock:
            self._endpoint_vel = ep_vel
            self._joint_velocity = vel



    def _start_robot(self):
        
        super()._start_robot()
        time.sleep(0.5)
        self._gazebo_sub = RRN.SubscribeService(self._gazebo_url)
        self._gazebo_sub.ClientConnected += self._on_sub_connect

    
    def _send_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):
        if joint_pos_cmd is not None and self._position_command_wire is not None:
            cmd = {}
            for i in range(len(self._joint_names)):
                cmd[self._joint_names[i]] = joint_pos_cmd[i]
            
            self._position_command_wire.SetOutValueAll(cmd)
            self._position_command = joint_pos_cmd

    def _send_disable(self, handler):
        raise RR.NotImplementedException("Unsupported function")

    def _send_enable(self, handler):
        raise RR.NotImplementedException("Unsupported function")

    def _send_reset_errors(self, handler):
        raise RR.NotImplementedException("Unsupported function")

def main():

    # Read and parse command line arguments for service. Arguments starting with ``--robotraconteur-`` are passed
    # to the node setup
    parser = argparse.ArgumentParser(description="Gazebo Robot Raconteur driver for simulated robots")
    parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Robot info file (required)")
    parser.add_argument("--robot-name", type=str,default=None,help="Optional device name override")
    parser.add_argument("--world-name", type=str, required=False, default="default", help="the gazebo world name")
    parser.add_argument("--model-name", type=str, required=True, help="the gazebo model to control")
    parser.add_argument("--gazebo-url", type=str, required=True, help="url for the Robot Raconteur Gazebo plugin")
    args, _ = parser.parse_known_args()

    # Register standard Robot Raconteur service types with the node
    RRC.RegisterStdRobDefServiceTypes(RRN)
    
    # Read the RobotInfo yaml file to string
    with args.robot_info_file:
        robot_info_text = args.robot_info_file.read()

    # Parse the RobotInfo file
    info_loader = InfoFileLoader(RRN)
    robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "device")

    # Create the node attributes from RobotInfo. These attributes are made available during Robot Raconteur 
    # service discovery to help clients identify services
    attributes_util = AttributesUtil(RRN)
    robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

    robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot")
    op_modes = robot_const["RobotOperationalMode"]

    robot_op_mode = op_modes["auto"]

    # Create the robot driver object. This object extends AbstractRobot
    robot = GazeboRobotImpl(robot_info, args.gazebo_url, args.world_name, args.model_name, robot_op_mode)
    try:
        

        # Use ServerNodeSetup to initialize the server node
        with RR.ServerNodeSetup("experimental.gazebo_robot",58653,argv=sys.argv):
            # Call _start_robot() to start the robot loop
            robot._start_robot()
            time.sleep(0.5)
            
            # Register the service and add attributes from RobotInfo file
            service_ctx = RRN.RegisterService("robot","com.robotraconteur.robotics.robot.Robot",robot)
            service_ctx.SetServiceAttributes(robot_attributes)

            drekar_launch_process.wait_exit()
            robot._close()

    except:
        # Close robot if there is an error
        with suppress(Exception):
            robot._close()
        raise

if __name__ == "__main__":
    main()