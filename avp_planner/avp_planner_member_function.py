import sys
sys.path.append("/usr/local/lib/python3/dist-packages")
import rclpy
from rclpy.node import Node
import numpy as np
import TOPP
import string
from .AVPRRT import *
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.robot_state import RobotState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.logging import get_logger
import time

# moveit_py functionality for coll.checking and simulation
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

class AVP_Planner(Node):

    def __init__(self):
        super().__init__('avp_planner')

        self.logger = get_logger("main_logger")
        self.logger.info("Initializing moveit instance")

        # Instantiating the moveit configurations
        self.moveit_py = MoveItPy(node_name="ur10e")
        self.logger.info("MoveItPy instance created")

        self.planning_scene_monitor = self.moveit_py.get_planning_scene_monitor()
        self.robot = self.moveit_py.get_planning_component("ur_manipulator")
        self.robot_model = self.moveit_py.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        self.plantime = []
        self.topptime = []

        # Now follows a first example to see the robot moving
        self.robot.set_start_state_to_current_state()
        
        # randomize the robot state
        self.robot_state.set_to_random_positions()

        # define desired position (which is random)
        self.robot.set_goal_state(robot_state=self.robot_state)

        # plan and execute both via moveit!!
        self.plan_and_execute(sleep_time=3.0)


        ###########################################################################
        # Plan 1 - set states with predefined string
        ###########################################################################

        # set plan start state using predefined state
        #ur_arm.set_start_state(configuration_name="ready")

        # set pose goal using predefined state
        #ur_arm.set_goal_state(configuration_name="extended")

        # plan to goal
        #self.plan_and_execute(self, ur_arm, logger, sleep_time=3.0)



        # initializing the publisher and topic where we will publish the planned state trajectory
        # Check if ros functionality still works
        #self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Wait for current state, destroy after message is first received
        #self.subscriber_ = self.create_subscription(JointTrajectoryControllerState, '/joint_trajectory_controller/state', self.state_callback,10)
        #self.subscriber_ # avoid unused variable warning

        self.define_params()

        # Defining start config 
        cStart = Config(self.qStart, self.qdStart, self.qddStart) # instance of config class
        vStart = Vertex(cStart, FW) # instance of vertex class 
        vStart.sdmin = self.sdbeg
        vStart.sdmax = self.sdbeg
    
        # Defining end config
        cGoal = Config(self.qGoal, self.qdGoal, self.qddGoal)
        vGoal = Vertex(cGoal, BW) # adding vertices forward means searching for final connection from the back
        vGoal.sdmin = self.sdend
        vGoal.sdmax = self.sdend

        self.birrtinstance = AVPBiRRTPlanner(vStart, vGoal, self.problemtype, 
                                            self.constraintsstring0, self.tuningsstring0, 
                                            self.nn, self.metrictype, self.polynomialdegree, ndof=6)
        
        # Get a plan via AVP-Planner
        self.birrtinstance.Run(self.allottedtime)
        self.plantime.append(self.birrtinstance.runningtime)

        # Run TOPP one last time to recover full solution
        if self.birrtinstance.found:
            self.Recover_start_to_end()
            self.SendTrajectoryToRobot() # send the trajecotry via moveit

    def plan_and_execute(self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
        ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("PLANNING TRAJECTORY")
        if multi_plan_parameters is not None:
            plan_result = self.robot.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = self.robot.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            self.logger.info("we want to plan now")
            plan_result = self.robot.plan()

        # execute the plan
        if plan_result:
            robot_trajectory = plan_result.trajectory
            # plots the resulting trajectory
            # self.logger.info(str(robot_trajectory.get_robot_trajectory_msg()))
            self.moveit_py.execute(robot_trajectory, controllers=[])
            #RobotTrajectory().
            self.logger.info("plan executed")

        time.sleep(sleep_time)  


    # All params that are needed, are defined here
    def define_params(self):
        self.problemtype = "KinematicLimits"

        # params for TOPP integration during expansion
        integrationtimestep0 = .5e-3
        reparamtimestep0 = 1e-2
        passswitchpointnsteps0 = 5
        discrtimestep0 = 0.5e-3

        # params for last TOPP integration
        self.integrationtimestep1 = .5e-3
        self.reparamtimestep1 = 1e-2
        self.passswitchpointnsteps1 = 5
        self.discrtimestep1 = 0.5e-3

        self.sdbeg = 0.0
        self.sdend = 0.0
        self.allottedtime = 600.0
        self.nn = 10
        self.metrictype = 1
        self.polynomialdegree = 5

        # joint limits
        qL = -90*np.ones((6))
        qU = 90*np.ones((6))
        qdU = 90.0 * np.ones((6))
        qddU = 60.0 * np.ones((6))

        # tuning and constraint params for TOPP in during expansion
        self.tuningsstring0 = "%f %f %d"%(integrationtimestep0, reparamtimestep0, passswitchpointnsteps0)    
        self.constraintsstring0 = str(discrtimestep0)
        self.constraintsstring0 += "\n" + ' '.join([str(v) for v in qL])
        self.constraintsstring0 += "\n" + ' '.join([str(v) for v in qU])
        self.constraintsstring0 += "\n" + ' '.join([str(v) for v in qdU])
        self.constraintsstring0 += "\n" + ' '.join([str(a) for a in qddU])

        # constraints params for last TOPP integration
        self.constraintsstring1 = str(self.discrtimestep1)
        self.constraintsstring1 += "\n" + ' '.join([str(v) for v in qdU])
        self.constraintsstring1 += "\n" + ' '.join([str(a) for a in qddU])

        # goal and start configurations
        self.qStart = np.ones(6)
        self.qStart[0] = 0
        self.qStart[1] = -90
        self.qStart[2] = 0
        self.qStart[3] = -90
        self.qStart[4] = 0
        self.qStart[5] = 0

        self.qGoal = np.ones(6)
        self.qGoal[0] = 40
        self.qGoal[1] = -120
        self.qGoal[2] = 20
        self.qGoal[3] = -100
        self.qGoal[4] = 20
        self.qGoal[5] = 60

        v = 1e-6
        self.qdStart = v*np.ones(6)
        self.qdGoal = v*np.ones(6)

        self.qddStart = np.zeros(6)
        self.qddGoal = np.zeros(6)
    
    #def state_callback(self, state):
    #    print("ok till here")
    #    self.subscriber_.destroy()

    def Recover_start_to_end(self):
        print("recovering solution")
        tsTOPP = time.time()
        trajectorystring0 = self.birrtinstance.GenerateFinalTrajectoryString() # recover partial trajectories from start to end
    
        x = TOPP.TOPPbindings.TOPPInstance(None, self.problemtype, self.constraintsstring1, trajectorystring0)
        x.integrationtimestep = self.integrationtimestep1
        x.reparamtimestep = self.reparamtimestep1
        x.extrareps = 5
        x.passswitchpointnsteps = self.passswitchpointnsteps1
        
        ret = x.RunComputeProfiles(self.sdbeg, self.sdend)
        if (ret == 1):
            x.ReparameterizeTrajectory()

            x.WriteResultTrajectory()

            # Can extract information on q and qd and qdd from traj1.Eval, traj1.Evald and traj1.Evaldd
            # more specifically: traj1.Eval(0.5) gives q-config at time 0.5!!!
            self.traj1 = TOPP.Trajectory.PiecewisePolynomialTrajectory.FromString(x.restrajectorystring)

            teTOPP = time.time()

            self.plantime.append(self.birrtinstance.runningtime)
            self.topptime.append(teTOPP - tsTOPP)

            print("AVPRRT running time = {0} sec.".format(self.plantime[-1]))
            print("TOPP running time = {0} sec.".format(self.topptime[-1]))
            print("trajectory duration.", self.traj1.duration)

    # Sending the whole trajectory for execution
    def SendTrajectoryToRobot(self):

        #Robot_traj_object = RobotTrajectory(self.robot_model)

        joint_traj = JointTrajectory()

        joint_traj.joint_names = self.get_joint_names() # names of each element of the robot

        joint_traj.header = self.get_header() # header information
     
        number_points = 100 # in the end we have one point more b.c of t = 0
        
        for iterator in range(number_points + 1):
            point = self.to_joint_trajectory_point(iterator, number_points)  # Assuming this method exists in your Chunk class
            joint_traj.points.append(point)

        # sending it via moveitpy
        #traj = Robot_traj_object.set_robot_trajectory_msg(joint_traj)

        #self.moveit_py.execute(joint_traj, controllers=[])

        #print(joint_traj)
        #print(self.traj1.Eval(self.traj1.duration))
        
        #self.publisher_.publish(joint_traj)


    # Return the names of the Ur10
    def get_joint_names(self):
        names_list = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        return names_list
    
    # Saving header information
    def get_header(self):
        header = Header()
        current_time = time.time()  
        header.stamp.sec = int(current_time)  # Extract seconds part
        header.stamp.nanosec = int((current_time - header.stamp.sec) * 1e9)  # Extract the nanoseconds part
        header.frame_id = "configuration space"  # Adjust this according to your frame ID
        
        return header
    
    # recovers the right format (trajectory_point), which is the message we need
    def to_joint_trajectory_point(self, iterator, number_points):
        #placeholder for current configuration
        point = JointTrajectoryPoint()
        # time at given iteration
        time = iterator / number_points * self.traj1.duration 

        # extract q, qd and qdd from calculated trajectory, given the current time
        point.positions = np.deg2rad(self.traj1.Eval(time)).tolist()
        point.velocities = np.deg2rad(self.traj1.Evald(time)).tolist()
        point.accelerations = np.deg2rad(self.traj1.Evaldd(time)).tolist()
        point.time_from_start.sec = int(time)
        point.time_from_start.nanosec = int((time - point.time_from_start.sec) * 1e9)

        return point

def main(args=None):
    rclpy.init(args=args)

    avp_planner = AVP_Planner()

    rclpy.spin(avp_planner)

    # Explicitely destroying the node
    avp_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()