#! /usr/bin/env python
import sys
from time import sleep
import moveit_commander
import moveit_msgs.msg
import rospy
import geometry_msgs.msg
from std_srvs.srv import Empty,EmptyResponse
from adroit_move.srv import step,run_once,reset
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64,String
import copy 
from math import pi

class Mover():
    def __init__(self):
        #fist initialize "moveit_commander" and a "rospy" node
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("mover")

        #instantiate a robotcommander object, provides information such
        #robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        #instantiate a "planningsceneinterface" object. provides a remote
        #interface for robot world
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        gripper_name = "pincer"
        gripper_group = moveit_commander.MoveGroupCommander(gripper_name)

        #used to dispaly trajectories in rviz
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        #getting basic information
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        print("============ Printing current pose")
        print(move_group.get_current_pose())

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        # self.move_group.set_goal_position_tolerance(0.01)
        
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_ink = eef_link
        self.group_names = group_names
        #add a table at the beginning to the planning scene
        # self.add_box("table",pos = [0,0,4],size = (1,2,0.4))

        # self.run_once = rospy.get_param("~run_once")
        self.waypoints = rospy.get_param("/waypoints")
        self.gripper_state = rospy.get_param("/gripper")
        self.gripper_strength = rospy.get_param("/gripper_strength")
        #define the service here
        self.reset = rospy.Service("reset",reset,self.reset_cback)
        self.step = rospy.Service("step",step,self.step_cback)
        self.follow = rospy.Service("follow",Empty,self.follow_cback)
        self.fusion = rospy.Service("start_fusion",Empty,self.fusion_cback)
        self.reset_proxy = rospy.ServiceProxy("reset",reset)
        # self.gripper = rospy.Service("control_gripper",control_gripper,self.grip_cback)
        self.gripper_pub = rospy.Publisher("/hdt_arm/pincer_joint_position_controller/command",Float64,queue_size = 100)
        self.arm_pub = rospy.Publisher("/arm_state",String,queue_size=100)
        #set the timer to run on the main_loop
        self.timer = rospy.Timer(rospy.Duration(50), self.main_loop)

        #add box 
        # box_name = "realsense"
        # if(box_name not in self.scene.get_known_object_names()):
        #     self.add_box(box_name,pos=[0.54,0.54,0.60],size = (0.115,0.115,0.06))

        self.add_box("ground",pos=[0,-0.5,-0.38],size = (0.6,0.6,0.12))
        self.add_box("object",pos=[0,-0.5,-0.3],size=(0.05,0.05,0.06))


    def go_to_pos_goal(self,pos,gripper):
        plan = self.plan_cartesian_path(pos)
        # print(plan)
        self.move_group.execute(plan,wait=True)
        print("plan success")
        #make sure you there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        success_flag = True
        if gripper:
            self.gripper_pub.publish(self.gripper_strength)
        else:
            self.gripper_pub.publish(1.0)
        print("current state")
        print(self.robot.get_current_state())
        print("current pos")
        print(self.move_group.get_current_pose())
        return success_flag

    def go_to_joint_goal(self,joint,gripper):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0]=joint[0]
        joint_goal[1]=joint[1]
        joint_goal[2]=joint[2]
        joint_goal[3]=joint[3]
        joint_goal[4]=joint[4]
        joint_goal[5]=joint[5]
        plan = self.move_group.go(joint_goal,wait=True)
        #make sure you there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if not gripper:
            self.gripper_pub.publish(1.0)

    def step_cback(self,req):
        success_flag = 1
        print("step sucess")
        self.go_to_pos_goal(req.position,req.gripper)
        print("go to pos")
        # if success_flag:
        #     self.waypoints.append([req.position.x,req.position.y,req.position.z])
        #     self.gripper_state.append(req.gripper)
        
        return True

    def follow_cback(self,req):
        if len(self.waypoints)==0:
            rospy.loginfo("no waypoints to follow!")
        else:
            for i in range(len(self.waypoints)):
                self.go_to_pos_goal(Vector3(self.waypoints[i][0],self.waypoints[i][1],self.waypoints[i][2]),\
                    self.gripper_state[i])
            self.go_to_joint_goal([0,0,57/180*pi,0,0,0],True)
        return EmptyResponse()

    def reset_cback(self,req):
        self.go_to_joint_goal([-pi/2+0.1,0,0,0,-pi/3,-1.5*pi],req.gripper)
        return True

    def fusion_cback(self,req):
        self.go_to_joint_goal([-pi/4,-pi/6,pi/3,pi,-pi/2,-1.5*pi],True)
        for i in range(6):
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[5]+=(pi/8)*(i+1)
            plan = self.move_group.go(joint_goal,wait=True)
            #make sure you there is no residual movement
            self.move_group.stop()
            self.arm_pub.publish("finish_one")
            self.move_group.clear_pose_targets()
            sleep(0.1)
        self.arm_pub.publish("finish_all")

        return EmptyResponse()
        
    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            if(box_name in scene.get_known_object_names()):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_box(self,name,pos,size,timeout = 4):
        box_name = name
        self.box_name = box_name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = pos[0]
        box_pose.pose.position.y = pos[1]
        box_pose.pose.position.z = pos[2]
        box_pose.pose.orientation.w = 1.0
        self.scene.add_box(box_name,box_pose,size = size)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def plan_cartesian_path(self,pos):
        wp = []
        wpose = self.move_group.get_current_pose().pose
        wpose.position.z += pos.z  # First move up (z)
        wpose.position.y += pos.y  # and sideways (y)
        wpose.position.x += pos.x  # Second move forward/backwards in (x)

        wp.append(copy.deepcopy(wpose))

        # wpose.position.x += pos.x  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            wp, 0.01, 0.0  # wp to follow  # eef_step
        )  # jump_threshold
        return plan
    def main_loop(self,event=None):
        try:
            
            # print("true")
            a = 1
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    move = Mover()
    rospy.spin()







