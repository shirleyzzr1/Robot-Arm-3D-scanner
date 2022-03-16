import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from realsense_reader.msg import PointCloudArray
from enum import Enum, auto
from std_srvs.srv import Empty,EmptyResponse
class State(Enum):
    """ The state of the control loop 
        These are different modes that the controller can be in
    """
    START = auto()
    END = auto()
class Reader():
    def __init__(self):
        self.depth_sub = rospy.Subscriber("/camera/depth/color/points",PointCloud2,self.depth_callback)
        self.arm_sub = rospy.Subscriber("/arm_state",String,self.armstate_callback)
        self.pc_pub = rospy.Publisher("pc_to_fuse",PointCloudArray,queue_size=10)
        self.read_num = 0
        self.curpc = PointCloud2()
        self.pc_fusion = []
        self.state = State.END
        self.fusion_flag = False
        self.trans = 0
        # self.rate = rospy.Rate(50)
        #set tf2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.timer = rospy.Timer(rospy.Duration(0.02), self.main_loop)

    def depth_callback(self,msg):
        self.curpc = msg
        return EmptyResponse()
    def armstate_callback(self,msg):
        print("msg",msg)
        if msg.data=="finish_one":
            print("state_start")
            self.state = State.START
        elif msg.data=="finish_all":
            self.state = State.END
            self.fusion_flag=True

        return EmptyResponse()
    
    def main_loop(self,event=None):
        # while not rospy.is_shutdown:
        try:
            self.trans = self.tfBuffer.lookup_transform("camera_depth_frame", "object", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        if self.state==State.START:
            rospy.loginfo("get one pc")
            print("get one pc")
            # newpc = do_transform_cloud(self.curpc,self.trans)
            newpc = self.curpc
            self.pc_fusion.append(newpc)
            self.state = State.END
        elif self.state==State.END:
            #fuse the pointcloud, make sure it only be called once 
            if (self.fusion_flag): 
                print("fusion")
                self.pc_pub.publish(self.pc_fusion)
                self.fusion_flag=False
        
            # self.rate.sleep()
            
if __name__=="__main__":
    rospy.init_node("read_camera")
    reader = Reader()
    # reader.main_loop()
    rospy.spin()