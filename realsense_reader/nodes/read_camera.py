from tokenize import String
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from realsense_reader.msg import PointCloudArray
from enum import Enum, auto
class State(Enum):
    """ The state of the control loop 
        These are different modes that the controller can be in
    """
    START = auto()
    END = auto()
class Reader():
    def __init__(self) -> None:
        self.timer = rospy.Timer(rospy.Duration(0.1), self.main_loop)
        self.depth_sub("/camera/depth/color/points",PointCloud2,self.depth_callback)
        self.arm_sub("/arm_state",String,self.armstate_callback)
        self.pc_pub("/pc_to_fuse",PointCloudArray,queue_size=10)
        self.read_num = 0
        self.curpc = PointCloud2()
        self.pc_fusion = []
        self.state = State.END
        self.fusion_flag = False
    def depth_callback(self,msg):
        self.curpc = msg
    def armstate_callback(self,msg):
        if msg=="finish_one":
            self.state = State.START
        elif msg=="finish_all":
            self.state = State.END
    def main_loop(self,event = None):
        #start collecting data for 3d scanner
        if self.state==State.START:
            tf_buffer = tf2_ros.Buffer()
            trans = tf_buffer.lookup_transform("camera", "object",rospy.Time.now())
            newpc = do_transform_cloud(self.curpc,trans)
            self.pc_fusion.append(newpc)
            self.fusion_flag=True
            self.state = State.END
        elif self.state==State.END:
            #fuse the pointcloud, make sure it only be called once 
            if (self.fusion_flag): 
                print("fusion")
                self.pc_pub.publish(self.pc_fusion)
                self.fusion_flag=False
            
if __name__=="main":
    rospy.init_node("read_camera")
    reader = Reader()
    reader.main_loop()