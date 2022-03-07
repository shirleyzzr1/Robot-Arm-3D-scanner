#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <realsense_reader/PointCloudArray.h>
#include <pcl/io/auto_io.h>
class Message_handle{
public:
    ros::Publisher pcl_pub;

    void pc_callback(const realsense_reader::PointCloudArray::ConstPtr& cloudMsgs);
};
void Message_handle::pc_callback(const realsense_reader::PointCloudArray::ConstPtr& cloudMsgs){

    pcl::PCLPointCloud2* fused_cloud = new pcl::PCLPointCloud2;
    for(int i=0;i<cloudMsgs->clouds.size();i++){
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 
        // Convert to PCL data type
        pcl_conversions::toPCL(cloudMsgs->clouds[i], *cloud);
        // pcl::PCLPointCloud2 cloud_filtered;

        // Convert to PCL data type
        // pcl_conversions::toPCL(cloudMsgs->clouds[i], cloud);

        //fuse all the pointcloud
        *fused_cloud += *cloud;
        // // Perform the actual filtering
        // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        // sor.setInputCloud (cloudPtr);
        // sor.setLeafSize (0.1, 0.1, 0.1);
        // sor.filter (cloud_filtered);

    }
    pcl::io::save("test_fusion.bin", *fused_cloud);
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(*fused_cloud, output);

    //Publish the data
    pcl_pub.publish (output);
}
int main(int argc, char ** argv){
    ros::init(argc,argv,"pcl_process");
    ros::NodeHandle n;
    Message_handle msgh;

    ros::Subscriber pc_sub= n.subscribe("/pc_to_fuse",1000,&Message_handle::pc_callback,&msgh);
    // msgh.pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/fused_pc", 1);

    ros::spin();
    return 0;
}