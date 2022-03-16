#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <realsense_reader/PointCloudArray.h>
#include "realsense_reader/savepc.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include <pcl/io/auto_io.h>
class Message_handle{
public:
    ros::Publisher pcl_pub;
    ros::ServiceServer save_pc;
    sensor_msgs::PointCloud2 cur_pc;

    sensor_msgs::PointCloud2 output;
    tf2_ros::Buffer tfBuffer;
    geometry_msgs::TransformStamped transformStamped;
    int get_pc_flag;
    void pc_callback(const realsense_reader::PointCloudArray::ConstPtr& cloudMsgs);
    bool save_pc_callback(realsense_reader::savepc::Request &req,realsense_reader::savepc::Response &res);
    void rs_pc_callback(const sensor_msgs::PointCloud2& cloudMsgs);

};
void Message_handle::pc_callback(const realsense_reader::PointCloudArray::ConstPtr& cloudMsgs){
    ROS_INFO("get the pointcloud2");
    this->get_pc_flag=1;
    pcl::PCLPointCloud2* fused_cloud = new pcl::PCLPointCloud2;
    for(int i=0;i<cloudMsgs->clouds.size();i++){
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 
        // Convert to PCL data type
        pcl_conversions::toPCL(cloudMsgs->clouds[i], *cloud);
        std::string filename = "to_fuse_"+std::to_string(i)+".pcd";
        pcl::io::save(filename, *cloud);
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
    // automatically saved to ~/.ros
    pcl::io::save("test_fusion.pcd", *fused_cloud);
    // Convert to ROS data type
    pcl_conversions::moveFromPCL(*fused_cloud, this->output);

}
bool Message_handle::save_pc_callback(realsense_reader::savepc::Request &req,realsense_reader::savepc::Response &res){
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;

    pcl_conversions::toPCL(this->cur_pc, *cloud);
    std::string filename = "cur_"+req.filename+".pcd";
    pcl::io::save(filename, *cloud);
    
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    double center_x = this->transformStamped.transform.translation.x;
    double center_y = this->transformStamped.transform.translation.y;
    double center_z = this->transformStamped.transform.translation.z;

    crop.setInputCloud(cloudPtr);
    crop.setMin(Eigen::Vector4f(center_x-0.4,center_y-0.4,center_z-0.3,1.0));
    crop.setMax(Eigen::Vector4f(center_x+0.4,center_y+0.4,center_z+0.3,1.0));
    crop.filter(*cloud_filtered);

    filename = "filtered_"+req.filename+".pcd";
    pcl::io::save(filename, *cloud_filtered);

    return true;
}
void Message_handle::rs_pc_callback(const sensor_msgs::PointCloud2& cloudMsgs){
    cur_pc = cloudMsgs;

}
int main(int argc, char ** argv){
    ros::init(argc,argv,"pcl_process");
    ros::NodeHandle n;
    Message_handle msgh;

    ros::Subscriber pc_sub= n.subscribe("/pc_to_fuse",1000,&Message_handle::pc_callback,&msgh);
    ros::Subscriber real_sense_sub = n.subscribe("/camera/depth/color/points",1000,&Message_handle::rs_pc_callback,&msgh);
    msgh.save_pc = n.advertiseService("save_pc",&Message_handle::save_pc_callback,&msgh);
    msgh.pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/fused_pc", 1);
    msgh.get_pc_flag=0;
    ros::Rate loop_rate(100);
    tf2_ros::TransformListener tfListener(msgh.tfBuffer);
    while(ros::ok()){
        //Publish the data
        try{
            msgh.transformStamped = msgh.tfBuffer.lookupTransform("camera_depth_frame", "object",
                                  ros::Time(0));
            // ROS_WARN("get transform");
        }
        catch (tf2::TransformException &ex) {
            // ROS_WARN("%s",ex.what());
            continue;
        }

        if(msgh.get_pc_flag)
            msgh.pcl_pub.publish (msgh.output);
        ros::spinOnce();

    }
    // ros::spin();
    return 0;
}