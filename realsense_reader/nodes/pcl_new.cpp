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
#include <tf/transform_datatypes.h>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include <pcl/io/auto_io.h>
#include "std_msgs/String.h"
#include <tf2/LinearMath/Transform.h>
#include <pcl/registration/icp.h>
// #include <pcl/common/impl/transforms.hpp>
class Message_handle{
public:
    ros::Publisher pcl_pub;
    ros::ServiceServer save_pc;
    sensor_msgs::PointCloud2 cur_pc;
    tf2_ros::Buffer tfBuffer;
    geometry_msgs::TransformStamped transformStamped;
    tf::Transform fix_frame;
    std::string state;
    int first_flag;
    int publish_flag;
    int idx;
    tf::Transform prev_trans;

    pcl::PCLPointCloud2* fused_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> prev_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> GlobalCloud;
    Eigen::Matrix4f AccumuTrans;
    void pc_callback(const realsense_reader::PointCloudArray::ConstPtr& cloudMsgs);
    void rs_pc_callback(const sensor_msgs::PointCloud2& cloudMsgs);
    void arm_callback(const std_msgs::String msg);

};
void Message_handle::arm_callback(const std_msgs::String msg){
    if (msg.data=="finish_one"){
        // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        // pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
        // sensor_msgs::PointCloud2 pc_out;
        // transformPointCloud(this->cur_pc,pc_out,transform);
        // pcl_conversions::toPCL(this->cur_pc, *cloud);
        // pcl_conversions::toPCL(pc_out, *cloud);
        // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        // pcl::CropBox<pcl::PCLPointCloud2> crop;

        // crop.setInputCloud(cloudPtr);
        // crop.setMin(Eigen::Vector4f(center_x-0.4,center_y-0.4,center_z-0.3,1.0));
        // crop.setMax(Eigen::Vector4f(center_x+0.4,center_y+0.4,center_z+0.3,1.0));
        // crop.filter(*cloud_filtered);
        // if(first_flag){
        //     ROS_INFO("first_flag");
        //     geometry_msgs::Transform trans = this->transformStamped.transform;
        //     fix_frame = tf::Transform(tf::Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w), \
        //     tf::Vector3(trans.translation.x,trans.translation.y, trans.translation.z));
        //     pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        //     pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
        //     pcl_conversions::toPCL(this->cur_pc, *cloud);
        //     // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        //     // pcl::CropBox<pcl::PCLPointCloud2> crop;
        //     // crop.filter(*cloud_filtered);
        //     // crop.setInputCloud(cloudPtr);
        //     // crop.setMin(Eigen::Vector4f(center_x-0.4,center_y-0.4,center_z-0.3,1.0));
        //     // crop.setMax(Eigen::Vector4f(center_x+0.4,center_y+0.4,center_z+0.3,1.0));
        //     // *fused_cloud = *cloud_filtered;
        //     *fused_cloud = *cloud;
        //     first_flag = 0;
        // }
        // else{
        //     ROS_INFO("receive msg");
        //     tf::Transform Twb2;
        //     tf::Transform Tb2w;

        //     tf::Transform Tb1b2;
        //     tf::Transform Tb2b1;
        //     tf::Transform pure_trans;
        //     tf::Transform pure_rot;


        //     Eigen::Matrix4f out_max;
        //     Eigen::Matrix4f Twb1_mat;
        //     Eigen::Matrix4f Twb2_mat;
        //     Eigen::Matrix4f Tb2w_mat;
        //     Eigen::Matrix4f Tb1b2_mat;
        //     Eigen::Matrix4f Tb2b1_mat;
        //     Eigen::Matrix4f pure_trans_mat;
        //     Eigen::Matrix4f pure_rot_mat;

        //     geometry_msgs::Transform trans = this->transformStamped.transform;
        //     sensor_msgs::PointCloud2 pb1;
        //     sensor_msgs::PointCloud2 pb2;
        //     sensor_msgs::PointCloud2 pc_out;

        //     sensor_msgs::PointCloud2 pc_out1;
        //     sensor_msgs::PointCloud2 pc_out1_rot;
        //     sensor_msgs::PointCloud2 pc_out2;
        //     sensor_msgs::PointCloud2 pc_out3;

        //     sensor_msgs::PointCloud2 pb2_rot;

        //     Twb2 = tf::Transform(tf::Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w), \
        //     tf::Vector3(trans.translation.x, trans.translation.y, trans.translation.z));
        //     tf::Transform fix_frame_new= this->fix_frame;
        //     pcl_ros::transformAsMatrix(Twb2,Twb2_mat);

        //     // pure_trans = tf::Transform(tf::Quaternion(0,0,0,1), \
        //     // tf::Vector3(-trans.translation.x, -trans.translation.y, -trans.translation.z));
        //     // pcl_ros::transformAsMatrix(pure_trans,pure_trans_mat);
        //     // std::cout << "Twb2" << std::endl;
        //     // std::cout << Twb2_mat << std::endl;

        //     // Tb2b1 = Twb2.inverse()*fix_frame_new;
        //     // pcl_ros::transformAsMatrix(Tb2b1,Tb2b1_mat);
        //     // std::cout << "Tb2b1_mat" << std::endl;
        //     // std::cout << Tb2b1_mat << std::endl;

        //     // pcl_ros::transformPointCloud(pure_trans_mat,this->cur_pc,pc_out1);

        //     // pure_rot = tf::Transform(tf::Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w), \
        //     // tf::Vector3(0,0,0));

        //     // pcl_ros::transformAsMatrix(pure_rot,pure_rot_mat);
        //     // std::cout << "pure_rot_mat" << std::endl;
        //     // std::cout << pure_rot_mat << std::endl;

        //     // pcl_ros::transformPointCloud(pure_rot_mat,pc_out1,pc_out1_rot);
        //     // pcl_ros::transformPointCloud(Tb2b1_mat,pc_out1_rot,pc_out2);

        //     // pure_rot = tf::Transform(tf::Quaternion(trans.rotation.x, -trans.rotation.y, -trans.rotation.z, -trans.rotation.w), \
        //     // tf::Vector3(0,0,0));
        //     // pcl_ros::transformAsMatrix(pure_rot,pure_rot_mat);
        //     // std::cout << "pure_rot_mat" << std::endl;
        //     // std::cout << pure_rot_mat << std::endl;


        //     // pcl_ros::transformPointCloud(pure_rot_mat,pc_out2,pc_out3);

        //     // pure_trans = tf::Transform(tf::Quaternion(0,0,0,1), \
        //     // tf::Vector3(trans.translation.x, trans.translation.y, trans.translation.z));
        //     // pcl_ros::transformAsMatrix(pure_trans,pure_trans_mat);
        //     // pcl_ros::transformPointCloud(pure_trans_mat,pc_out3,pc_out);


        //     Tb2w = Twb2.inverse();
        //     pcl_ros::transformAsMatrix(Tb2w,Tb2w_mat);
        //     pcl_ros::transformPointCloud(Tb2w_mat,this->cur_pc,pb2);
        //     //Tb1w*Twb2 = Tb1b2
        //     Tb1b2 = fix_frame_new.inverse()*(Twb2);
        //     Tb2b1 = Twb2.inverse()*fix_frame_new;
        //     pcl_ros::transformAsMatrix(Tb1b2,Tb1b2_mat);
        //     pcl_ros::transformAsMatrix(Tb2b1,Tb2b1_mat);
        //     // std::cout << "outputpc" << Tb1b2_mat << std::endl;
        //     // //the pointcluod in P1 frame;
        //     //rotate the pointcloud
        //     // pcl_ros::transformPointCloud(Tb2b1_mat,pb2,pb2_rot);
        //     // //Transform them back to the world frame
        //     pcl_ros::transformAsMatrix(fix_frame_new,Twb1_mat);
        //     std::cout << "Twb2" << Twb2_mat << std::endl;
        //     std::cout << "Tb2w" << Tb2w_mat << std::endl;
        //     std::cout << "Twb1" << Twb1_mat << std::endl;
        //     std::cout << "Tb1b2" << Tb1b2_mat << std::endl;
        //     pcl_ros::transformPointCloud(Twb1_mat,pb2,pc_out);


        //     pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        //     pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
        //     // pcl_conversions::toPCL(this->cur_pc, *cloud);
        //     pcl_conversions::toPCL(pc_out, *cloud);

        //     // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        //     // pcl::CropBox<pcl::PCLPointCloud2> crop;
        //     // crop.filter(*cloud_filtered);
        //     // crop.setInputCloud(cloudPtr);
        //     // crop.setMin(Eigen::Vector4f(center_x-0.4,center_y-0.4,center_z-0.3,1.0));
        //     // crop.setMax(Eigen::Vector4f(center_x+0.4,center_y+0.4,center_z+0.3,1.0));
        //     // *fused_cloud += *cloud_filtered;
        //     std::string filename = "to_fuse_"+std::to_string(idx)+".pcd";
        //     idx+=1;

        //     *fused_cloud += *cloud;  
        //     pcl::io::save(filename, *cloud);

        // }
        double center_x = this->transformStamped.transform.translation.x;
        double center_y = this->transformStamped.transform.translation.y;
        double center_z = this->transformStamped.transform.translation.z;

        if(first_flag){
            publish_flag=1;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(prev_cloud));
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(prev_cloud));
            pcl::fromROSMsg(this->cur_pc,*tmp_cloud_Ptr);

            pcl::CropBox<pcl::PointXYZRGB> crop;
            crop.setInputCloud(tmp_cloud_Ptr);
            crop.setMin(Eigen::Vector4f(center_x-0.15,center_y-0.15,center_z-0.15,1.0));
            crop.setMax(Eigen::Vector4f(center_x+0.15,center_y+0.15,center_z+0.15,1.0));
            crop.filter(*cur_cloud_Ptr);

            geometry_msgs::Transform trans = this->transformStamped.transform;
            fix_frame = tf::Transform(tf::Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w), 
             tf::Vector3(trans.translation.x,trans.translation.y, trans.translation.z));
            prev_trans = tf::Transform(tf::Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w), \
            tf::Vector3(trans.translation.x, trans.translation.y, trans.translation.z));
            first_flag = 0;
            prev_cloud = *cur_cloud_Ptr;

        }
        else
        {   
            Eigen::Matrix4f Twb2_mat;
            Eigen::Matrix4f Tb1b2_mat;

            tf::Transform Twb2;
            tf::Transform Tb1b2;
            

            geometry_msgs::Transform trans = this->transformStamped.transform;

            Twb2 = tf::Transform(tf::Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w), \
            tf::Vector3(trans.translation.x, trans.translation.y, trans.translation.z));
            pcl_ros::transformAsMatrix(Twb2,Twb2_mat);
            Tb1b2 = prev_trans.inverse()*(Twb2);
            pcl_ros::transformAsMatrix(Tb1b2,Tb1b2_mat);
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(prev_cloud));
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


            pcl::fromROSMsg(this->cur_pc,*tmp_cloud_Ptr);
            pcl::CropBox<pcl::PointXYZRGB> crop;
            crop.setInputCloud(tmp_cloud_Ptr);
            crop.setMin(Eigen::Vector4f(center_x-0.15,center_y-0.15,center_z-0.15,1.0));
            crop.setMax(Eigen::Vector4f(center_x+0.15,center_y+0.15,center_z+0.15,1.0));
            crop.filter(*cur_cloud_Ptr);


            icp.setInputSource(prev_cloud_Ptr);
            icp.setInputTarget(cur_cloud_Ptr);

            // Set the max correspondence distance to 5cm (e.g., correspondences with higher
            // distances will be ignored)
            icp.setMaxCorrespondenceDistance (0.05);
            // Set the maximum number of iterations (criterion 1)
            icp.setMaximumIterations (80);
            // Set the transformation epsilon (criterion 2)
            icp.setTransformationEpsilon (0.1);
            // Set the euclidean distance difference epsilon (criterion 3)
            icp.setEuclideanFitnessEpsilon (1);
            std::cout << "Tb1b2_mat" << std::endl;
            std::cout << Tb1b2_mat << std::endl;
            icp.align(*aligned_cloud_Ptr,Tb1b2_mat);
            prev_trans = Twb2;
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            // transformation(0,3) = 0;
            // transformation(1,3) = 0;
            // transformation(2,3) = 0;

            pcl::PointCloud<pcl::PointXYZRGB> trans_cloud;
            AccumuTrans = AccumuTrans*transformation;

            tf::Transform Tbbn;
            tf::Transform Twbbn;
            Eigen::Matrix4f Twbbn_mat;
            Eigen::Matrix3f rotate;
            rotate = AccumuTrans.block(0,0,3,3);
            Eigen::Quaternionf q(rotate);
            Tbbn = tf::Transform(tf::Quaternion(q.x(),q.y(),q.z(),q.w()),\
            tf::Vector3(AccumuTrans(0,3), AccumuTrans(1,3), AccumuTrans(2,3)));
            Twbbn = fix_frame*(fix_frame*Tbbn).inverse();
            pcl_ros::transformAsMatrix(Twbbn,Twbbn_mat);

            std::cout << "transformation" <<std::endl;
            std::cout << transformation <<std::endl;

            pcl::transformPointCloud(*cur_cloud_Ptr,trans_cloud,Twbbn_mat);
            std::cout <<"Accumtrans" << std::endl;
            std::cout << AccumuTrans << std::endl;
            std::string filename = "to_fuse_"+std::to_string(idx)+".pcd";

            GlobalCloud += trans_cloud;
            prev_cloud = *cur_cloud_Ptr;

            pcl::io::save(filename, GlobalCloud);


            idx+=1;

        }

    }
    else if(msg.data=="finish_all"){
        ROS_INFO("finish_all");

        pcl::io::save("test_fusion.pcd", GlobalCloud);
        first_flag = 1;
    }
}
void Message_handle::rs_pc_callback(const sensor_msgs::PointCloud2& cloudMsgs){
    cur_pc = cloudMsgs;
}
int main(int argc, char ** argv){
    ros::init(argc,argv,"pcl_process");
    ros::NodeHandle n;
    Message_handle msgh;

    ros::Subscriber real_sense_sub = n.subscribe("/camera/depth/color/points",1000,&Message_handle::rs_pc_callback,&msgh);
    // msgh.pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/fused_pc", 1);
    ros::Subscriber arm_sub = n.subscribe("/arm_state",1000,&Message_handle::arm_callback,&msgh);
    msgh.first_flag=1;
    msgh.publish_flag=0;
    msgh.idx = 1;
    msgh.fused_cloud = new pcl::PCLPointCloud2;
    msgh.state = "start";
    // msgh.fixed_frame = Eigen::Matrix4f::Identity();

    msgh.AccumuTrans = Eigen::Matrix4f::Identity();
    ros::Rate loop_rate(30);
    tf2_ros::TransformListener tfListener(msgh.tfBuffer);
    msgh.pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/fused_pc", 1);

    while(ros::ok()){
        //Publish the data
        try{
            msgh.transformStamped = msgh.tfBuffer.lookupTransform("camera_depth_optical_frame", "object",
                                  ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            continue;
        }
        if(msgh.publish_flag){
            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(msgh.prev_cloud, cloud);
            msgh.pcl_pub.publish(cloud);
        }
        

        ros::spinOnce();

    }
    // ros::spin();
    return 0;
}