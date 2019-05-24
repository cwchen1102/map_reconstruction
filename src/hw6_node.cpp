#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <string.h>
#include <vector>


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix4f trans;
bool map_done = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    pcl::PCLPointCloud2::Ptr pcl_cloud2 (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pcl_cloud2_filtered (new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
    pcl::toPCLPointCloud2(*msg,*pcl_cloud2);
    downsample.setInputCloud(pcl_cloud2);
    downsample.setLeafSize(0.8f,0.8f,0.8f);
    downsample.filter(*pcl_cloud2_filtered);
    pcl::fromPCLPointCloud2(*pcl_cloud2_filtered,*cloud_filtered);
    return cloud_filtered;
}

void cloud_data(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg1, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg2)
{
    //icp
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg3 (new pcl::PointCloud<pcl::PointXYZ>);
    icp.setInputSource(msg2);
    icp.setInputTarget(msg1);
    icp.align(*msg3,trans);
    //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    //icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;
    trans = icp.getFinalTransformation();
    *Final = *msg1;
    *Final += *msg3;
    std::cout <<"input data size = "<<msg3->points.size() << std::endl;
    std::cout <<"total data size = "<<Final->points.size() << std::endl;

}

void tf_brocast(Eigen::Matrix4f trans){
  static tf::TransformBroadcaster br;
  tf::Transform tf_map_scan;
  Eigen::Quaternionf q(trans.topLeftCorner<3, 3>());
  Eigen::Vector3f v = trans.topRightCorner<3, 1>();
  tf_map_scan.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_map_scan.setOrigin(tf::Vector3(v(0), v(1), v(2)));
  br.sendTransform(tf::StampedTransform(tf_map_scan, ros::Time::now(), "map", "scan"));
}


int main(int argc, char** argv)
{
    //pcd data path
    std::string dir = "/home/ncrl/catkin_ws2/src/hw6_0751081/pcd/";
    std::string filename = ".pcd";
    std::vector<pcl::PointCloud<pcl::PointXYZ> > v;
    for (int i = 1; i < 180; ++i)
    {
        std::stringstream file;
        file<<dir<<i<<filename;

        //load map
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (file.str(), *cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file map.pcd \n");
          return (-1);
        }
        std::cout << "cloud data_" << i << "Loaded"<< std::endl;
        cloud_filtered = downsample(cloud);
        v.push_back(*cloud_filtered);
        //std::cout << v[i] << std::endl;
    }

    ros::init(argc, argv, "hw6");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/sensor/msgs/PointCloud2", 1000);
    ros::Rate rate(30);

    //initialization
    Eigen::Matrix4f intial_guess;
    intial_guess << 1.0,0.0,0.0,0.0,
                    0.0,1.0,0.0,0.0,
                    0.0,0.0,1.0,0.0,
                    0.0,0.0,0.0,1.0;
    trans = intial_guess;

    *cloud_filtered_1 = v[1];
    int i = 0;

    while (ros::ok())
    {
        if (map_done == false)
        {
            *cloud_filtered_2 = v[i];
            //cloud_filtered_2->points.resize (cloud_filtered_1->width * cloud_filtered_1->height);
            cloud_data(cloud_filtered_1,cloud_filtered_2);
            cloud_filtered_1 = Final;
            std::cout << "==========="<<i<<"===========" << std::endl;
            i++;

            Final->header.frame_id = "/map";
            map_pub.publish(Final);
            tf_brocast(trans);
            ros::spinOnce();
            rate.sleep();
            if (i==179)
                {
                map_done = true;
                }
            }
        else
        {
            pcl::io::savePCDFileASCII ("map.pcd", *Final);
            std::cerr << "Saved data points to map.pcd." << std::endl;
            break;
        }

    }


    return 0;

}

