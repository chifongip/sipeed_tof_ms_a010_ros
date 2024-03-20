#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

using namespace std;

class PointCloudProcessor
{
public:
  PointCloudProcessor(ros::NodeHandle nh)
    : nh_(nh)
  {
    nh_.getParam("point_cloud_processing_node/sub_topic", sub_topic);
    nh_.getParam("point_cloud_processing_node/MeanK", MeanK);
    nh_.getParam("point_cloud_processing_node/StddevMulThresh", StddevMulThresh);
    nh_.getParam("point_cloud_processing_node/RadiusSearch", RadiusSearch);
    nh_.getParam("point_cloud_processing_node/MinNeighborsInRadius", MinNeighborsInRadius);

    // Create a subscriber for the PointCloud2 message
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 1, &PointCloudProcessor::pointCloudCallback, this);

    // Create a publisher for the processed PointCloud2 message
    processed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Convert PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Perform voxel grid downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*downsampled_cloud);

    // Perform outlier removal using StatisticalOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(downsampled_cloud);
    sor.setMeanK(MeanK);  // Number of neighbors to analyze for each point
    sor.setStddevMulThresh(StddevMulThresh);  // Standard deviation threshold
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_cloud);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(filtered_cloud);
    ror.setRadiusSearch(RadiusSearch);  // Radius within which to search for neighbors
    ror.setMinNeighborsInRadius(MinNeighborsInRadius);  // Minimum number of neighbors within the radius
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    ror.filter(*filtered_cloud_2);

    // Convert filtered PointCloud back to PointCloud2 message
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud_2, filtered_msg);
    filtered_msg.header = msg->header;

    // Publish the processed PointCloud2 message
    processed_pub_.publish(filtered_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher processed_pub_;

  int MeanK;
  double StddevMulThresh;
  double RadiusSearch;
  int MinNeighborsInRadius;
  string sub_topic;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_processing_node");
  ros::NodeHandle nh;

  PointCloudProcessor processor(nh);

  ROS_INFO("PCL Filtering Node Started.");

  ros::spin();

  return 0;
}