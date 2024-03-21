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
    nh_.getParam("point_cloud_processing_node/LeafSize", LeafSize);
    nh_.getParam("point_cloud_processing_node/MeanK", MeanK);
    nh_.getParam("point_cloud_processing_node/StddevMulThresh", StddevMulThresh);
    nh_.getParam("point_cloud_processing_node/RadiusSearch", RadiusSearch);
    nh_.getParam("point_cloud_processing_node/MinNeighborsInRadius", MinNeighborsInRadius);

    // Create a subscriber for the PointCloud2 message
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 1, &PointCloudProcessor::pointCloudCallback, this);

    // Create a publisher for the processed PointCloud2 message
    processed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1);

    // Initialize filters
    voxel_.setLeafSize(LeafSize, LeafSize, LeafSize);
    sor_.setMeanK(MeanK);
    sor_.setStddevMulThresh(StddevMulThresh);
    ror_.setRadiusSearch(RadiusSearch);
    ror_.setMinNeighborsInRadius(MinNeighborsInRadius);

    // Initialize the point cloud object
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }


  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Convert PointCloud2 message to PCL PointCloud
    pcl::fromROSMsg(*msg, *cloud_);
    pcl::removeNaNFromPointCloud(*cloud_, *cloud_, indices_);

    // Perform voxel grid downsampling
    voxel_.setInputCloud(cloud_);
    voxel_.filter(*cloud_);

    // Perform statistical outlier removal
    sor_.setInputCloud(cloud_);
    sor_.filter(*cloud_);

    // Perform radius outlier removal
    ror_.setInputCloud(cloud_);
    ror_.filter(*cloud_);

    // Convert filtered PointCloud back to PointCloud2 message
    pcl::toROSMsg(*cloud_, filtered_msg_);
    filtered_msg_.header = msg->header;

    // Publish the processed PointCloud2 message
    processed_pub_.publish(filtered_msg_);
  }


private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher processed_pub_;

  float LeafSize;
  int MeanK;
  double StddevMulThresh;
  double RadiusSearch;
  int MinNeighborsInRadius;
  string sub_topic;

  std::vector<int> indices_;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  sensor_msgs::PointCloud2 filtered_msg_;
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