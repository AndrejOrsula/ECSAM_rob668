#include <cmath>
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <octomap/OcTreeNode.h>
#include <octomap_msgs/Octomap.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "ecsam_definitions.h"

//Plane detection
#define Iterations                  500
#define ThresholdDistance           0.015 //1.5cm

//Clustering
#define ClusterTolerance            0.02 //2cm
#define MinClusterSize              50
#define MaxClusterSize              25000


//#define PUBLISH_CONTINUOSLY


class segmentationClass
{

 public:
  segmentationClass();

  ~segmentationClass();

 private:

  ros::NodeHandle nh;

  // ROS subscribers for the input point cloud
  ros::Subscriber pointCloudsSub;
  ros::Subscriber commandSub;
  ros::Subscriber gazeCorrelationSub;
  ros::Subscriber sub_ecsam_status_;
  // ROS publishers for the output point clouds
  ros::Publisher pub;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub5;

#ifdef PUBLISH_CONTINUOSLY
  ros::Publisher table_pub;
  ros::Publisher pub_x[20];
#endif

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud);

  void commandFunction(const std_msgs::StringConstPtr &command);

  void gazeFunction(const geometry_msgs::Point &point);

  void statusCallback(const std_msgs::StringConstPtr &status);

  float xGaze = 0, yGaze = 0, zGaze = 0;
  float z = 0, lowOldZ = 0, highOldZ = 0, height = 0, oldRadius = 0;
  bool found_cluster = false;
  bool publishObject = false;
  bool publishPlane = false;
  bool publishAllObjects = false;

  bool picking_object_ = false, placing_object_ = false;
};

void segmentationClass::commandFunction(const std_msgs::StringConstPtr &command)
{

  if (command->data == ECSAM_COMMAND_PICK)
  {
    publishAllObjects = true;
  }

  if (command->data == ECSAM_COMMAND_PLACE)
  {
    publishPlane = true;
  }

  picking_object_ = placing_object_ = false;
}

void segmentationClass::gazeFunction(const geometry_msgs::Point &point)
{

  xGaze = point.x;
  yGaze = point.y;
  zGaze = point.z;
  publishObject = true;

  picking_object_ = false;
}

void segmentationClass::statusCallback(const std_msgs::StringConstPtr &status)
{
  picking_object_ = placing_object_ = publishObject = publishPlane = publishAllObjects = false;
}

void segmentationClass::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////VARIABLES///////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef PUBLISH_CONTINUOSLY
  publishAllObjects = publishPlane = true;
  publishObject = picking_object_ = placing_object_ = false;
#endif

  if (publishAllObjects || publishPlane || publishObject || picking_object_ || placing_object_)
  {

    if (placing_object_)
    {
      pub2.publish(*cloud);
      return;
    }

    //PCL pointclouds - move to class if possible

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chosen_cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);

    //Sensor message pointclouds - move to class if possible

    sensor_msgs::PointCloud2::Ptr output = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr output2 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr output3 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr output4 = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

    //Downsampling and table plane detection declarations

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZ>);


    //Clustering declarations

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    // Removal of points

    pcl::PassThrough<pcl::PointXYZ> pass;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////FUNCTION STATEMENTS//////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //Converts from ROS to PCL
    pcl::fromROSMsg(*cloud, *cloud_filtered);

    // Create the filtering object: downsample the dataset using a leaf size of 'x' cm
    *cloud_filtered2 = *cloud_filtered;

    // Create the segmentation object for the planar model and set all the parameters

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(Iterations);
    seg.setDistanceThreshold(ThresholdDistance);

    int nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.empty())
      {
        break;
      }

      // Extract the planar inliers from the input cloud
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Get the points associated with the planar surface
      extract.filter(*cloud_plane);
      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_f);
      *cloud_filtered = *cloud_f;
    }

    tree->setInputCloud(cloud_filtered);

    ec.setClusterTolerance(ClusterTolerance);
    ec.setMinClusterSize(MinClusterSize);
    ec.setMaxClusterSize(MaxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

#ifdef PUBLISH_CONTINUOSLY
    int cluster_counter = 0;
#endif
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it)
    {
      cloud_cluster->points.clear();
      for (int indice : it->indices)
      {
        cloud_cluster->points.push_back(cloud_filtered->points[indice]);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      *merged_cloud += *cloud_cluster;

#ifdef PUBLISH_CONTINUOSLY
      sensor_msgs::PointCloud2 xsfa;
      pcl::toROSMsg(*cloud_cluster, xsfa);
      xsfa.header.frame_id = "root";
      pub_x[cluster_counter].publish(xsfa);
      cluster_counter++;
#endif

      if (!found_cluster)
      {
        for (auto &point : cloud_cluster->points)  //Finds a matching cluster based on the gaze correlation
        {
          float distance = sqrt(pow(point.x - xGaze, 2) + pow(point.y - yGaze, 2) + pow(point.z - zGaze, 2));
          if (distance < 0.025)
          {
            *chosen_cluster = *cloud_cluster;
            found_cluster = true;

            break;
          }
        }
      }
    }

#ifdef PUBLISH_CONTINUOSLY
    for (int j = cluster_counter +1; j < 20; ++j) {
        sensor_msgs::PointCloud2 xsfa;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2 = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>);

temp_cloud2->points.resize(1);
temp_cloud2->width = 1;
        temp_cloud2->height = 1;
        temp_cloud2->points[0].x = 0;
        temp_cloud2->points[0].y = 5000;
        temp_cloud2->points[0].z = -100;


        pcl::toROSMsg(*temp_cloud2, xsfa);
        xsfa.header.frame_id = "root";
        pub_x[j].publish(xsfa);
    }
#endif

    if (found_cluster)
    {
      if (publishObject) // publish object info and the chosen segmented object
      {
        float position_x, position_y;
        float boundaries_x[2] = {-100, 100}, boundaries_y[2] = {-100, 100};
        for (auto &point : chosen_cluster->points)
        {
          if (point.z > highOldZ)
          {
            highOldZ = point.z;
          }
          else if (point.z < lowOldZ)
          {
            lowOldZ = point.z;
          }

          if (point.x > boundaries_x[0])
          {
            boundaries_x[0] = point.x;
          }
          else if (point.x < boundaries_x[1])
          {
            boundaries_x[1] = point.x;
          }

          if (point.y > boundaries_y[0])
          {
            boundaries_y[0] = point.y;
          }
          else if (point.y < boundaries_y[1])
          {
            boundaries_y[1] = point.y;
          }

        }
        height = highOldZ - lowOldZ + ThresholdDistance;
        position_x = (boundaries_x[0] + boundaries_x[1]) / 2;
        position_y = (boundaries_y[0] + boundaries_y[1]) / 2;
        oldRadius =
            sqrt(pow(boundaries_x[1] - boundaries_x[0], 2) + pow(boundaries_y[1] - boundaries_y[0], 2)) / 2;

        std_msgs::Float32MultiArray radiusAndHeight;
        radiusAndHeight.data.clear();
        radiusAndHeight.data.push_back(oldRadius);
        radiusAndHeight.data.push_back(height);
        radiusAndHeight.data.push_back(position_x);
        radiusAndHeight.data.push_back(position_y);
        pub5.publish(radiusAndHeight);

        pcl::toROSMsg(*chosen_cluster, *output);
        output->header.frame_id = cloud->header.frame_id;
        pub.publish(*output);

        picking_object_ = true;

#ifdef EXECUTION_TIME_FILE_PATH
        std::ofstream output_file_stream;
        output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

        output_file_stream << "\n" << ros::Time::now() << " Correlated object segmented";

        output_file_stream.close();
#endif

      }

      if (publishObject || picking_object_) // publish octomap point cloud
      {
        for (int k = 0; k < cloud_filtered2->points.size(); ++k)
        {
          for (int i = 0; i < chosen_cluster->points.size(); ++i)
          {
            if (cloud_filtered2->points[k].x == chosen_cluster->points[i].x &&
                cloud_filtered2->points[k].y == chosen_cluster->points[i].y &&
                cloud_filtered2->points[k].z == chosen_cluster->points[i].z)
            {
              cloud_filtered2->points[k].x = 0;
              cloud_filtered2->points[k].y = 0;
              cloud_filtered2->points[k].z = -10;
            }
          }
        }

        pass.setInputCloud(cloud_filtered2);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-10.0, -10.0);
        pass.setFilterLimitsNegative(true);
        pass.filter(*cloud_filtered2);

        pcl::toROSMsg(*cloud_filtered2, *output2);
        output2->header.frame_id = cloud->header.frame_id;
        pub2.publish(*output2);

      }
    }

    if (publishAllObjects) // publish all segmented objects
    {
      pcl::toROSMsg(*merged_cloud, *output4);
      output4->header.frame_id = cloud->header.frame_id;
      pub3.publish(*output4);
#ifdef EXECUTION_TIME_FILE_PATH
      std::ofstream output_file_stream;
      output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

      output_file_stream << "\n" << ros::Time::now() << " All objects segmented";

      output_file_stream.close();
#endif
    }

    if (publishPlane) // publish segmented plane and everything for octomap
    {
      pcl::toROSMsg(*cloud_plane, *output3);
      output3->header.frame_id = cloud->header.frame_id;

#ifdef PUBLISH_CONTINUOSLY
      table_pub.publish(*output3);
      return;
#endif

      pub3.publish(*output3);
      pub2.publish(*cloud);
      placing_object_ = true;

#ifdef EXECUTION_TIME_FILE_PATH
      std::ofstream output_file_stream;
      output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

      output_file_stream << "\n" << ros::Time::now() << " Plane segmented";

      output_file_stream.close();
#endif
    }

    found_cluster = false;
    publishObject = false;
    publishPlane = false;
    publishAllObjects = false;

    oldRadius = 0;
    height = 0;
    z = 0;
    lowOldZ = 0;
    highOldZ = 0;
  }
}

segmentationClass::segmentationClass()
{

  // ROS subscribers for the input point cloud
  pointCloudsSub = nh.subscribe("/d435i/depth/point_cloud", 1, &segmentationClass::cloud_cb, this); //Use with camera
  commandSub = nh.subscribe(std::string(ECSAM_TOPIC_COMMAND), 1, &segmentationClass::commandFunction,
                            this); //Use with camera
  sub_ecsam_status_ = nh.subscribe<std_msgs::String>(std::string(ECSAM_TOPIC_STATUS), 1,
                                                     &segmentationClass::statusCallback, this);
  gazeCorrelationSub = nh.subscribe("/ecsam/correlated_cloud_point", 1, &segmentationClass::gazeFunction,
                                    this); //Use with camera

  // ROS publishers for the output point clouds
  pub = nh.advertise<sensor_msgs::PointCloud2>("/ecsam/correlated_object_point_cloud", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2>("/ecsam/octomap_point_cloud", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2>("/ecsam/correlation_point_cloud", 1);
  pub5 = nh.advertise<std_msgs::Float32MultiArray>("/ecsam/object_info", 1);

#ifdef PUBLISH_CONTINUOSLY
  for (int i = 0; i < 20; ++i) {
      std::stringstream ss;
      ss << i;
      pub_x[i] = nh.advertise<sensor_msgs::PointCloud2>("/segmentation_cluster_" + ss.str(), 1);
  }
  table_pub = nh.advertise<sensor_msgs::PointCloud2>("/ecsam/table_point_cloud", 1);
#endif

  ros::spin();
}

segmentationClass::~segmentationClass() = default;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ecsam_segmentation");
  segmentationClass segmentationclass;
  return 0;
}
