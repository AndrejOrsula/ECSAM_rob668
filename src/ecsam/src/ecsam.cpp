#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include "j2n6s200_moveit.h"
#include "ecsam_definitions.h"

class EcsamClass
{
 public:
  explicit EcsamClass(ros::NodeHandle &node_handle);

 private:
  void detectedGraspListCallback(const gpd_ecsam::DetectedGraspListConstPtr &detected_grasp_list);

  void objectInfoCallback(const std_msgs::Float32MultiArrayConstPtr &object_info);

  void placementPositionCallback(const geometry_msgs::PointStampedConstPtr &position);

  ros::NodeHandle n_;
  ros::Subscriber sub_detected_grasp_list_, sub_object_info_, sub_placement_position_;
  ros::Publisher pub_ecsam_status_;

  float object_height_, object_radius_;
  geometry_msgs::Point object_position_;

  j2n6s200MoveitClass jaco;
};

EcsamClass::EcsamClass(ros::NodeHandle &node_handle)
    : n_(node_handle), jaco(node_handle)
{
  sub_detected_grasp_list_ = n_.subscribe<gpd_ecsam::DetectedGraspList>("gpd_ecsam/grasp_poses", 1,
                                                                        &EcsamClass::detectedGraspListCallback, this);
  sub_object_info_ = n_.subscribe<std_msgs::Float32MultiArray>("ecsam/object_info", 1,
                                                               &EcsamClass::objectInfoCallback, this);
  sub_placement_position_ = n_.subscribe<geometry_msgs::PointStamped>("ecsam/placement_position", 1,
                                                                      &EcsamClass::placementPositionCallback, this);

  pub_ecsam_status_ = n_.advertise<std_msgs::String>(std::string(ECSAM_TOPIC_STATUS), 10);

  object_height_ = object_radius_ = object_position_.z = 0;
}

void EcsamClass::detectedGraspListCallback(const gpd_ecsam::DetectedGraspListConstPtr &detected_grasp_list)
{
  std_msgs::String status_msg;
  if (detected_grasp_list->detected_grasps.empty())
  {
    status_msg.data = ECSAM_ERROR_NO_GRASP_DETECTED;
    pub_ecsam_status_.publish(status_msg);
    return;
  }

#ifdef EXECUTION_TIME_FILE_PATH
  std::ofstream output_file_stream;
  output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

  output_file_stream << "\n" << ros::Time::now() << " Grasps generated";

  output_file_stream.close();
#endif

  if (!jaco.pick(*detected_grasp_list, object_position_, object_height_, object_radius_))
  {
    status_msg.data = ECSAM_ERROR_PICKING_FAILED;
    pub_ecsam_status_.publish(status_msg);
    return;
  }

  status_msg.data = ECSAM_INFO_PICKING_SUCCESSFUL;
  pub_ecsam_status_.publish(status_msg);

}

void EcsamClass::objectInfoCallback(const std_msgs::Float32MultiArrayConstPtr &object_info)
{
  object_radius_ = object_info->data.at(0);
  object_height_ = object_info->data.at(1);
  object_position_.x = object_info->data.at(2);
  object_position_.y = object_info->data.at(3);
}

void EcsamClass::placementPositionCallback(const geometry_msgs::PointStampedConstPtr &position)
{
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf_listener(tf2_buffer);
  geometry_msgs::TransformStamped transform_stamped;

  try
  {
    transform_stamped = tf2_buffer.lookupTransform(ROOT_FRAME, position->header.frame_id, ros::Time(0),
                                                   ros::Duration(5.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("placing coordinates - %s", ex.what());
    return;
  }

  geometry_msgs::PointStamped transformed_position;
  transformed_position.header.frame_id = ROOT_FRAME;

  tf2::doTransform(*position, transformed_position, transform_stamped);

  transformed_position.point.x += PLACE_CORRELATION_LINE_POINT_DISTANCE_THRESHOLD / 2;

  std_msgs::String status_msg;
  if (!jaco.place(transformed_position.point))
  {
    status_msg.data = ECSAM_ERROR_PLACING_FAILED;
    pub_ecsam_status_.publish(status_msg);
    return;
  }

  status_msg.data = ECSAM_INFO_PLACING_SUCCESSFUL;
  pub_ecsam_status_.publish(status_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ecsam");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  EcsamClass ecsam(n);

  ros::waitForShutdown();
  return 0;
}
