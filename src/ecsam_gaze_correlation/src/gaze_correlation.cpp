#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "et7/GazeDirection.h"
#include "aruco_mapping_ecsam/ArucoMarker.h"
#include "ecsam_definitions.h"

#define VISUALISE

#define ET7_SCENE_CAMERA_FRAME  "et7_scene_camera_frame"
#define INPUT_POINT_CLOUD_FRAME "root"


#ifdef VISUALISE

#include <visualization_msgs/Marker.h>

#define VISUALISATION_MARKER_SCALE    0.025
#define VISUALISATION_MARKER_COLOUR_R 1
#define VISUALISATION_MARKER_COLOUR_G 0.75
#define VISUALISATION_MARKER_COLOUR_B 0
#endif

class GazeCorrelationClass
{
 public:
  GazeCorrelationClass();

 private:
  // Variables
  ros::NodeHandle n_;
  ros::Subscriber sub_gaze_direction_, sub_input_point_cloud_, sub_ecsam_user_command_;
  ros::Publisher pub_correlated_grasp_point_, pub_correlated_placement_position_, pub_ecsam_status_;

  geometry_msgs::PointStamped gaze_origin_et7_, gaze_direction_point_et7_,
      gaze_origin_d435i_, gaze_direction_point_d435i_;
  boost::mutex mutex_gaze_and_point_cloud_, mutex_user_command_;

  sensor_msgs::PointCloud2 point_cloud_;
  et7::GazeDirection gaze_direction_;

  bool gaze_direction_received_, point_cloud_received_;

  std::string command_;

  // Methods
  void gazeDirectionCallback(const et7::GazeDirectionConstPtr &gaze_direction);

  void inputPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud);

  void userCommandCallback(const std_msgs::StringConstPtr &command);

  void publishCorrelatedCloudPoint();

#ifdef VISUALISE
  ros::Publisher pub_marker_;
  visualization_msgs::Marker marker_;

  void gazeCorrelationMarkerSetup();

  void visualiseGazeCorrelation(const geometry_msgs::Point &point);

#endif
};

GazeCorrelationClass::GazeCorrelationClass()
{
  sub_input_point_cloud_ = n_.subscribe<sensor_msgs::PointCloud2>("ecsam/correlation_point_cloud", 1,
                                                                  &GazeCorrelationClass::inputPointCloudCallback,
                                                                  this);
  sub_gaze_direction_ = n_.subscribe<et7::GazeDirection>("et7/gaze_direction", 1,
                                                         &GazeCorrelationClass::gazeDirectionCallback, this);
  sub_ecsam_user_command_ = n_.subscribe<std_msgs::String>(std::string(ECSAM_TOPIC_COMMAND), 1,
                                                           &GazeCorrelationClass::userCommandCallback, this);

  pub_correlated_grasp_point_ = n_.advertise<geometry_msgs::Point>("ecsam/correlated_cloud_point", 1);
  pub_correlated_placement_position_ = n_.advertise<geometry_msgs::PointStamped>("ecsam/placement_position", 1);
  pub_ecsam_status_ = n_.advertise<std_msgs::String>(std::string(ECSAM_TOPIC_STATUS), 10);

  gaze_origin_et7_.header.frame_id = std::string(ET7_SCENE_CAMERA_FRAME);
  gaze_direction_point_et7_.header.frame_id = std::string(ET7_SCENE_CAMERA_FRAME);
  gaze_origin_d435i_.header.frame_id = std::string(INPUT_POINT_CLOUD_FRAME);
  gaze_direction_point_d435i_.header.frame_id = std::string(INPUT_POINT_CLOUD_FRAME);
  gaze_origin_et7_.point.x = gaze_origin_et7_.point.y = gaze_origin_et7_.point.z = 0.0;
  gaze_direction_point_et7_.point.z = 1.0;

  gaze_direction_received_ = point_cloud_received_ = false;
  command_ = "";

#ifdef VISUALISE
  gazeCorrelationMarkerSetup();
  pub_marker_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
#endif

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}

void GazeCorrelationClass::publishCorrelatedCloudPoint()
{
  boost::mutex::scoped_lock lock(mutex_user_command_); // To avoid user rewriting command mid-execution
  gaze_direction_received_ = point_cloud_received_ = false;

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf_listener(tf2_buffer);
  geometry_msgs::TransformStamped transform_stamped;

  try
  {
    transform_stamped = tf2_buffer.lookupTransform(point_cloud_.header.frame_id,
                                                   std::string(ET7_SCENE_CAMERA_FRAME), ros::Time(0),
                                                   ros::Duration(5.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("gaze_correlation - %s", ex.what());
    return;
  }

  gaze_direction_point_et7_.point.x = gaze_direction_.x;
  gaze_direction_point_et7_.point.y = gaze_direction_.y;

  tf2::doTransform(gaze_origin_et7_, gaze_origin_d435i_, transform_stamped);
  tf2::doTransform(gaze_direction_point_et7_, gaze_direction_point_d435i_, transform_stamped);

  tf2::Vector3 gaze_origin_d435i(gaze_origin_d435i_.point.x, gaze_origin_d435i_.point.y, gaze_origin_d435i_.point.z);
  tf2::Vector3 gaze_direction_unit_vector_d435i = (
      tf2::Vector3(gaze_direction_point_d435i_.point.x, gaze_direction_point_d435i_.point.y,
                   gaze_direction_point_d435i_.point.z) - gaze_origin_d435i).normalize();

  geometry_msgs::Point closest_correlated_cloud_point_msg;
  float closest_distance_to_correlated_cloud_point = 256; // Something high, it does not really matter
  bool is_gaze_correlated = false;
  {
    float correlation_line_to_point_distance = 0;
    if (command_ == ECSAM_COMMAND_PICK)
    {
      correlation_line_to_point_distance = PICK_CORRELATION_LINE_POINT_DISTANCE_THRESHOLD;
    }
    else if (command_ == ECSAM_COMMAND_PLACE)
    {
      correlation_line_to_point_distance = PLACE_CORRELATION_LINE_POINT_DISTANCE_THRESHOLD;
    }
    sensor_msgs::PointCloud2Iterator<float> point_x(point_cloud_, "x"), point_y(point_cloud_, "y"), point_z(
        point_cloud_, "z");
    for (; point_x != point_x.end(); ++point_x, ++point_y, ++point_z)
    {
      tf2::Vector3 cloud_point(*point_x, *point_y, *point_z);
      tf2::Vector3 gaze_origin_to_cloud_point = cloud_point - gaze_origin_d435i;
      tf2::Vector3 cloud_point_projected_to_gaze_direction = gaze_origin_d435i +
                                                             gaze_direction_unit_vector_d435i *
                                                             gaze_origin_to_cloud_point.dot(
                                                                 gaze_direction_unit_vector_d435i);

      if (cloud_point_projected_to_gaze_direction.distance(cloud_point) < correlation_line_to_point_distance)
      {
        float gaze_origin_to_projected_cloud_point_distance = gaze_origin_d435i.distance(
            cloud_point_projected_to_gaze_direction);
        if (gaze_origin_to_projected_cloud_point_distance < closest_distance_to_correlated_cloud_point)
        {
          closest_correlated_cloud_point_msg.x = *point_x;
          closest_correlated_cloud_point_msg.y = *point_y;
          closest_correlated_cloud_point_msg.z = *point_z;
          closest_distance_to_correlated_cloud_point = gaze_origin_to_projected_cloud_point_distance;
          is_gaze_correlated = true;
        }
      }
    }
  }

  if (is_gaze_correlated)
  {
    if (command_ == ECSAM_COMMAND_PICK)
    {
      pub_correlated_grasp_point_.publish(closest_correlated_cloud_point_msg);
    }
    else if (command_ == ECSAM_COMMAND_PLACE)
    {
      geometry_msgs::PointStamped closest_correlated_cloud_point_msg_stamped;
      closest_correlated_cloud_point_msg_stamped.header.frame_id = point_cloud_.header.frame_id;
      closest_correlated_cloud_point_msg_stamped.point = closest_correlated_cloud_point_msg;
      pub_correlated_placement_position_.publish(closest_correlated_cloud_point_msg_stamped);
    }
#ifdef VISUALISE
    visualiseGazeCorrelation(closest_correlated_cloud_point_msg);
#endif

#ifdef GAZE_CORRELATION_TABLE_FILE_PATH
    std::ofstream output_file_stream_gaze_correlation;
    output_file_stream_gaze_correlation.open(GAZE_CORRELATION_TABLE_FILE_PATH, std::ofstream::out | std::ofstream::app);

    output_file_stream_gaze_correlation << "\n" << closest_correlated_cloud_point_msg.x << " " << closest_correlated_cloud_point_msg.y << " " << closest_correlated_cloud_point_msg.z;

    output_file_stream_gaze_correlation.close();
#endif

#ifdef EXECUTION_TIME_FILE_PATH
    std::ofstream output_file_stream;
    output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

    output_file_stream << "\n" << ros::Time::now() << " Gaze correlated";

    output_file_stream.close();
#endif
  }
  else
  {
    std_msgs::String status_msg;
    status_msg.data = ECSAM_ERROR_GAZE_NOT_CORRELATED;
    pub_ecsam_status_.publish(status_msg);
  }
}

void GazeCorrelationClass::gazeDirectionCallback(const et7::GazeDirectionConstPtr &gaze_direction)
{
  gaze_direction_ = *gaze_direction;

  boost::mutex::scoped_lock lock(mutex_gaze_and_point_cloud_); //To avoid race-condition
  if (point_cloud_received_)
  {
    publishCorrelatedCloudPoint();
    return;
  }
  gaze_direction_received_ = true;
}

void GazeCorrelationClass::inputPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud)
{
  point_cloud_ = *point_cloud;

  boost::mutex::scoped_lock lock(mutex_gaze_and_point_cloud_); //To avoid race-condition
  if (gaze_direction_received_)
  {
    publishCorrelatedCloudPoint();
    return;
  }
  point_cloud_received_ = true;
}

void GazeCorrelationClass::userCommandCallback(const std_msgs::StringConstPtr &command)
{
  boost::mutex::scoped_lock lock(mutex_user_command_);
  command_ = command->data;
}

#ifdef VISUALISE

void GazeCorrelationClass::gazeCorrelationMarkerSetup()
{
  marker_.header.frame_id = std::string(INPUT_POINT_CLOUD_FRAME);
  marker_.lifetime = ros::Duration(5);
  marker_.id = 0;
  marker_.ns = "correlated_gaze";
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.action = visualization_msgs::Marker::ADD;

  marker_.scale.x = marker_.scale.y = marker_.scale.z = VISUALISATION_MARKER_SCALE;

  marker_.color.r = VISUALISATION_MARKER_COLOUR_R;
  marker_.color.g = VISUALISATION_MARKER_COLOUR_G;
  marker_.color.b = VISUALISATION_MARKER_COLOUR_B;
  marker_.color.a = 1.0;
}

void GazeCorrelationClass::visualiseGazeCorrelation(const geometry_msgs::Point &point)
{
  marker_.header.stamp = ros::Time::now();
  marker_.pose.position = point;
  pub_marker_.publish(marker_);
}

#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gaze_correlation");
  GazeCorrelationClass gaze_correlation;
  return (0);
}
