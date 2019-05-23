#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include "aruco_mapping_ecsam/ArucoMarker.h"

#define NUMBER_OF_MARKERS 9

#define JACO_X_OFFSET  0.1013
#define JACO_Y_OFFSET  0.0772
#define TABLE_Z_OFFSET (-0.044)

class RootMarkerStaticTfBroadcasterClass
{
 public:
  RootMarkerStaticTfBroadcasterClass();

 private:
  // Variables
  ros::NodeHandle n_;
  ros::Subscriber sub_aruco_poses_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  geometry_msgs::TransformStamped static_transformStamped_;

  struct marker_pose
  {
    uint16_t id{};
    geometry_msgs::Pose pose{};
  } marker_poses_[NUMBER_OF_MARKERS];

  // Methods
  void defineMarkers();

  void arucoPosesCallback(const aruco_mapping_ecsam::ArucoMarkerConstPtr &aruco_poses);
};

RootMarkerStaticTfBroadcasterClass::RootMarkerStaticTfBroadcasterClass()
{
  ros::NodeHandle n_params("~");
  std::string root_frame_, markers_frame_;
  n_params.param<std::string>("root_frame", root_frame_, "root");
  n_params.param<std::string>("marker_origin_frame", markers_frame_, "et7_scene_camera_frame");

  static_transformStamped_.header.frame_id = root_frame_;
  static_transformStamped_.child_frame_id = markers_frame_;

  defineMarkers();

  sub_aruco_poses_ = n_.subscribe<aruco_mapping_ecsam::ArucoMarker>("aruco_poses", 1,
                                                                    &RootMarkerStaticTfBroadcasterClass::arucoPosesCallback,
                                                                    this);

  ros::spin();
}

void
RootMarkerStaticTfBroadcasterClass::arucoPosesCallback(const aruco_mapping_ecsam::ArucoMarkerConstPtr &aruco_poses)
{
  if (aruco_poses->num_of_visible_markers == 0)
  {
    return;
  }

  // find ID of the first marker (located at origin)
  int origin_marker_id = -1;
  for (int i = 0; i < aruco_poses->num_of_visible_markers; ++i)
  {
    if (aruco_poses->global_marker_poses[i].position.x == 0 &&
        aruco_poses->global_marker_poses[i].position.y == 0 &&
        aruco_poses->global_marker_poses[i].position.z == 0)
    {
      origin_marker_id = aruco_poses->marker_ids[i];
      break;
    }
  }
  if (origin_marker_id == -1)
  {
    return;
  }

  //find index of the marker ID and assign its pose as transformation
  for (auto &marker_pose : marker_poses_)
  {
    if (marker_pose.id == origin_marker_id)
    {
      static_transformStamped_.transform.translation.x = marker_pose.pose.position.x;
      static_transformStamped_.transform.translation.y = marker_pose.pose.position.y;
      static_transformStamped_.transform.translation.z = marker_pose.pose.position.z;
      static_transformStamped_.transform.rotation = marker_pose.pose.orientation;

      static_transformStamped_.header.stamp = ros::Time::now();
      static_broadcaster_.sendTransform(static_transformStamped_);

      sub_aruco_poses_.shutdown();
      return;
    }
  }
}

void
RootMarkerStaticTfBroadcasterClass::defineMarkers()
{
  marker_poses_[0].id = 273; // E
  marker_poses_[0].pose.position.x = JACO_X_OFFSET + 0.429;
  marker_poses_[0].pose.position.y = JACO_Y_OFFSET - 0.004;
  marker_poses_[0].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[0].pose.orientation.x = 0;
  marker_poses_[0].pose.orientation.y = 0;
  marker_poses_[0].pose.orientation.z = 0;
  marker_poses_[0].pose.orientation.w = 1;

  marker_poses_[1].id = 771; // C
  marker_poses_[1].pose.position.x = JACO_X_OFFSET + 0.429;
  marker_poses_[1].pose.position.y = JACO_Y_OFFSET - 0.164;
  marker_poses_[1].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[1].pose.orientation.x = 0;
  marker_poses_[1].pose.orientation.y = 0;
  marker_poses_[1].pose.orientation.z = 0;
  marker_poses_[1].pose.orientation.w = 1;

  marker_poses_[2].id = 473; // S
  marker_poses_[2].pose.position.x = JACO_X_OFFSET + 0.429;
  marker_poses_[2].pose.position.y = JACO_Y_OFFSET - 0.324;
  marker_poses_[2].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[2].pose.orientation.x = 0;
  marker_poses_[2].pose.orientation.y = 0;
  marker_poses_[2].pose.orientation.z = 0;
  marker_poses_[2].pose.orientation.w = 1;

  marker_poses_[3].id = 442; // A
  marker_poses_[3].pose.position.x = JACO_X_OFFSET + 0.429;
  marker_poses_[3].pose.position.y = JACO_Y_OFFSET - 0.484;
  marker_poses_[3].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[3].pose.orientation.x = 0;
  marker_poses_[3].pose.orientation.y = 0;
  marker_poses_[3].pose.orientation.z = 0;
  marker_poses_[3].pose.orientation.w = 1;

  marker_poses_[4].id = 819; // M
  marker_poses_[4].pose.position.x = JACO_X_OFFSET + 0.429;
  marker_poses_[4].pose.position.y = JACO_Y_OFFSET - 0.644;
  marker_poses_[4].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[4].pose.orientation.x = 0;
  marker_poses_[4].pose.orientation.y = 0;
  marker_poses_[4].pose.orientation.z = -0.707;
  marker_poses_[4].pose.orientation.w = 0.707;

  marker_poses_[5].id = 341;
  marker_poses_[5].pose.position.x = JACO_X_OFFSET + 0.083;
  marker_poses_[5].pose.position.y = JACO_Y_OFFSET - 0.083;
  marker_poses_[5].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[5].pose.orientation.x = 0;
  marker_poses_[5].pose.orientation.y = 0;
  marker_poses_[5].pose.orientation.z = 0;
  marker_poses_[5].pose.orientation.w = 1;

  marker_poses_[6].id = 682;
  marker_poses_[6].pose.position.x = JACO_X_OFFSET + 0.083;
  marker_poses_[6].pose.position.y = JACO_Y_OFFSET - 0.243666666;
  marker_poses_[6].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[6].pose.orientation.x = 0;
  marker_poses_[6].pose.orientation.y = 0;
  marker_poses_[6].pose.orientation.z = 0;
  marker_poses_[6].pose.orientation.w = 1;

  marker_poses_[7].id = 789;
  marker_poses_[7].pose.position.x = JACO_X_OFFSET + 0.083;
  marker_poses_[7].pose.position.y = JACO_Y_OFFSET - 0.404333333;
  marker_poses_[7].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[7].pose.orientation.x = 0;
  marker_poses_[7].pose.orientation.y = 0;
  marker_poses_[7].pose.orientation.z = 0;
  marker_poses_[7].pose.orientation.w = 1;

  marker_poses_[8].id = 855;
  marker_poses_[8].pose.position.x = JACO_X_OFFSET + 0.083;
  marker_poses_[8].pose.position.y = JACO_Y_OFFSET - 0.565;
  marker_poses_[8].pose.position.z = TABLE_Z_OFFSET;
  marker_poses_[8].pose.orientation.x = 0;
  marker_poses_[8].pose.orientation.y = 0;
  marker_poses_[8].pose.orientation.z = 0;
  marker_poses_[8].pose.orientation.w = 1;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "root_marker_static_tf_broadcaster");
  RootMarkerStaticTfBroadcasterClass root_marker_static_tf_broadcaster;
  return (0);
}
