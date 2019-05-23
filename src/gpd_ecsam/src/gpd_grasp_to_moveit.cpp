#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gpd/GraspConfigList.h"
#include "gpd_ecsam/DetectedGraspList.h"

#define _PUBLISH_POSE_AT_FINGER_TIP // publishes palm pose instead if this identifier is not defined
#define _PUBLISH_BOTH_GRIPPER_ORIENTATIONS // there are two solutions due to gripper symmetry

#define MINIMUM_ALLOWED_GRASP_AXIS_Z 0.8
#define MAXIMUM_ALLOWED_GRASP_APPROACH_Z 0

class GpdGraspToMoveitClass
{
 public:
  GpdGraspToMoveitClass();

 private:
  void clusteredGraspsCallback(const gpd::GraspConfigList::ConstPtr &grasp_config_list);

  ros::NodeHandle n_;
  ros::Subscriber sub_clustered_grasps_;
  ros::Publisher pub_detected_grasp_list_;
};

GpdGraspToMoveitClass::GpdGraspToMoveitClass()
{
  sub_clustered_grasps_ = n_.subscribe<gpd::GraspConfigList>("detect_grasps/clustered_grasps", 1,
                                                             &GpdGraspToMoveitClass::clusteredGraspsCallback, this);
  pub_detected_grasp_list_ = n_.advertise<gpd_ecsam::DetectedGraspList>("gpd_ecsam/grasp_poses", 1);
  ros::spin();
}

void GpdGraspToMoveitClass::clusteredGraspsCallback(const gpd::GraspConfigList::ConstPtr &grasp_config_list)
{
  gpd_ecsam::DetectedGraspList detected_grasp_list_msg;
  detected_grasp_list_msg.header = grasp_config_list->header;

  for (const auto &grasp : grasp_config_list->grasps)
  {

    if (sqrtf(powf(grasp.axis.x, 2) + powf(grasp.axis.y, 2)) > MINIMUM_ALLOWED_GRASP_AXIS_Z * fabs(grasp.axis.z))
    {
      continue;
    }

    if (grasp.approach.z > MAXIMUM_ALLOWED_GRASP_APPROACH_Z)
    {
      continue;
    }

    gpd_ecsam::DetectedGrasp detected_grasp_msg;

#ifdef _PUBLISH_POSE_AT_FINGER_TIP
    detected_grasp_msg.pose.position = grasp.top;
#else
    detected_grasp_msg.pose.position = grasp.bottom;
#endif

    detected_grasp_msg.approach = grasp.approach;
    detected_grasp_msg.grasp_width = grasp.width;

    tf2::Matrix3x3 orientation_mat1(grasp.binormal.x, grasp.axis.x, grasp.approach.x,
                                    grasp.binormal.y, grasp.axis.y, grasp.approach.y,
                                    grasp.binormal.z, grasp.axis.z, grasp.approach.z);
    tf2::Quaternion orientation_quat_1;
    orientation_mat1.getRotation(orientation_quat_1);

    detected_grasp_msg.pose.orientation.x = orientation_quat_1.x();
    detected_grasp_msg.pose.orientation.y = orientation_quat_1.y();
    detected_grasp_msg.pose.orientation.z = orientation_quat_1.z();
    detected_grasp_msg.pose.orientation.w = orientation_quat_1.w();
    detected_grasp_list_msg.detected_grasps.push_back(detected_grasp_msg);

#ifdef _PUBLISH_BOTH_GRIPPER_ORIENTATIONS
    tf2::Quaternion orientation_quat_2;
    tf2::Matrix3x3 orientation_mat2(-grasp.binormal.x, -grasp.axis.x, grasp.approach.x,
                                    -grasp.binormal.y, -grasp.axis.y, grasp.approach.y,
                                    -grasp.binormal.z, -grasp.axis.z, grasp.approach.z);
    orientation_mat2.getRotation(orientation_quat_2);
    detected_grasp_msg.pose.orientation.x = orientation_quat_2.x();
    detected_grasp_msg.pose.orientation.y = orientation_quat_2.y();
    detected_grasp_msg.pose.orientation.z = orientation_quat_2.z();
    detected_grasp_msg.pose.orientation.w = orientation_quat_2.w();
    detected_grasp_list_msg.detected_grasps.push_back(detected_grasp_msg);
#endif
  }

  pub_detected_grasp_list_.publish(detected_grasp_list_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpd_grasp_to_moveit");
  GpdGraspToMoveitClass gpdGraspToMoveit;
  return 0;
}
