#ifndef j2n6s200_Moveit_Class
#define j2n6s200_Moveit_Class

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_driver/kinova_ros_types.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>

#include "gpd_ecsam/DetectedGraspList.h"
#include "ecsam_definitions.h"




/// Robot specifications
#define ROBOT_TYPE       "j2n6s200"
#define ROOT_FRAME       "root"
#define NUMBER_OF_JOINTS 6
#define FINGER_MAX       6400


/// Path planning parameters
#define NUMBER_OF_PLANNING_ATTEMPTS     1
#define PLANNING_TIME                   2.5
#define GOAL_POSITION_TOLERANCE         0.005
#define GOAL_ORIENTATION_TOLERANCE      0.05

#define CARTESIAN_PLAN_STEP_SIZE        0.0025
#define CARTESIAN_PLAN_JUMP_THRESHOLD   0.0

#define PICK_NUMBER_OF_IK_ATTEMPTS  10
#define PICK_IK_TIMEOUT             0.05

const float JOINT_DISTANCE_WEIGHTS[NUMBER_OF_JOINTS] = {2, 1, 0.75, 0.5, 0.5, 0.5};


/// Home configuration
#define HOME_POSE_JOINT_0 (180 *M_PI/180.0)
#define HOME_POSE_JOINT_1 (150 *M_PI/180.0)
#define HOME_POSE_JOINT_2 (35  *M_PI/180.0)
#define HOME_POSE_JOINT_3 (-90 *M_PI/180.0)
#define HOME_POSE_JOINT_4 (0   *M_PI/180.0)
#define HOME_POSE_JOINT_5 (100 *M_PI/180.0)

#define HOME_POSE_POSITION_X    0.212852
#define HOME_POSE_POSITION_Y    0.00981297
#define HOME_POSE_POSITION_Z    0.354043
#define HOME_POSE_ORIENTATION_X 0.542067
#define HOME_POSE_ORIENTATION_Y 0.646076
#define HOME_POSE_ORIENTATION_Z 0.345415
#define HOME_POSE_ORIENTATION_W 0.411628


/// Static collision objects
#define NUMBER_OF_STATIC_COLLISION_OBJECTS 5

#define TABLE_POSITION_X_OFFSET 0.117
#define TABLE_POSITION_Y_OFFSET 0.095
#define TABLE_POSITION_Z_OFFSET 0.05
#define TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT (TABLE_POSITION_Z_OFFSET - 0.006)
#define TABLE_RIM_SIZE          0.015
#define USER_TABLE_DISTANCE     0.1

#define TABLE_DIMENSION_X 0.53
#define TABLE_DIMENSION_Y 0.68
#define TABLE_DIMENSION_Z 0.77
#define TABLE_POSITION_X  (TABLE_DIMENSION_X/2.0 - TABLE_POSITION_X_OFFSET)
#define TABLE_POSITION_Y  (-TABLE_DIMENSION_Y/2.0 + TABLE_POSITION_Y_OFFSET)
#define TABLE_POSITION_Z  (-TABLE_DIMENSION_Z/2.0 - TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT)

#define TABLE_EXTENDED_DIMENSION_X 0.195
#define TABLE_EXTENDED_DIMENSION_Y (TABLE_DIMENSION_Y + 2*(0.08 - TABLE_RIM_SIZE))
#define TABLE_EXTENDED_DIMENSION_Z 0.77
#define TABLE_EXTENDED_POSITION_X  (TABLE_POSITION_X + TABLE_DIMENSION_X/2.0 + TABLE_EXTENDED_DIMENSION_X/2.0)
#define TABLE_EXTENDED_POSITION_Y  (TABLE_POSITION_Y)
#define TABLE_EXTENDED_POSITION_Z  (-TABLE_EXTENDED_DIMENSION_Z/2.0 - TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT)

#define TABLE_EXTRA_DIMENSION_X 0.5
#define TABLE_EXTRA_DIMENSION_Y 1.25
#define TABLE_EXTRA_DIMENSION_Z 0.77
#define TABLE_EXTRA_POSITION_X  (TABLE_POSITION_X + TABLE_DIMENSION_X/2.0 + TABLE_EXTRA_DIMENSION_X/2.0)
#define TABLE_EXTRA_POSITION_Y  (TABLE_POSITION_Y)
#define TABLE_EXTRA_POSITION_Z  (-TABLE_EXTRA_DIMENSION_Z/2.0 - TABLE_POSITION_Z_OFFSET)

#define CAMERA_DIMENSION_X 0.175
#define CAMERA_DIMENSION_Y 0.125
#define CAMERA_DIMENSION_Z 0.8
#define CAMERA_POSITION_X  (-TABLE_POSITION_X_OFFSET + CAMERA_DIMENSION_X/2.0 + TABLE_RIM_SIZE)
#define CAMERA_POSITION_Y  (-TABLE_DIMENSION_Y + TABLE_POSITION_Y_OFFSET + CAMERA_DIMENSION_Y/2.0 + TABLE_RIM_SIZE)
#define CAMERA_POSITION_Z  (CAMERA_DIMENSION_Z/2.0 - TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT)

#define USER_DIMENSION_X 0.25
#define USER_DIMENSION_Y 0.3
#define USER_DIMENSION_Z 2.0
#define USER_POSITION_X  (-USER_DIMENSION_X/2.0 - USER_TABLE_DISTANCE)
#define USER_POSITION_Y  (-TABLE_DIMENSION_Y/2.0 + TABLE_POSITION_Y_OFFSET)
#define USER_POSITION_Z  (USER_DIMENSION_Z/2.0 - TABLE_DIMENSION_Z - TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT)


/// Pick and place
#define TCP_TO_FINGERTIP_DISTANCE 0.05
#define WRIST_TABLE_MINIMAL_DISTANCE 0.025

#define PICK_APPROACH_DISTANCE 0.15

#define MAXIMUM_NUMBER_OF_PICK_ATTEMPTS 5

#define PICK_PINCHING_THRESHOLD                    0.0
#define PICKING_PINCHING_GRASP_FINGER_DISTANCE     0.06
#define PICKING_CYLINDRICAL_GRASP_FINGER_DISTANCE  0.03

#define POST_PICK_DEFAULT_POSITION_X 0.4
#define POST_PICK_DEFAULT_POSITION_Y 0
#define POST_PICK_DEFAULT_POSITION_Z 0.35
#define DEFAULT_POSE_POSITION_TOLERANCE_INCREASE_STEP_MULTIPLIER 5
#define DEFAULT_POSE_ORIENTATION_TOLERANCE_INCREASE_STEP_MULTIPLIER 2.5

#define GRASPED_OBJECT_ID               "object_of_interest"
#define GRIPPER_COMPACT_CLOSING_FACTOR  0.9
#define GRIPPER_GRASPING_CLOSING_FACTOR 1.0
#define SUCCESSFUL_GRASP_PERCENTAGE     0.001
#define PICKED_OBJECT_CREATION_OFFSET_FROM_TABLE 0.005
#define PICKED_OBJECT_CREATION_HEIGHT_SCALING_FACTOR 1.0
#define PICKED_OBJECT_CREATION_RADIUS_SCALING_FACTOR 1.0
#define MAX_OBJECT_RADIUS 0.075

#define POST_PICK_PRE_PLACE_ASCENSION_DISTANCE 0.1
#define PLACING_SAFETY_HEIGHT 0.006
#define POST_PLACING_SAFETY_HEIGHT 0.075


/// Constraints
#define ORIENTATION_CONSTRAINT_TOLERANCE 0.1

class j2n6s200MoveitClass
{
//// Methods ////
 public:
  explicit j2n6s200MoveitClass(ros::NodeHandle &nh);

  ~j2n6s200MoveitClass();

/// Motion
  geometry_msgs::Pose getCurrentPose();

  std::vector<double> getCurrentJointAngles();

  void setPoseReferenceFrame(const std::string &pose_reference_frame);

// Motion with wait
  bool moveHome();

  bool moveJoint(const geometry_msgs::Pose &goal);

  bool moveLinear(const std::vector<geometry_msgs::Pose> &waypoints);

// Async motion
  void stopAsyncExecution();

  bool moveHomeAsync();

  bool moveJointAsync(const geometry_msgs::Pose &goal);

  bool moveLinearAsync(const std::vector<geometry_msgs::Pose> &waypoints);

/// Gripper
  bool gripperClose(const float &percentage);

  bool gripperOpen();

/// Pick and place
  bool pick(const gpd_ecsam::DetectedGraspList &detected_grasp_list, const geometry_msgs::Point &object_position,
            const float &object_height, const float &object_radius);

  bool place(const geometry_msgs::Point &goal_position);

/// Path constrains
  void clearConstraints();

  void setOrientationConstraint(const std::string &tolerance);

 private:
/// Initial setup
  void initialGroupSetup();

  void setupHomePose();

  void setupPickPlace();

  void addStaticCollisionObjects();

/// Motion
  bool moveHomePose();

  bool moveHomeJoint();

  void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose);

  void currentJointAnglesCallback(const kinova_msgs::JointAnglesConstPtr &angles);

  void currentFingerPositionCallback(const kinova_msgs::FingerPositionConstPtr &finger_position);

  geometry_msgs::Quaternion computeDirectApproachOrientation(const double &position_x, const double &position_y);

  int8_t computeShorterWeightedTrajectoryInJointSpace(geometry_msgs::Pose poses_to_compare[2]);

/// Gripper
  bool gripperAction(float fingerPosition);

/// Pick and place
  void addPickedObject(const geometry_msgs::Point &position, const float &height, const float &radius);

  void removePlacedObject();

///Octomap
  void clearOctomap();


///// Variables /////
 private:
  ros::NodeHandle n_;
  bool robot_connected_;

/// Motion
  ros::Subscriber sub_joint_angles_, sub_pose_, sub_fingers_position_;
  boost::mutex mutex_joint_angles_, mutex_pose_, mutex_fingers_position_;
  moveit::planning_interface::MoveGroup *arm_group_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::vector<double> home_joint_configuration_, current_joint_angles_;
  geometry_msgs::Pose current_pose_, home_pose_;
  float current_finger_position_percentage_;

///Octomap
  ros::ServiceClient client_clear_octomap_;
  std_srvs::Empty empty_srv_;

/// Gripper
  moveit::planning_interface::MoveGroup *gripper_group_;
  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> *finger_client_;

/// Pick and place
  ros::ServiceClient client_planning_scene_diff_;
  moveit_msgs::CollisionObject picked_collision_object_msg_;
  std::vector<std::string> touch_links_;
  geometry_msgs::Pose transformed_post_grasp_pose_;
  float grasped_position_z_;
};

#endif
