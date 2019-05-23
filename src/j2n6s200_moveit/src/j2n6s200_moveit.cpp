#include "../include/j2n6s200_moveit.h"

j2n6s200MoveitClass::j2n6s200MoveitClass(ros::NodeHandle &node_handle)
    : n_(node_handle)
{
  // ROS init (node, communication, topics
  ros::NodeHandle n_params("~");
  n_params.param<bool>("robot_connected", robot_connected_, false);
  n_params.deleteParam("robot_connected");

  current_joint_angles_.resize(NUMBER_OF_JOINTS);
  if (robot_connected_)
  {
    sub_pose_ = n_.subscribe<geometry_msgs::PoseStamped>("/j2n6s200_driver/out/tool_pose", 1,
                                                         &j2n6s200MoveitClass::currentPoseCallback, this);
    sub_joint_angles_ = n_.subscribe<kinova_msgs::JointAngles>("/j2n6s200_driver/out/joint_angles", 1,
                                                               &j2n6s200MoveitClass::currentJointAnglesCallback,
                                                               this);
    sub_fingers_position_ = n_.subscribe<kinova_msgs::FingerPosition>("/j2n6s200_driver/out/finger_position", 1,
                                                                      &j2n6s200MoveitClass::currentFingerPositionCallback,
                                                                      this);

  }

  // MoveIt! init
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();

  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  arm_group_ = new moveit::planning_interface::MoveGroup("arm");
  gripper_group_ = new moveit::planning_interface::MoveGroup("gripper");
  arm_group_->setEndEffectorLink(std::string(ROBOT_TYPE) + "_end_effector");

  client_planning_scene_diff_ = n_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  client_planning_scene_diff_.waitForExistence();

  client_clear_octomap_ = n_.serviceClient<std_srvs::Empty>("clear_octomap");
  client_clear_octomap_.waitForExistence();

  // Gripper init
  finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
      "/" + std::string(ROBOT_TYPE) + "_driver/fingers_action/finger_positions", false);
  while (robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the finger action server to come up");
  }

  initialGroupSetup();
  setupHomePose();
  setupPickPlace();


  // Add static objects
  if (robot_connected_)
  {
    addStaticCollisionObjects();
  }

  // Move to home pose
  gripperOpen();
  ros::Duration(0.5).sleep();
  gripperClose(GRIPPER_COMPACT_CLOSING_FACTOR);
  moveHomeJoint();

  // Add static objects
  if (!robot_connected_)
  {
    addStaticCollisionObjects();
  }
}

j2n6s200MoveitClass::~j2n6s200MoveitClass()
{
  delete arm_group_;
  delete gripper_group_;
  delete finger_client_;
}

/// Motion
geometry_msgs::Pose j2n6s200MoveitClass::getCurrentPose()
{
  if (!robot_connected_)
  {
    return arm_group_->getCurrentPose().pose;
  }
  boost::mutex::scoped_lock lock(mutex_pose_);
  return current_pose_;
}

std::vector<double> j2n6s200MoveitClass::getCurrentJointAngles()
{
  if (!robot_connected_)
  {
    return arm_group_->getCurrentJointValues();
  }
  boost::mutex::scoped_lock lock(mutex_joint_angles_);
  return current_joint_angles_;
}

void j2n6s200MoveitClass::setPoseReferenceFrame(const std::string &pose_reference_frame)
{
  arm_group_->setPoseReferenceFrame(pose_reference_frame);
}

// Motion with wait
bool j2n6s200MoveitClass::moveHome()
{
  if (moveHomeJoint())
  {
    return true;
  }
  else
  {
    if (moveHomePose())
    {
      return true;
    }
  }
  return false;
}

bool j2n6s200MoveitClass::moveHomePose()
{
  arm_group_->setPoseTarget(home_pose_);
  moveit::planning_interface::MoveGroup::Plan trajectory_plan;
  if (arm_group_->plan(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Planning successful, executing path");
    if (arm_group_->execute(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Path executed successfully");
      return true;
    }
    else
    {
      ROS_WARN("Path execution failed");
      return false;
    }
  }
  ROS_WARN("Planning failed");
  return false;
}

bool j2n6s200MoveitClass::moveHomeJoint()
{
  arm_group_->setJointValueTarget(home_joint_configuration_);
  moveit::planning_interface::MoveGroup::Plan trajectory_plan;
  if (arm_group_->plan(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Planning successful, executing path");
    if (arm_group_->execute(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Path executed successfully");
      return true;
    }
    else
    {
      ROS_WARN("Path execution failed");
      return false;
    }
  }
  ROS_WARN("Planning failed");
  return false;
}

bool j2n6s200MoveitClass::moveJoint(const geometry_msgs::Pose &goal)
{
  arm_group_->setPoseTarget(goal);
  moveit::planning_interface::MoveGroup::Plan trajectory_plan;
  if (arm_group_->plan(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Planning successful, executing path");
    if (arm_group_->execute(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Path executed successfully");
      return true;
    }
    else
    {
      ROS_WARN("Path execution failed");
      return false;
    }
  }
  ROS_WARN("Planning failed");
  return false;
}

bool j2n6s200MoveitClass::moveLinear(const std::vector<geometry_msgs::Pose> &waypoints)
{
  moveit::planning_interface::MoveGroup::Plan trajectory_plan;
  if (arm_group_->computeCartesianPath(waypoints, CARTESIAN_PLAN_STEP_SIZE, CARTESIAN_PLAN_JUMP_THRESHOLD,
                                       trajectory_plan.trajectory_) == 1.0)
  {
    ROS_INFO("Planning successful, executing path");
    if (arm_group_->execute(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Path executed successfully");
      return true;
    }
    else
    {
      ROS_WARN("Path execution failed");
      return false;
    }
  }
  ROS_WARN("Planning failed");
  return false;
}

// Async motion
void j2n6s200MoveitClass::stopAsyncExecution()
{
  arm_group_->stop();
}

bool j2n6s200MoveitClass::moveHomeAsync()
{
  arm_group_->setJointValueTarget(home_joint_configuration_);
  moveit::planning_interface::MoveGroup::Plan trajectory_plan;
  if (arm_group_->plan(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Planning successful, executing path");
    if (arm_group_->asyncExecute(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Path executed successfully");
      return true;
    }
    else
    {
      ROS_WARN("Path execution failed");
      return false;
    }
  }
  ROS_WARN("Planning failed");
  return false;
}

bool j2n6s200MoveitClass::moveJointAsync(const geometry_msgs::Pose &goal)
{
  arm_group_->setPoseTarget(goal);
  moveit::planning_interface::MoveGroup::Plan trajectory_plan;
  if (arm_group_->plan(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Planning successful, executing path");
    if (arm_group_->asyncExecute(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Path executed successfully");
      return true;
    }
    else
    {
      ROS_WARN("Path execution failed");
      return false;
    }
  }
  ROS_WARN("Planning failed");
  return false;
}

bool j2n6s200MoveitClass::moveLinearAsync(const std::vector<geometry_msgs::Pose> &waypoints)
{
  moveit::planning_interface::MoveGroup::Plan trajectory_plan;
  if (arm_group_->computeCartesianPath(waypoints, CARTESIAN_PLAN_STEP_SIZE, CARTESIAN_PLAN_JUMP_THRESHOLD,
                                       trajectory_plan.trajectory_) == 1.0)
  {
    ROS_INFO("Planning successful, executing path");
    if (arm_group_->asyncExecute(trajectory_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Path executed successfully");
      return true;
    }
    else
    {
      ROS_WARN("Path execution failed");
      return false;
    }
  }
  ROS_WARN("Planning failed");
  return false;
}

/// Gripper
bool j2n6s200MoveitClass::gripperClose(const float &percentage)
{
  return gripperAction(percentage * FINGER_MAX);
}

bool j2n6s200MoveitClass::gripperOpen()
{
  return gripperAction(0.0);
}

/// Pick and place
bool j2n6s200MoveitClass::pick(const gpd_ecsam::DetectedGraspList &detected_grasp_list,
                               const geometry_msgs::Point &object_position, const float &object_height,
                               const float &object_radius)
{
  clearOctomap();

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(detected_grasp_list.detected_grasps.size());

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf_listener(
      tf2_buffer); // This is used for transforming the grasp into the arm coordinate frame
  geometry_msgs::TransformStamped transform_stamped;

  try
  {
    transform_stamped = tf2_buffer.lookupTransform(arm_group_->getPoseReferenceFrame(),
                                                   detected_grasp_list.header.frame_id, ros::Time(0),
                                                   ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  bool away_from_home = false;

  uint8_t number_of_grasps_attempted = 0;
  for (int i = 0; i < detected_grasp_list.detected_grasps.size(); i++)
  {
    if (number_of_grasps_attempted >= MAXIMUM_NUMBER_OF_PICK_ATTEMPTS)
    {
      break;
    }

    if (i % 2 == 1)
    {
      i++; // This is performed as every grasp is send twice (to account for gripper symmetry)
      if (i >= detected_grasp_list.detected_grasps.size())
      {
        break;
      }
    }

    geometry_msgs::Pose original_grasp_finger_pose, transformed_grasp_finger_pose;
    original_grasp_finger_pose.position.x =
        detected_grasp_list.detected_grasps[i].pose.position.x + TCP_TO_FINGERTIP_DISTANCE *
                                                                 detected_grasp_list.detected_grasps[i].approach.x;
    original_grasp_finger_pose.position.y =
        detected_grasp_list.detected_grasps[i].pose.position.y + TCP_TO_FINGERTIP_DISTANCE *
                                                                 detected_grasp_list.detected_grasps[i].approach.y;
    original_grasp_finger_pose.position.z =
        detected_grasp_list.detected_grasps[i].pose.position.z + TCP_TO_FINGERTIP_DISTANCE *
                                                                 detected_grasp_list.detected_grasps[i].approach.z;
    original_grasp_finger_pose.orientation = detected_grasp_list.detected_grasps[i].pose.orientation;
    tf2::doTransform(original_grasp_finger_pose, transformed_grasp_finger_pose, transform_stamped);

    if (transformed_grasp_finger_pose.position.z + TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT <
        WRIST_TABLE_MINIMAL_DISTANCE)
    {
      continue; // Try another grasp since this one is too close to table
    }

    if (!away_from_home)
    {
      gripperClose(GRIPPER_COMPACT_CLOSING_FACTOR); // Make gripper compact to ease obstacle avoidance
    }

    geometry_msgs::Pose original_pre_grasp_pose[2], transformed_pre_grasp_pose[2];
    original_pre_grasp_pose[0].position.x = detected_grasp_list.detected_grasps[i].pose.position.x -
                                            PICK_APPROACH_DISTANCE *
                                            detected_grasp_list.detected_grasps[i].approach.x;
    original_pre_grasp_pose[0].position.y = detected_grasp_list.detected_grasps[i].pose.position.y -
                                            PICK_APPROACH_DISTANCE *
                                            detected_grasp_list.detected_grasps[i].approach.y;
    original_pre_grasp_pose[0].position.z = detected_grasp_list.detected_grasps[i].pose.position.z -
                                            PICK_APPROACH_DISTANCE *
                                            detected_grasp_list.detected_grasps[i].approach.z;
    original_pre_grasp_pose[0].orientation = detected_grasp_list.detected_grasps[i].pose.orientation;
    tf2::doTransform(original_pre_grasp_pose[0], transformed_pre_grasp_pose[0], transform_stamped);

    original_pre_grasp_pose[1].position = original_pre_grasp_pose[0].position;
    original_pre_grasp_pose[1].orientation = detected_grasp_list.detected_grasps[i + 1].pose.orientation;
    tf2::doTransform(original_pre_grasp_pose[1], transformed_pre_grasp_pose[1], transform_stamped);

    // This part selects one of the symetric grasps that is closer in joint space (weighted)
    int8_t offset = computeShorterWeightedTrajectoryInJointSpace(transformed_pre_grasp_pose);
    if (offset == -1)
    {
      continue;
    }
    i += offset;

    // Decide whether it is pinching or cylindrical grasp
    float picking_distance_offset;
    if (detected_grasp_list.detected_grasps[i].grasp_width.data < PICK_PINCHING_THRESHOLD)
    {
      picking_distance_offset = PICKING_PINCHING_GRASP_FINGER_DISTANCE;
    }
    else
    {
      picking_distance_offset = PICKING_CYLINDRICAL_GRASP_FINGER_DISTANCE;
    }

    geometry_msgs::Pose original_grasp_pose;
    std::vector<geometry_msgs::Pose> transformed_grasp_pose;
    transformed_grasp_pose.resize(1);
    original_grasp_pose.position.x = detected_grasp_list.detected_grasps[i].pose.position.x -
                                     picking_distance_offset * detected_grasp_list.detected_grasps[i].approach.x;
    original_grasp_pose.position.y = detected_grasp_list.detected_grasps[i].pose.position.y -
                                     picking_distance_offset * detected_grasp_list.detected_grasps[i].approach.y;
    original_grasp_pose.position.z = detected_grasp_list.detected_grasps[i].pose.position.z -
                                     picking_distance_offset * detected_grasp_list.detected_grasps[i].approach.z;
    original_grasp_pose.orientation = detected_grasp_list.detected_grasps[i].pose.orientation;
    tf2::doTransform(original_grasp_pose, transformed_grasp_pose[0], transform_stamped);

    if (transformed_grasp_pose[0].position.z + TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT <
        WRIST_TABLE_MINIMAL_DISTANCE)
    {
      continue; // Try another grasp since this one is too close to table
    }

    addPickedObject(object_position, object_height, object_radius);

    if (!moveJoint(transformed_pre_grasp_pose[offset]))
    {
#ifdef EXECUTION_TIME_FILE_PATH
      std::ofstream output_file_stream;
      output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

      output_file_stream << "\n" << ros::Time::now() << " Pre-grasp-motion failed";

      output_file_stream.close();
#endif
      continue; // Try another grasp
    }
#ifdef EXECUTION_TIME_FILE_PATH
    std::ofstream output_file_stream;
    output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

    output_file_stream << "\n" << ros::Time::now() << " Pre-grasp-motion succeeded";

    output_file_stream.close();
#endif

    away_from_home = true;
    number_of_grasps_attempted++;

    removePlacedObject();

    gripperOpen();

    if (!moveLinear(transformed_grasp_pose))
    {
      clearOctomap();
#ifdef EXECUTION_TIME_FILE_PATH
      output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

      output_file_stream << "\n" << ros::Time::now() << " Grasp-motion failed";

      output_file_stream.close();
#endif
      continue; // Try another grasp
    }
#ifdef EXECUTION_TIME_FILE_PATH
    output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

    output_file_stream << "\n" << ros::Time::now() << " Grasp-motion succeeded";

    output_file_stream.close();
#endif

    std::vector<geometry_msgs::Pose> transformed_post_grasp_pose;

    gripperClose(GRIPPER_GRASPING_CLOSING_FACTOR);

    addPickedObject(getCurrentPose().position, object_height, object_radius);
    arm_group_->attachObject(std::string(GRASPED_OBJECT_ID), std::string(ROBOT_TYPE) + "_end_effector",
                             touch_links_);

    transformed_post_grasp_pose.push_back(getCurrentPose());
    transformed_post_grasp_pose[0].position.z += POST_PICK_PRE_PLACE_ASCENSION_DISTANCE;
    transformed_post_grasp_pose_.orientation = computeDirectApproachOrientation(POST_PICK_DEFAULT_POSITION_X,
                                                                                POST_PICK_DEFAULT_POSITION_Y);

    bool ascension_failed = false;
    if (!moveLinear(transformed_post_grasp_pose))
    {
      if (!moveJoint(transformed_post_grasp_pose_))
      {
#ifdef EXECUTION_TIME_FILE_PATH
        output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

        output_file_stream << "\n" << ros::Time::now() << " Post-grasp-motion failed";

        output_file_stream.close();
#endif
        ascension_failed = true;
      }
#ifdef EXECUTION_TIME_FILE_PATH
      output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

      output_file_stream << "\n" << ros::Time::now() << " Post-grasp-motion failed";

      output_file_stream.close();
#endif
    }

    // Release object and try another grasp if it is not possible to ascend
    if (ascension_failed)
    {
      ROS_WARN_STREAM("Cannot ascend the grasped object. Attempting a different grasp...");
      gripperOpen();
      arm_group_->detachObject(std::string(GRASPED_OBJECT_ID));
      removePlacedObject();

      // Return to pre-grasp pose
      std::vector<geometry_msgs::Pose> transformed_pre_grasp_failed_pose;
      transformed_pre_grasp_failed_pose.push_back(transformed_pre_grasp_pose[offset]);
      moveLinear(transformed_pre_grasp_failed_pose);

#ifdef EXECUTION_TIME_FILE_PATH
      output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

      output_file_stream << "\n" << ros::Time::now() << " Different grasp is attempted";

      output_file_stream.close();
#endif

      continue;
    }

#ifdef EXECUTION_TIME_FILE_PATH
    output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

    output_file_stream << "\n" << ros::Time::now() << " Post-grasp-motion succeeded";

    output_file_stream.close();
#endif

    if (!ascension_failed && !moveJoint(transformed_post_grasp_pose_))
    {
      arm_group_->setGoalPositionTolerance(
          arm_group_->getGoalPositionTolerance() * DEFAULT_POSE_POSITION_TOLERANCE_INCREASE_STEP_MULTIPLIER);
      arm_group_->setGoalOrientationTolerance(arm_group_->getGoalOrientationTolerance() *
                                              DEFAULT_POSE_ORIENTATION_TOLERANCE_INCREASE_STEP_MULTIPLIER);
      while (!moveJoint(transformed_post_grasp_pose_))
      {
        arm_group_->setGoalPositionTolerance(arm_group_->getGoalPositionTolerance() *
                                             DEFAULT_POSE_POSITION_TOLERANCE_INCREASE_STEP_MULTIPLIER);
        arm_group_->setGoalOrientationTolerance(arm_group_->getGoalOrientationTolerance() *
                                                DEFAULT_POSE_ORIENTATION_TOLERANCE_INCREASE_STEP_MULTIPLIER);
        clearOctomap();
      }
      arm_group_->setGoalPositionTolerance(GOAL_POSITION_TOLERANCE);
      arm_group_->setGoalOrientationTolerance(GOAL_ORIENTATION_TOLERANCE);
    }

    if (robot_connected_)
    {
      gripperClose(GRIPPER_GRASPING_CLOSING_FACTOR);
      ros::Duration(0.1).sleep();
      {
        boost::mutex::scoped_lock lock(mutex_fingers_position_);
        if (GRIPPER_GRASPING_CLOSING_FACTOR - current_finger_position_percentage_ >
            SUCCESSFUL_GRASP_PERCENTAGE)
        {
          grasped_position_z_ = transformed_grasp_pose[0].position.z;
          ROS_INFO_STREAM("Picking successful");

          return true;
        }
        else
        {
          ROS_WARN_STREAM("Object was not grasped successfully");

          arm_group_->detachObject(std::string(GRASPED_OBJECT_ID));
          removePlacedObject();

#ifdef EXECUTION_TIME_FILE_PATH
          output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

          output_file_stream << "\n" << ros::Time::now() << " Object is not in the gripper";

          output_file_stream.close();
#endif
          return false;
        }
      }
    }
    else
    {
      grasped_position_z_ = transformed_grasp_pose[0].position.z;
      ROS_INFO_STREAM("Picking successful");

      return true;
    }
  }
  ROS_WARN_STREAM("No viable grasp detected. Terminating...");
  if (away_from_home)
  {
    moveHome();
    clearOctomap();
  }
  return false;
}

bool j2n6s200MoveitClass::place(const geometry_msgs::Point &goal_position)
{
  geometry_msgs::Pose pre_placement_pose;
  std::vector<geometry_msgs::Pose> placement_pose;

  pre_placement_pose.orientation = computeDirectApproachOrientation(goal_position.x, goal_position.y);
  pre_placement_pose.position = goal_position;
  pre_placement_pose.position.z = grasped_position_z_ + PLACING_SAFETY_HEIGHT;

  static robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model_));
  static const robot_state::JointModelGroup *joint_model_group = robot_model_->getJointModelGroup("arm");
  if (!robot_state->setFromIK(joint_model_group, pre_placement_pose, PICK_NUMBER_OF_IK_ATTEMPTS, PICK_IK_TIMEOUT))
  {
    return false;
  }

  placement_pose.push_back(pre_placement_pose);
  pre_placement_pose.position.z += POST_PICK_PRE_PLACE_ASCENSION_DISTANCE;

  if (!moveJoint(pre_placement_pose))
  {
#ifdef EXECUTION_TIME_FILE_PATH
    std::ofstream output_file_stream;
    output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

    output_file_stream << "\n" << ros::Time::now() << " Pre-placement-motion failed";

    output_file_stream.close();
#endif
    clearOctomap();
    return false;
  }
#ifdef EXECUTION_TIME_FILE_PATH
  std::ofstream output_file_stream;
  output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

  output_file_stream << "\n" << ros::Time::now() << " Pre-placement-motion succeeded";

  output_file_stream.close();
#endif

  if (!moveLinear(placement_pose))
  {
#ifdef EXECUTION_TIME_FILE_PATH
    output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

    output_file_stream << "\n" << ros::Time::now() << " Placement-motion failed";

    output_file_stream.close();
#endif
    // Return to default position
    transformed_post_grasp_pose_.orientation = computeDirectApproachOrientation(POST_PICK_DEFAULT_POSITION_X,
                                                                                POST_PICK_DEFAULT_POSITION_Y);
    moveJoint(transformed_post_grasp_pose_);

    clearOctomap();
    return false;
  }
#ifdef EXECUTION_TIME_FILE_PATH
  output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

  output_file_stream << "\n" << ros::Time::now() << " Placement-motion succeeded";

  output_file_stream.close();
#endif

  gripperOpen();
  arm_group_->detachObject(std::string(GRASPED_OBJECT_ID));
  removePlacedObject();

  std::vector<geometry_msgs::Pose> post_placement_pose;
  post_placement_pose.push_back(getCurrentPose());
  post_placement_pose[0].position.z = goal_position.z +
                                      picked_collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] +
                                      POST_PLACING_SAFETY_HEIGHT;

  moveLinear(post_placement_pose);

  moveHome();

  clearOctomap();
  return true;
}

/// Path constrains
void j2n6s200MoveitClass::clearConstraints()
{
  arm_group_->clearPathConstraints();
}

void j2n6s200MoveitClass::setOrientationConstraint(const std::string &tolerance)
{
  moveit_msgs::OrientationConstraint orientation_constraint;
  moveit_msgs::Constraints constraints;
  orientation_constraint.header.frame_id = arm_group_->getPoseReferenceFrame();
  orientation_constraint.link_name = std::string(ROBOT_TYPE) + "_end_effector";
  orientation_constraint.orientation = getCurrentPose().orientation;
  orientation_constraint.weight = 1.0;

  for (char letter : tolerance)
  {
    switch (letter)
    {
      case 'x':orientation_constraint.absolute_x_axis_tolerance = 2 * M_PI;
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_CONSTRAINT_TOLERANCE;
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_CONSTRAINT_TOLERANCE;
        break;
      case 'y':orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_CONSTRAINT_TOLERANCE;
        orientation_constraint.absolute_y_axis_tolerance = 2 * M_PI;
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_CONSTRAINT_TOLERANCE;
        break;
      case 'z':orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_CONSTRAINT_TOLERANCE;
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_CONSTRAINT_TOLERANCE;
        orientation_constraint.absolute_z_axis_tolerance = 2 * M_PI;
        break;
      default:break;
    }
  }

  constraints.orientation_constraints.push_back(orientation_constraint);
  arm_group_->setPathConstraints(constraints);
}

///// PRIVATE /////
/// Initial setup
void j2n6s200MoveitClass::initialGroupSetup()
{
  arm_group_->setPoseReferenceFrame("root");
  arm_group_->setNamedTarget("Home");
  gripper_group_->setNamedTarget("Open");
  arm_group_->setNumPlanningAttempts(NUMBER_OF_PLANNING_ATTEMPTS);
  arm_group_->setPlanningTime(PLANNING_TIME);
  arm_group_->setGoalPositionTolerance(GOAL_POSITION_TOLERANCE);
  arm_group_->setGoalOrientationTolerance(GOAL_ORIENTATION_TOLERANCE);
  arm_group_->detachObject(std::string(GRASPED_OBJECT_ID));
  removePlacedObject();
  clearOctomap();
}

void j2n6s200MoveitClass::setupHomePose()
{
  home_joint_configuration_.resize(6);
  home_joint_configuration_[0] = HOME_POSE_JOINT_0;
  home_joint_configuration_[1] = HOME_POSE_JOINT_1;
  home_joint_configuration_[2] = HOME_POSE_JOINT_2;
  home_joint_configuration_[3] = HOME_POSE_JOINT_3;
  home_joint_configuration_[4] = HOME_POSE_JOINT_4;
  home_joint_configuration_[5] = HOME_POSE_JOINT_5;

  home_pose_.position.x = HOME_POSE_POSITION_X;
  home_pose_.position.y = HOME_POSE_POSITION_Y;
  home_pose_.position.z = HOME_POSE_POSITION_Z;
  home_pose_.orientation.x = HOME_POSE_ORIENTATION_X;
  home_pose_.orientation.y = HOME_POSE_ORIENTATION_Y;
  home_pose_.orientation.z = HOME_POSE_ORIENTATION_Z;
  home_pose_.orientation.w = HOME_POSE_ORIENTATION_W;
}

void j2n6s200MoveitClass::setupPickPlace()
{
  arm_group_->setSupportSurfaceName("table");
  picked_collision_object_msg_.primitives.resize(1);
  picked_collision_object_msg_.primitive_poses.resize(1);
  picked_collision_object_msg_.id = std::string(GRASPED_OBJECT_ID);
  picked_collision_object_msg_.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  picked_collision_object_msg_.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);

  touch_links_.push_back(std::string(ROBOT_TYPE) + "_end_effector");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_finger_1");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_finger_2");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_finger_3");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_finger_tip_1");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_finger_tip_2");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_finger_tip_3");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_4");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_5");
  touch_links_.push_back(std::string(ROBOT_TYPE) + "_link_6");

  transformed_post_grasp_pose_.position.x = POST_PICK_DEFAULT_POSITION_X;
  transformed_post_grasp_pose_.position.y = POST_PICK_DEFAULT_POSITION_Y;
  transformed_post_grasp_pose_.position.z = POST_PICK_DEFAULT_POSITION_Z;

  grasped_position_z_ = 0;
}

void j2n6s200MoveitClass::addStaticCollisionObjects()
{
  std::vector<moveit_msgs::CollisionObject> static_collision_objects_msg;
  static_collision_objects_msg.resize(NUMBER_OF_STATIC_COLLISION_OBJECTS);

  static_collision_objects_msg[0].header.frame_id = arm_group_->getPoseReferenceFrame();
  static_collision_objects_msg[0].header.stamp = ros::Time::now();
  for (int i = 1; i < NUMBER_OF_STATIC_COLLISION_OBJECTS; ++i)
  {
    static_collision_objects_msg[i].header.frame_id = static_collision_objects_msg[0].header.frame_id;
    static_collision_objects_msg[i].header.stamp = static_collision_objects_msg[0].header.stamp;
  }
  for (int i = 0; i < NUMBER_OF_STATIC_COLLISION_OBJECTS; ++i)
  {
    static_collision_objects_msg[i].primitives.resize(1);
    static_collision_objects_msg[i].primitive_poses.resize(1);
  }

  static_collision_objects_msg[0].id = "table";
  static_collision_objects_msg[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  static_collision_objects_msg[0].primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  static_collision_objects_msg[0].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = TABLE_DIMENSION_X;
  static_collision_objects_msg[0].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = TABLE_DIMENSION_Y;
  static_collision_objects_msg[0].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = TABLE_DIMENSION_Z;
  static_collision_objects_msg[0].primitive_poses[0].position.x = TABLE_POSITION_X;
  static_collision_objects_msg[0].primitive_poses[0].position.y = TABLE_POSITION_Y;
  static_collision_objects_msg[0].primitive_poses[0].position.z = TABLE_POSITION_Z;
  static_collision_objects_msg[0].operation = moveit_msgs::CollisionObject::ADD;

  static_collision_objects_msg[1].id = "table_extended";
  static_collision_objects_msg[1].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  static_collision_objects_msg[1].primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  static_collision_objects_msg[1].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] =
      TABLE_EXTENDED_DIMENSION_X;
  static_collision_objects_msg[1].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] =
      TABLE_EXTENDED_DIMENSION_Y;
  static_collision_objects_msg[1].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] =
      TABLE_EXTENDED_DIMENSION_Z;
  static_collision_objects_msg[1].primitive_poses[0].position.x = TABLE_EXTENDED_POSITION_X;
  static_collision_objects_msg[1].primitive_poses[0].position.y = TABLE_EXTENDED_POSITION_Y;
  static_collision_objects_msg[1].primitive_poses[0].position.z = TABLE_EXTENDED_POSITION_Z;
  static_collision_objects_msg[1].operation = moveit_msgs::CollisionObject::ADD;

  static_collision_objects_msg[2].id = "table_extra";
  static_collision_objects_msg[2].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  static_collision_objects_msg[2].primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  static_collision_objects_msg[2].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] =
      TABLE_EXTRA_DIMENSION_X;
  static_collision_objects_msg[2].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] =
      TABLE_EXTRA_DIMENSION_Y;
  static_collision_objects_msg[2].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] =
      TABLE_EXTRA_DIMENSION_Z;
  static_collision_objects_msg[2].primitive_poses[0].position.x = TABLE_EXTRA_POSITION_X;
  static_collision_objects_msg[2].primitive_poses[0].position.y = TABLE_EXTRA_POSITION_Y;
  static_collision_objects_msg[2].primitive_poses[0].position.z = TABLE_EXTRA_POSITION_Z;
  static_collision_objects_msg[2].operation = moveit_msgs::CollisionObject::ADD;

  static_collision_objects_msg[3].id = "camera";
  static_collision_objects_msg[3].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  static_collision_objects_msg[3].primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  static_collision_objects_msg[3].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = CAMERA_DIMENSION_X;
  static_collision_objects_msg[3].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = CAMERA_DIMENSION_Y;
  static_collision_objects_msg[3].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = CAMERA_DIMENSION_Z;
  static_collision_objects_msg[3].primitive_poses[0].position.x = CAMERA_POSITION_X;
  static_collision_objects_msg[3].primitive_poses[0].position.y = CAMERA_POSITION_Y;
  static_collision_objects_msg[3].primitive_poses[0].position.z = CAMERA_POSITION_Z;
  static_collision_objects_msg[3].operation = moveit_msgs::CollisionObject::ADD;

  static_collision_objects_msg[4].id = "user";
  static_collision_objects_msg[4].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  static_collision_objects_msg[4].primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  static_collision_objects_msg[4].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = USER_DIMENSION_X;
  static_collision_objects_msg[4].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = USER_DIMENSION_Y;
  static_collision_objects_msg[4].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = USER_DIMENSION_Z;
  static_collision_objects_msg[4].primitive_poses[0].position.x = USER_POSITION_X;
  static_collision_objects_msg[4].primitive_poses[0].position.y = USER_POSITION_Y;
  static_collision_objects_msg[4].primitive_poses[0].position.z = USER_POSITION_Z;
  static_collision_objects_msg[4].operation = moveit_msgs::CollisionObject::ADD;

  moveit_msgs::PlanningScene planning_scene_diff_msg;
  planning_scene_diff_msg.is_diff = true;
  for (int i = 0; i < NUMBER_OF_STATIC_COLLISION_OBJECTS; ++i)
  {
    planning_scene_diff_msg.world.collision_objects.push_back(static_collision_objects_msg[i]);
  }

  moveit_msgs::ApplyPlanningScene srv_apply_planning_scene;
  srv_apply_planning_scene.request.scene = planning_scene_diff_msg;
  client_planning_scene_diff_.call(srv_apply_planning_scene);
}

/// Motion
void j2n6s200MoveitClass::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
  boost::mutex::scoped_lock lock(mutex_pose_);
  current_pose_ = pose->pose;
}

void j2n6s200MoveitClass::currentJointAnglesCallback(const kinova_msgs::JointAnglesConstPtr &angles)
{
  boost::mutex::scoped_lock lock(mutex_joint_angles_);
  current_joint_angles_[0] = angles->joint1;
  current_joint_angles_[1] = angles->joint2;
  current_joint_angles_[2] = angles->joint3;
  current_joint_angles_[3] = angles->joint4;
  current_joint_angles_[4] = angles->joint5;
  current_joint_angles_[5] = angles->joint6;
}

void j2n6s200MoveitClass::currentFingerPositionCallback(const kinova_msgs::FingerPositionConstPtr &finger_position)
{
  boost::mutex::scoped_lock lock(mutex_fingers_position_);
  current_finger_position_percentage_ = (finger_position->finger1 + finger_position->finger2) / (2 * FINGER_MAX);
}

int8_t j2n6s200MoveitClass::computeShorterWeightedTrajectoryInJointSpace(
    geometry_msgs::Pose poses_to_compare[2]) // Used to figure out which one of the symmetric grasp is closer in joint space
{
  static robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model_));
  static const robot_state::JointModelGroup *joint_model_group = robot_model_->getJointModelGroup("arm");
  static std::vector<double> ik_joint_values[2];
  bool is_reachable = false;
  float joint_space_weighted_distance[2] = {0.0, 0.0};

  for (int j = 0; j < 2; ++j)
  {
    if (robot_state->setFromIK(joint_model_group, poses_to_compare[j], PICK_NUMBER_OF_IK_ATTEMPTS,
                               PICK_IK_TIMEOUT))
    {
      ik_joint_values[j].resize(NUMBER_OF_JOINTS);
      robot_state->copyJointGroupPositions(joint_model_group, ik_joint_values[j]);
      is_reachable = true;
    }
  }

  if (!is_reachable)
  {
    return -1;
  }

  static std::vector<double> current_joint_angles(NUMBER_OF_JOINTS);
  current_joint_angles = getCurrentJointAngles();

  for (int j = 0; j < 2; ++j)
  {
    for (int k = 0; k < NUMBER_OF_JOINTS; ++k)
    {
      joint_space_weighted_distance[j] += JOINT_DISTANCE_WEIGHTS[k] *
                                          (fabsf(fmodf(ik_joint_values[j][k], 2 * M_PI) -
                                                 fmodf(current_joint_angles[k], 2 * M_PI)));
    }
  }

  if (joint_space_weighted_distance[1] < joint_space_weighted_distance[0])
  {
    return 1;
  }
  return 0;
}

geometry_msgs::Quaternion
j2n6s200MoveitClass::computeDirectApproachOrientation(const double &position_x, const double &position_y)
{
  geometry_msgs::Quaternion current_rotation_quat_msg = getCurrentPose().orientation;
  tf2::Quaternion current_rotation_quat, goal_rotation_quat;
  tf2::fromMsg(current_rotation_quat_msg, current_rotation_quat);
  tf2::Matrix3x3 current_rotation_mat(current_rotation_quat);

  float absolute_z_rotation = atan2(position_y, position_x) - atan2(current_rotation_mat.getColumn(2).getY(),
                                                                    current_rotation_mat.getColumn(2).getX());
  tf2::Matrix3x3 goal_rotation_mat(cosf(absolute_z_rotation), -sinf(absolute_z_rotation), 0,
                                   sinf(absolute_z_rotation), cosf(absolute_z_rotation), 0,
                                   0, 0, 1);

  goal_rotation_mat = goal_rotation_mat * current_rotation_mat;
  goal_rotation_mat.getRotation(goal_rotation_quat);

  return tf2::toMsg(goal_rotation_quat);
}

/// Gripper
bool j2n6s200MoveitClass::gripperAction(float finger_position)
{
  if (!robot_connected_)
  {
    if (finger_position > 0.5 * FINGER_MAX)
    {
      gripper_group_->setNamedTarget("Close");
    }
    else
    {
      gripper_group_->setNamedTarget("Open");
    }
    gripper_group_->move();
    return true;
  }

  if (finger_position < 0)
  {
    finger_position = 0.0;
  }
  else if (finger_position > FINGER_MAX)
  {
    finger_position = FINGER_MAX;
  }

  kinova_msgs::SetFingersPositionGoal goal;
  goal.fingers.finger1 = goal.fingers.finger2 = finger_position;
  finger_client_->sendGoal(goal); //

  if (finger_client_->waitForResult(ros::Duration(5.0)))
  {
    finger_client_->getResult();
    return true;
  }
  else
  {
    finger_client_->cancelAllGoals();
    ROS_WARN("The gripper action failed");
    return false;
  }
}

/// Pick and place
void
j2n6s200MoveitClass::addPickedObject(const geometry_msgs::Point &position, const float &height, const float &radius)
{
  picked_collision_object_msg_.id = std::string(GRASPED_OBJECT_ID);
  picked_collision_object_msg_.header.frame_id = arm_group_->getPoseReferenceFrame();

  picked_collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] =
      height * PICKED_OBJECT_CREATION_HEIGHT_SCALING_FACTOR;
  if (radius > MAX_OBJECT_RADIUS)
  {
    picked_collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] =
        MAX_OBJECT_RADIUS;
  }
  else
  {
    picked_collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] =
        radius * PICKED_OBJECT_CREATION_RADIUS_SCALING_FACTOR;
  }

  picked_collision_object_msg_.primitive_poses[0].position.x = position.x;
  picked_collision_object_msg_.primitive_poses[0].position.y = position.y;
  picked_collision_object_msg_.primitive_poses[0].position.z =
      height / 2 - TABLE_POSITION_Z_OFFSET_WITH_PLATE_HEIGHT +
      PICKED_OBJECT_CREATION_OFFSET_FROM_TABLE; //TODO: make height based on distance to the table ???
  picked_collision_object_msg_.operation = moveit_msgs::CollisionObject::ADD;

  moveit_msgs::PlanningScene planning_scene_diff_msg;
  planning_scene_diff_msg.is_diff = true;
  planning_scene_diff_msg.world.collision_objects.push_back(picked_collision_object_msg_);

  moveit_msgs::ApplyPlanningScene srv_apply_planning_scene_;
  srv_apply_planning_scene_.request.scene = planning_scene_diff_msg;
  client_planning_scene_diff_.call(srv_apply_planning_scene_);
}

void j2n6s200MoveitClass::removePlacedObject()
{
  picked_collision_object_msg_.id = std::string(GRASPED_OBJECT_ID);
  picked_collision_object_msg_.operation = moveit_msgs::CollisionObject::REMOVE;

  moveit_msgs::PlanningScene planning_scene_diff_msg;
  planning_scene_diff_msg.is_diff = true;
  planning_scene_diff_msg.world.collision_objects.push_back(picked_collision_object_msg_);

  moveit_msgs::ApplyPlanningScene srv_apply_planning_scene_;
  srv_apply_planning_scene_.request.scene = planning_scene_diff_msg;
  client_planning_scene_diff_.call(srv_apply_planning_scene_);
}

void j2n6s200MoveitClass::clearOctomap()
{
  arm_group_->stop();
  if (!client_clear_octomap_.isValid())
  {
    client_clear_octomap_.shutdown();
    client_clear_octomap_ = n_.serviceClient<std_srvs::Empty>("clear_octomap");
    client_clear_octomap_.waitForExistence();
  }
  client_clear_octomap_.call(empty_srv_);
}
