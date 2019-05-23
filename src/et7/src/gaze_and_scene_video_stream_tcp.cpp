#include <sys/socket.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>

#include "et7/GazeCoordinates.h"
#include "et7/GazeDirection.h"
#include "ecsam_definitions.h"

#define VISUALISE
//#define SIMULATE_EYE_GAZE
#define PUBLISH_GAZE_COORDINATES

#define ADDRESS_ET7   "169.254.181.51"
#define PORT_ET7      51000
#define SAMPLING_RATE (1/29.97) // 29.97 hz

#define SCENE_CAMERA_FRAME "et7_scene_camera_frame"

#define SCENE_CAMERA_RESOLUTION_HORIZONTAL 640
#define SCENE_CAMERA_RESOLUTION_VERTICAL   480
#define SCENE_CAMERA_FOV_HORIZONTAL        (52.5939786 *M_PI/180.0) // must be less than 180 deg
#define SCENE_CAMERA_FOV_VERTICAL          (44.397125  *M_PI/180.0) // must be less than 180 deg

#ifdef VISUALISE

#include <visualization_msgs/Marker.h>

#define VISUALISATION_MARKER_COLOUR_R    0
#define VISUALISATION_MARKER_COLOUR_G    1
#define VISUALISATION_MARKER_COLOUR_B    0.75
#define VISUALISATION_MARKER_LENGTH      100 // length in the direction of camera Z-axis
//    #define VISUALISATION_MARKER_FIXED_WIDTH 0.005 // keep undefined to use correlation distances
#endif

#define FLUSH_BUFFER_SIZE 1000000

#define COMMAND_SEND_BUFFER_SIZE    20
#define MSG_ASL_SIGNATURE_0         0x20 // " "
#define MSG_ASL_SIGNATURE_1         0x41 // "A"
#define MSG_ASL_SIGNATURE_2         0x53 // "S"
#define MSG_ASL_SIGNATURE_3         0x4C // "L"
#define CMD_SET_CONNECT_TYPE        0x07
#define SOCKET_TYPE_SVIDEO_TCP      0x07
#define REPLY_DATA_BEGINNING        56
#define REPLY_DATA_SIZE             9 // 5 for system data + 2 horizontal coordinate + 2 vertical coordinate

class ET7SceneVideoSteamTcpClass
{
 public:
  ET7SceneVideoSteamTcpClass();

  ~ET7SceneVideoSteamTcpClass();

 private:
  // Variables
  int command_socket_, video_socket_;
  struct sockaddr_in server_et7_;
  uint32_t reply_msg_size_;
  bool first_reply_;

  ros::NodeHandle n_;
#ifdef PUBLISH_GAZE_COORDINATES
  ros::Publisher pub_gaze_coordinates_;
#endif
  ros::Subscriber sub_ecsam_user_command_;
  ros::Publisher pub_gaze_direction_;
  ros::Publisher pub_blinking_;
  ros::Publisher pub_scene_video_;
  ros::Timer reply_callback_timer_;
  sensor_msgs::CompressedImage scene_image_;
  boost::mutex mutex_user_command_;

  bool previous_blink_state_, command_received_;
  std::string command_;

  // Methods
  void requestVideoConnection();

  void replyCallback(const ros::TimerEvent &);

  void publishGazeDirection(const float &horizontal_coordinate, const float &vertical_coordinate);

  void userCommandCallback(const std_msgs::StringConstPtr &command);

#ifdef VISUALISE
  ros::Publisher pub_marker_;
  visualization_msgs::Marker marker_;

  void gazeMarkerSetup();

  void visualiseGazeDirection(const double &x, const double &y);

#endif
};

ET7SceneVideoSteamTcpClass::ET7SceneVideoSteamTcpClass()
{
  // Definitions of gaze publishers
#ifdef PUBLISH_GAZE_COORDINATES
  pub_gaze_coordinates_ = n_.advertise<et7::GazeCoordinates>("et7/gaze_coordinates", 1);
#endif
  sub_ecsam_user_command_ = n_.subscribe<std_msgs::String>(std::string(ECSAM_TOPIC_COMMAND), 1,
                                                           &ET7SceneVideoSteamTcpClass::userCommandCallback, this);
  pub_gaze_direction_ = n_.advertise<et7::GazeDirection>("et7/gaze_direction", 1);
  pub_blinking_ = n_.advertise<std_msgs::Bool>("et7/blinking", 10);

#ifdef VISUALISE
  gazeMarkerSetup();
  pub_marker_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  command_ = ECSAM_COMMAND_PLACE;
#endif

#ifdef SIMULATE_EYE_GAZE
  ros::AsyncSpinner simulation_spinner(2);
  simulation_spinner.start();
      while(ros::ok()) {
//            for (int vertical_coordinate = 0; vertical_coordinate < SCENE_CAMERA_RESOLUTION_VERTICAL + 1; vertical_coordinate = vertical_coordinate + SCENE_CAMERA_RESOLUTION_VERTICAL/20.0)
//            {
//                for (int horizontal_coordinate = 0; horizontal_coordinate < SCENE_CAMERA_RESOLUTION_HORIZONTAL + 1; horizontal_coordinate = horizontal_coordinate + SCENE_CAMERA_RESOLUTION_HORIZONTAL/20.0)
//                {

//            int START_VERTICAL = 0;
//            int START_HORIZONTAL = 150;
//            int END_VERTICAL = 100;
//            int END_HORIZONTAL = 200;
//            int STEP_VERTICAL = 3;
//            int STEP_HORIZONTAL = 2;
//            for (int vertical_coordinate = START_VERTICAL; vertical_coordinate < END_VERTICAL + 1; vertical_coordinate = vertical_coordinate + STEP_VERTICAL)
//            {
//                for (int horizontal_coordinate = START_HORIZONTAL; horizontal_coordinate < END_HORIZONTAL + 1; horizontal_coordinate = horizontal_coordinate + STEP_HORIZONTAL)
//                {
////                    std_msgs::Bool blinking_msg;
////                    blinking_msg.data = random() % 101 > 90;
////                    pub_blinking_.publish(blinking_msg);
//                    #ifdef PUBLISH_GAZE_COORDINATES
//                        et7::GazeCoordinates gaze_coordinates_msg;
//                        gaze_coordinates_msg.horizontal = horizontal_coordinate;
//                        gaze_coordinates_msg.vertical = vertical_coordinate;
//                        pub_gaze_coordinates_.publish(gaze_coordinates_msg);
//                    #endif
//                    publishGazeDirection(horizontal_coordinate, vertical_coordinate);
//
//                    ros::Duration(SAMPLING_RATE).sleep();
//                    if (!ros::ok())
//                    {
//                        exit(0);
//                    }
//                }
//            }
          publishGazeDirection(400, 125);
          ros::Duration(SAMPLING_RATE).sleep();
      }
      exit(0);
#endif

  //Video setup
  pub_scene_video_ = n_.advertise<sensor_msgs::CompressedImage>("et7/scene_camera/image_raw/compressed", 1);
  scene_image_.header.frame_id = std::string(SCENE_CAMERA_FRAME);
  scene_image_.format = "JPEG";

  //TCP connections
  server_et7_.sin_addr.s_addr = inet_addr(ADDRESS_ET7);
  server_et7_.sin_family = AF_INET;
  server_et7_.sin_port = htons(PORT_ET7);

  command_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (command_socket_ == -1)
  {
    ROS_ERROR_STREAM("et7 - Could not create \"Command\" socket");
    exit(-1);
  }

  video_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (video_socket_ == -1)
  {
    ROS_ERROR_STREAM("et7 - Could not create \"Data\" socket");
    exit(-1);
  }

  if (connect(command_socket_, (struct sockaddr *) &server_et7_, sizeof(server_et7_)) < 0)
  {
    ROS_ERROR_STREAM("et7 - \"Command\" socket connection failed");
    exit(-1);
  }

  requestVideoConnection();

  reply_msg_size_ = FLUSH_BUFFER_SIZE; //initial size
  first_reply_ = previous_blink_state_ = command_received_ = false;

  if (connect(video_socket_, (struct sockaddr *) &server_et7_, sizeof(server_et7_)) < 0)
  {
    ROS_ERROR_STREAM("et7 - \"Data\" socket connection failed");
    exit(-1);
  }

  reply_callback_timer_ = n_.createTimer(ros::Duration(SAMPLING_RATE), &ET7SceneVideoSteamTcpClass::replyCallback,
                                         this);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
}

ET7SceneVideoSteamTcpClass::~ET7SceneVideoSteamTcpClass()
{
  close(video_socket_);
}

void ET7SceneVideoSteamTcpClass::requestVideoConnection()
{
  unsigned char command_message[COMMAND_SEND_BUFFER_SIZE];
  command_message[0] = MSG_ASL_SIGNATURE_3; // " ASL"
  command_message[1] = MSG_ASL_SIGNATURE_2;
  command_message[2] = MSG_ASL_SIGNATURE_1;
  command_message[3] = MSG_ASL_SIGNATURE_0;
  command_message[4] = 0x14; // Length
  command_message[5] = 0x00;
  command_message[6] = 0x00;
  command_message[7] = 0x00;
  command_message[8] = CMD_SET_CONNECT_TYPE; // Command type
  command_message[9] = 0x00;
  command_message[10] = 0x00;
  command_message[11] = 0x00;
  command_message[12] = 0xDE; // Checksum
  command_message[13] = 0x00;
  command_message[14] = 0x00;
  command_message[15] = 0x00;
  command_message[16] = SOCKET_TYPE_SVIDEO_TCP; // Argument
  command_message[17] = 0x00;
  command_message[18] = 0x00;
  command_message[19] = 0x00;
  if (send(command_socket_, command_message, COMMAND_SEND_BUFFER_SIZE, 0) < 0)
  {
    ROS_ERROR_STREAM("et7 - \"Command\" socket could not initialise \"Data\"");
    exit(-1);
  }
  ros::Duration(0.25).sleep();
  close(command_socket_);
}

void ET7SceneVideoSteamTcpClass::replyCallback(const ros::TimerEvent &)
{
  uint8_t data_reply[reply_msg_size_];

  if (recv(video_socket_, data_reply, reply_msg_size_, 0) < 0)
  {
    ROS_INFO_STREAM("et7 - No \"Data\" packet available yet");
    return;
  }

  scene_image_.header.stamp = ros::Time::now();

  if (data_reply[0] != MSG_ASL_SIGNATURE_3 ||
      data_reply[1] != MSG_ASL_SIGNATURE_2 ||
      data_reply[2] != MSG_ASL_SIGNATURE_1 ||
      data_reply[3] != MSG_ASL_SIGNATURE_0)
  {
    ROS_WARN_STREAM("et7 - \"Data\" packet header does not match");
    reply_msg_size_ = FLUSH_BUFFER_SIZE; //initial size
    first_reply_ = true;
    return;
  }

  if (first_reply_)
  {
    reply_msg_size_ = data_reply[4] + (data_reply[5] << 8) + (data_reply[6] << 16) + (data_reply[7] << 24);
    first_reply_ = false;
  }

  std_msgs::Bool blinking_msg;
  blinking_msg.data = (data_reply[REPLY_DATA_BEGINNING + 1] & 0b00110000) != 0b00110000; // Status byte

  et7::GazeCoordinates gaze_coordinates_msg;
  gaze_coordinates_msg.horizontal =
      0.1 * (data_reply[REPLY_DATA_BEGINNING + 6] << 8 | data_reply[REPLY_DATA_BEGINNING + 5]);
  gaze_coordinates_msg.vertical =
      0.1 * (data_reply[REPLY_DATA_BEGINNING + 8] << 8 | data_reply[REPLY_DATA_BEGINNING + 7]);

  scene_image_.data.resize(reply_msg_size_ - (REPLY_DATA_BEGINNING + REPLY_DATA_SIZE));
  for (int i = 0; i < reply_msg_size_ - (REPLY_DATA_BEGINNING + REPLY_DATA_SIZE); ++i)
  {
    scene_image_.data[i] = data_reply[i + (REPLY_DATA_BEGINNING + REPLY_DATA_SIZE)];
  }

  if (previous_blink_state_ != blinking_msg.data) // In order to publish only the blinking's change of state
  {
    previous_blink_state_ = blinking_msg.data;
    pub_blinking_.publish(blinking_msg);
  }

  pub_scene_video_.publish(scene_image_);

  {
    boost::mutex::scoped_lock lock(mutex_user_command_);
    if (command_received_)
    {
      publishGazeDirection(gaze_coordinates_msg.horizontal, gaze_coordinates_msg.vertical);
#ifdef PUBLISH_GAZE_COORDINATES
      pub_gaze_coordinates_.publish(gaze_coordinates_msg);
#endif
      pub_scene_video_.publish(scene_image_);
      command_received_ = false;
    }
  }
}

void
ET7SceneVideoSteamTcpClass::publishGazeDirection(const float &horizontal_coordinate, const float &vertical_coordinate)
{
  et7::GazeDirection gaze_direction_msg;
  gaze_direction_msg.x = tan(((horizontal_coordinate - SCENE_CAMERA_RESOLUTION_HORIZONTAL / 2.0) /
                              (SCENE_CAMERA_RESOLUTION_HORIZONTAL / 2.0)) * SCENE_CAMERA_FOV_HORIZONTAL / 2.0);
  gaze_direction_msg.y = tan(((vertical_coordinate - SCENE_CAMERA_RESOLUTION_VERTICAL / 2.0) /
                              (SCENE_CAMERA_RESOLUTION_VERTICAL / 2.0)) * SCENE_CAMERA_FOV_VERTICAL / 2.0);

#ifdef VISUALISE
  visualiseGazeDirection(gaze_direction_msg.x, gaze_direction_msg.y);
#endif

  pub_gaze_direction_.publish(gaze_direction_msg);
}

void ET7SceneVideoSteamTcpClass::userCommandCallback(const std_msgs::StringConstPtr &command)
{
  if (command->data == ECSAM_COMMAND_PICK || command->data == ECSAM_COMMAND_PLACE)
  {
    boost::mutex::scoped_lock lock(mutex_user_command_);
    command_received_ = true;
    command_ = command->data;
  }
}

#ifdef VISUALISE

void ET7SceneVideoSteamTcpClass::gazeMarkerSetup()
{
  marker_.header.frame_id = std::string(SCENE_CAMERA_FRAME);
  marker_.lifetime = ros::Duration(5);
  marker_.id = 0;
  marker_.ns = "et7_gaze";
  marker_.type = visualization_msgs::Marker::ARROW;
  marker_.action = visualization_msgs::Marker::ADD;

  marker_.scale.z = 1;

  marker_.color.r = VISUALISATION_MARKER_COLOUR_R;
  marker_.color.g = VISUALISATION_MARKER_COLOUR_G;
  marker_.color.b = VISUALISATION_MARKER_COLOUR_B;
  marker_.color.a = 1.0;

  marker_.points.resize(2);
  marker_.points[0].x = 0;
  marker_.points[0].y = 0;
  marker_.points[0].z = 0;
  marker_.points[1].z = VISUALISATION_MARKER_LENGTH;
}

void ET7SceneVideoSteamTcpClass::visualiseGazeDirection(const double &x, const double &y)
{

#ifndef VISUALISATION_MARKER_FIXED_WIDTH
  if (command_ == ECSAM_COMMAND_PICK)
  {
    marker_.scale.x = marker_.scale.y = PICK_CORRELATION_LINE_POINT_DISTANCE_THRESHOLD;
  }
  else if (command_ == ECSAM_COMMAND_PLACE)
  {
    marker_.scale.x = marker_.scale.y = PLACE_CORRELATION_LINE_POINT_DISTANCE_THRESHOLD;
  }
#else
  marker_.scale.x = marker_.scale.y = VISUALISATION_MARKER_FIXED_WIDTH;
#endif

  marker_.header.stamp = ros::Time::now();
  marker_.points[1].x = VISUALISATION_MARKER_LENGTH * x;
  marker_.points[1].y = VISUALISATION_MARKER_LENGTH * y;
  pub_marker_.publish(marker_);
}

#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "et7_gaze_and_scene_video_stream_tcp");
  ET7SceneVideoSteamTcpClass et7_gaze_and_scene_video_stream_tcp;
  return (0);
}
