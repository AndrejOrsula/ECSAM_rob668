#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>
#include <boost/thread/mutex.hpp>

#include "../include/ecsam_definitions.h"

#define COMMAND_INIT_CLOSED_EYE_TIME 1.5 //s -> Time required for the user to keep their eye closed
#define COMMAND_INIT_DELAY_TIME      0.75 //s -> Time between user opening their eye after COMMAND_INIT_CLOSED_EYE_TIME and publishing the command

#define TIMER_RATE 0.02

class EcsamInterfaceClass
{
 public:
  EcsamInterfaceClass();

 private:
  // Variables
  ros::NodeHandle n_;
  ros::Subscriber sub_blinking_, sub_ecsam_status_;
  ros::Publisher pub_ecsam_command_;
  ros::Timer interface_frequency_timer_;
  boost::mutex mutex_blinking_;

  ros::Time closed_eye_time_, command_initiated_time_;
  bool eye_closed_, is_blinking_state_changed_, eye_closed_timer_started_, command_initiated_, is_object_grasped_,
      is_action_in_progress_, beeped_;

  sound_play::SoundClient sound_client_;

  // Methods
  void blinkingCallback(const std_msgs::BoolConstPtr &blink);

  void statusCallback(const std_msgs::StringConstPtr &status);

  void timerCallback(const ros::TimerEvent &);
};

EcsamInterfaceClass::EcsamInterfaceClass()
{
  sub_blinking_ = n_.subscribe<std_msgs::Bool>("et7/blinking", 10, &EcsamInterfaceClass::blinkingCallback, this);
  sub_ecsam_status_ = n_.subscribe<std_msgs::String>(std::string(ECSAM_TOPIC_STATUS), 1,
                                                     &EcsamInterfaceClass::statusCallback, this);

  pub_ecsam_command_ = n_.advertise<std_msgs::String>(std::string(ECSAM_TOPIC_COMMAND), 1);

  interface_frequency_timer_ = n_.createTimer(ros::Duration(TIMER_RATE), &EcsamInterfaceClass::timerCallback, this);

  eye_closed_ = is_blinking_state_changed_ = eye_closed_timer_started_ = command_initiated_ = is_object_grasped_ =
  is_action_in_progress_ = beeped_ = false;

#ifdef EXECUTION_TIME_FILE_PATH
  std::ofstream output_file_stream;
  output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

  output_file_stream << "\n-----------------------"
                        "\nNew session begins here at time: " << ros::Time::now();

  output_file_stream.close();
#endif

#ifdef GAZE_CORRELATION_TABLE_FILE_PATH
  std::ofstream output_file_stream_gaze_correlation;
  output_file_stream_gaze_correlation.open(GAZE_CORRELATION_TABLE_FILE_PATH, std::ofstream::out | std::ofstream::app);

output_file_stream_gaze_correlation << "\n-----------------------"
                                     "\nNew session begins here at time: " << ros::Time::now();


  output_file_stream_gaze_correlation.close();
#endif

  ros::Duration(5).sleep();

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
}

void EcsamInterfaceClass::blinkingCallback(const std_msgs::BoolConstPtr &blink)
{
  if (is_action_in_progress_) // Do nothing if action is in progress
  { /// step 4 -> waiting for status callback
    return;
  }

  boost::mutex::scoped_lock lock(mutex_blinking_);

  eye_closed_ = blink->data;
  is_blinking_state_changed_ = true;
}

void EcsamInterfaceClass::statusCallback(const std_msgs::StringConstPtr &status)
{
  sound_client_.say(status->data);

  if (status->data == ECSAM_INFO_PICKING_SUCCESSFUL)
  {
    is_object_grasped_ = true;
  }
  else if (status->data == ECSAM_INFO_PLACING_SUCCESSFUL)
  {
    is_object_grasped_ = false;
  }
  else
  {
    ROS_ERROR_STREAM("Action not successful: " << status->data);
  }

  command_initiated_ = is_action_in_progress_ = false;

#ifdef EXECUTION_TIME_FILE_PATH
  std::ofstream output_file_stream;
  output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

  output_file_stream << "\n" << ros::Time::now() << " " << status->data;

  output_file_stream.close();
#endif
}

void EcsamInterfaceClass::timerCallback(const ros::TimerEvent &)
{
  if (is_action_in_progress_) // Do nothing if action is in progress
  { /// step 4 -> waiting for status callback
    return;
  }
  if (command_initiated_) // eye has been closed for COMMAND_INIT_CLOSED_EYE_TIME and user subsequently opened their eye
  { /// step 3
    if ((ros::Time::now().toSec() - command_initiated_time_.toSec()) >= COMMAND_INIT_DELAY_TIME)
    {
      std_msgs::String command_msg;
#ifdef GAZE_CORRELATION_TABLE_FILE_PATH
      command_msg.data = ECSAM_COMMAND_PLACE;
#else
      if (is_object_grasped_)
      {
        command_msg.data = ECSAM_COMMAND_PLACE;
      }
      else
      {
        command_msg.data = ECSAM_COMMAND_PICK;
      }
#endif
#ifdef GAZE_CORRELATION_OBJECT
      command_msg.data = ECSAM_COMMAND_PICK;
#endif

      pub_ecsam_command_.publish(command_msg);
      sound_client_.say(command_msg.data);
      eye_closed_timer_started_ = false;
      is_action_in_progress_ = true;

#ifdef EXECUTION_TIME_FILE_PATH
      std::ofstream output_file_stream;
      output_file_stream.open(EXECUTION_TIME_FILE_PATH, std::ofstream::out | std::ofstream::app);

      output_file_stream << "\n\n\n" << ros::Time::now() << " " << command_msg.data;

      output_file_stream.close();
#endif
    }
  }
  else // eye has been closed for COMMAND_INIT_CLOSED_EYE_TIME but user did NOT open their eye
  {
    boost::mutex::scoped_lock lock(mutex_blinking_);
    if (eye_closed_timer_started_ && (ros::Time::now().toSec() - closed_eye_time_.toSec()) >=
                                     COMMAND_INIT_CLOSED_EYE_TIME) // eye has been closed for COMMAND_INIT_CLOSED_EYE_TIME
    { /// step 2
      if (!beeped_)
      {
        sound_client_.say("beep");
        beeped_ = true;
      }
      else if (!eye_closed_) // user just opened their eye
      {
        //Activate command init delay timer
        command_initiated_time_ = ros::Time::now();
        command_initiated_ = true;
        beeped_ = false;
      }
    }
    else // eye has NOT been closed for COMMAND_INIT_CLOSED_EYE_TIME
    { /// step 1
      if (!eye_closed_)
      {
        eye_closed_timer_started_ = false;
      }
      else if (is_blinking_state_changed_) // if user just closed their eye, start command timing
      {
        closed_eye_time_ = ros::Time::now();
        eye_closed_timer_started_ = true;
        is_blinking_state_changed_ = false;
        return;
      }
    }

  }
  if (is_blinking_state_changed_)
  {
    is_blinking_state_changed_ = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ecsam_interface");
  EcsamInterfaceClass ecsamInterface;
  return (0);
}
