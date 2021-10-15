
#include "ros/ros.h"
#include "std_msgs/String.h"

#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include <thread>
#include <string>

#include "RobotStatus.pb.h"
#include "JoystickStatus.pb.h"

#include <rio_control_node/Joystick_Status.h>

ros::NodeHandle * node;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

void robot_receive_loop ()
{
  void *context = zmq_ctx_new ();
  void *subscriber = zmq_socket(context, ZMQ_DISH);

  int rc = zmq_bind(subscriber, "udp://*:5801");
  rc = zmq_join(subscriber, "robotstatus");
  rc = zmq_join(subscriber, "joystickstatus");

  // ck::RobotStatus status;
  char buffer [10000];

  memset(buffer, 0, 10000);

  ROS_INFO("WOOGITY WOOGITY test WOO %d %d %d", context, subscriber, rc);

  ros::Publisher joystick_pub = node->advertise<rio_control_node::Joystick_Status>("JoystickStatus", 1);

  while (ros::ok())
  {
    zmq_msg_t message;
    zmq_msg_init(&message);
    zmq_msg_recv(&message, subscriber, 0);

    std::string message_group(zmq_msg_group(&message));

    switch(str2int(message_group.c_str()))
    {
      case str2int("joystickstatus"):
        {
          static ck::JoystickStatus status;
          void * data = zmq_msg_data(&message);
          bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
          if (parse_result)
          {

            rio_control_node::Joystick_Status joystick_status;

            for (int i = 0; i < status.joysticks_size(); i++)
            {
              const ck::JoystickStatus::Joystick& joystick = status.joysticks(i);
              rio_control_node::Joystick stick;

              stick.index = joystick.index();
              
              uint32_t buttons = joystick.buttons();
              for (int j = 0; j < 32; j++)
              {
                stick.buttons.push_back((buttons >> j) & 0x0001);
              }

              for(int j = 0; j < joystick.axes_size(); j++)
              {
                stick.axes.push_back(joystick.axes(j));
              }

              for(int j = 0; j < joystick.povs_size(); j++)
              {
                stick.povs.push_back(joystick.povs(j));
              }

              joystick_status.joysticks.push_back(stick);

            }
            joystick_pub.publish(joystick_status);
          }
        }
        break; 
      case str2int("robotstatus"):
        {
          static ck::RobotStatus status;
        }
        break;
      default:
        ROS_INFO("Got unrecognized message: %s", message_group.c_str());
        break;
    }

    
    // bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
    
    // ROS_INFO("Parse result: %d", parse_result);
    // ROS_INFO("State: %s", ck::RobotStatus_RobotState_Name(status.robot_state()).c_str());
    // ROS_INFO("Alliance: %s", ck::RobotStatus_Alliance_Name(status.alliance()).c_str());
    // ROS_INFO("Time: %f", status.match_time());
    // ROS_INFO("Game Data: %s", status.game_data().c_str());

    zmq_msg_close(&message);
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");
  // GOOGLE_PROTOBUF_VERIFY_VERSION;

  ros::NodeHandle n;

  node = &n;

  std::thread rioReceiveThread (robot_receive_loop);

  ros::spin();

  return 0;
}