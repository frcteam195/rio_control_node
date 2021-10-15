
#include "ros/ros.h"
#include "std_msgs/String.h"

#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include <thread>
#include <string>

#include "RobotStatus.pb.h"
#include "JoystickStatus.pb.h"

#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <rio_control_node/Motor_Control.h>

ros::NodeHandle * node;

class MotorTracker 
{
  public:
  rio_control_node::Motor motor;
};

void motorControlCallback(const rio_control_node::Motor_Control& msg)
{
  ROS_INFO("Got some motor control data");
}

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

void process_joystick_status(zmq_msg_t &message)
{
  static ros::Publisher joystick_pub = node->advertise<rio_control_node::Joystick_Status>("JoystickStatus", 1);
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

void process_robot_status(zmq_msg_t &message)
{    
  static ck::RobotStatus status;
  static ros::Publisher robot_status_pub = node->advertise<rio_control_node::Robot_Status>("RobotStatus", 1);

  void * data = zmq_msg_data(&message);
  bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
  
  if (parse_result)
  {
    rio_control_node::Robot_Status robot_status;
    robot_status.alliance = status.alliance();
    robot_status.robot_state = status.robot_state();
    robot_status.match_time = status.match_time();
    robot_status.game_data = status.game_data().c_str();  
    ROS_INFO("Got %d bytes : String %s : C_String %s : Hex %X", status.game_data().size(), status.game_data(), status.game_data().c_str(), status.game_data().c_str());
    robot_status_pub.publish(robot_status);
  }
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
          process_joystick_status(message);
        }
        break; 
      case str2int("robotstatus"):
        {
          process_robot_status(message);
        }
        break;
      default:
        ROS_INFO("Got unrecognized message: %s", message_group.c_str());
        break;
    }



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

  ros::Subscriber motorControl = node->subscribe("MotorControl", 10, motorControlCallback);

  ros::spin();

  return 0;
}