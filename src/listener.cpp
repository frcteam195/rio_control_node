#include "ros/ros.h"
#include "std_msgs/String.h"

#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include <thread>
#include <string>
#include <RobotStatus.pb.h>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void robot_receive_loop ()
{
  void *context = zmq_ctx_new ();
  void *subscriber = zmq_socket(context, ZMQ_DISH);

  int rc = zmq_bind(subscriber, "udp://*:5801");
  int rc2 = zmq_join(subscriber, "robotstatus");

  // ck::RobotStatus status;
  char buffer [10000];

  memset(buffer, 0, 10000);

  ROS_INFO("WOOGITY WOOGITY test WOO %d %d %d %d", context, subscriber, rc, rc2);

  while (ros::ok())
  {
    zmq_msg_t message;
    zmq_msg_init(&message);
    zmq_msg_recv(&message, subscriber, 0);

    ROS_INFO("Got: %d %s\n", zmq_msg_size(&message), zmq_msg_group(&message));
    void * data = zmq_msg_data(&message);

    ck::RobotStatus status;
    status.ParseFromArray(data, zmq_msg_size(&message));
    
    ROS_INFO("State: %s", ck::RobotStatus_RobotState_Name(status.robot_state()));
    ROS_INFO("Alliance: %s", ck::RobotStatus_Alliance_Name(status.alliance()));
    ROS_INFO("Time: %f", status.match_time());
    ROS_INFO("Game Data: %s", status.game_data());

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

  std::thread rioReceiveThread (robot_receive_loop);

  ros::spin();

  return 0;
}