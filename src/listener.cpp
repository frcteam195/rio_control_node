#include "ros/ros.h"
#include "std_msgs/String.h"

#define ZMQ_BUILD_DRAFT_API

#include "zmq.h"


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
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
  ros::Time::init();

  void *context = zmq_ctx_new ();
  void *subscriber = zmq_socket(context, ZMQ_DISH);
  // zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

  int rc = zmq_bind(subscriber, "udp://*:5801");
  // int rc2;
  int rc2 = zmq_join(subscriber, "robotstatus");

  
  ROS_INFO("WOOGITY WOOGITY test WOO %d %d %d %d", context, subscriber, rc, rc2);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();

    static char buffer [10000];

    int32_t results = zmq_recv(subscriber, &buffer, 10000, ZMQ_NOBLOCK);

    if(results > 0)
    {
      ROS_INFO("WE GOT SOME FUCKING SHIT! %d", results);
    }




    loop_rate.sleep();
  }

  return 0;
}