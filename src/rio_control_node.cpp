
#include "ros/ros.h"
#include "std_msgs/String.h"

#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>

#include "RobotStatus.pb.h"
#include "JoystickStatus.pb.h"
#include "MotorControl.pb.h"
#include "MotorStatus.pb.h"
#include "MotorConfiguration.pb.h"

#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Motor_Status.h>
#include <rio_control_node/Motor_Configuration.h>

// #define ROBOT_CONNECT_STRING "udp://10.1.95.2:5801"
#define ROBOT_CONNECT_STRING "udp://10.1.95.99:5801"	//DISABLE ROBOT DRIVE

void *context;

ros::NodeHandle *node;

constexpr float MOTOR_CONTROL_TIMEOUT = 0.2;

std::mutex motor_config_mutex;

class MotorConfigTracker
{
public:
	rio_control_node::Motor_Config motor;
};

static std::map<int32_t, MotorConfigTracker> motor_config_map;

void motorConfigCallback(const rio_control_node::Motor_Configuration &msg)
{
	std::lock_guard<std::mutex> lock(motor_config_mutex);
	for (int i = 0; i < msg.motors.size(); i++)
	{
		rio_control_node::Motor_Config updated_motor;
		updated_motor = msg.motors[i];

		MotorConfigTracker updated_tracked_motor;
		updated_tracked_motor.motor = updated_motor;

		motor_config_map[msg.motors[i].id] = updated_tracked_motor;
	}
}

void motor_config_transmit_loop()
{
	void *publisher = zmq_socket(context, ZMQ_RADIO);

	int rc = zmq_connect(publisher, ROBOT_CONNECT_STRING);

	if (rc < 0)
	{
		ROS_INFO("Failed to initialize motor publisher");
	}

	char buffer[10000];

	memset(buffer, 0, 10000);

	ros::Rate rate(10);

	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(motor_config_mutex);
			ck::MotorConfiguration motor_config;
			for (std::map<int32_t, MotorConfigTracker>::iterator i = motor_config_map.begin();
				 i != motor_config_map.end();
				 i++)
			{

				ck::MotorConfiguration::Motor *new_motor = motor_config.add_motors();

				new_motor->set_id((*i).second.motor.id);
				new_motor->set_controller_type((ck::MotorConfiguration_Motor_ControllerType)(*i).second.motor.controller_type);
				new_motor->set_controller_mode((ck::MotorConfiguration_Motor_ControllerMode)(*i).second.motor.controller_mode);

				ck::MotorConfiguration_Motor_MotorControllerConfiguration *motor_config_internal = new ck::MotorConfiguration_Motor_MotorControllerConfiguration();

				motor_config_internal->set_kp((*i).second.motor.kP);
				motor_config_internal->set_ki((*i).second.motor.kI);
				motor_config_internal->set_kd((*i).second.motor.kD);
				motor_config_internal->set_kf((*i).second.motor.kF);
				motor_config_internal->set_izone((*i).second.motor.iZone);
				motor_config_internal->set_max_i_accum((*i).second.motor.max_i_accum);
				motor_config_internal->set_allowed_closed_loop_error((*i).second.motor.allowed_closed_loop_error);
				motor_config_internal->set_max_closed_loop_peak_output((*i).second.motor.max_closed_loop_peak_output);
				motor_config_internal->set_motion_cruise_velocity((*i).second.motor.motion_cruise_velocity);
				motor_config_internal->set_motion_acceleration((*i).second.motor.motion_acceleration);
				motor_config_internal->set_motion_s_curve_strength((*i).second.motor.motion_s_curve_strength);
				motor_config_internal->set_forward_soft_limit((*i).second.motor.forward_soft_limit);
				motor_config_internal->set_forward_soft_limit_enable((*i).second.motor.forward_soft_limit_enable);
				motor_config_internal->set_reverse_soft_limit((*i).second.motor.reverse_soft_limit);
				motor_config_internal->set_reverse_soft_limit_enable((*i).second.motor.reverse_soft_limit_enable);
				motor_config_internal->set_feedback_sensor_coefficient((*i).second.motor.feedback_sensor_coefficient);
				motor_config_internal->set_voltage_compensation_saturation((*i).second.motor.voltage_compensation_saturation);
				motor_config_internal->set_voltage_compensation_enabled((*i).second.motor.voltage_compensation_enabled);
				motor_config_internal->set_invert_type((ck::MotorConfiguration_Motor_MotorControllerConfiguration_InvertType)(*i).second.motor.invert_type);
				motor_config_internal->set_sensor_phase_inverted((*i).second.motor.sensor_phase_inverted);
				motor_config_internal->set_neutral_mode((ck::MotorConfiguration_Motor_MotorControllerConfiguration_NeutralMode)(*i).second.motor.neutral_mode);
				motor_config_internal->set_open_loop_ramp((*i).second.motor.open_loop_ramp);
				motor_config_internal->set_closed_loop_ramp((*i).second.motor.closed_loop_ramp);
				ck::MotorConfiguration_Motor_MotorControllerConfiguration_CurrentLimitConfiguration *supply_limit = new ck::MotorConfiguration_Motor_MotorControllerConfiguration_CurrentLimitConfiguration();
				supply_limit->set_enable((*i).second.motor.supply_current_limit_config.enable);
				supply_limit->set_current_limit((*i).second.motor.supply_current_limit_config.current_limit);
				supply_limit->set_trigger_threshold_current((*i).second.motor.supply_current_limit_config.trigger_threshold_current);
				supply_limit->set_trigger_threshold_time((*i).second.motor.supply_current_limit_config.trigger_threshold_time);
				motor_config_internal->set_allocated_supply_current_limit_config(supply_limit);
				ck::MotorConfiguration_Motor_MotorControllerConfiguration_CurrentLimitConfiguration *stator_limit = new ck::MotorConfiguration_Motor_MotorControllerConfiguration_CurrentLimitConfiguration();
				stator_limit->set_enable((*i).second.motor.stator_current_limit_config.enable);
				stator_limit->set_current_limit((*i).second.motor.stator_current_limit_config.current_limit);
				stator_limit->set_trigger_threshold_current((*i).second.motor.stator_current_limit_config.trigger_threshold_current);
				stator_limit->set_trigger_threshold_time((*i).second.motor.stator_current_limit_config.trigger_threshold_time);
				motor_config_internal->set_allocated_supply_current_limit_config(stator_limit);
				new_motor->set_allocated_motor_configuration(motor_config_internal);
			}

			bool serialize_status = motor_config.SerializeToArray(buffer, 10000);

			if (!serialize_status)
			{
				ROS_INFO("Failed to serialize motor status!!");
			}
			else
			{
				zmq_msg_t message;
				zmq_msg_init_size(&message, motor_config.ByteSizeLong());
				memcpy(zmq_msg_data(&message), buffer, motor_config.ByteSizeLong());
				zmq_msg_set_group(&message, "motorconfig");
				zmq_msg_send(&message, publisher, 0);
				zmq_msg_close(&message);
			}
		}

		rate.sleep();
	}
}

std::mutex motor_control_mutex;

class MotorTracker
{
public:
	rio_control_node::Motor motor;
	ros::Time active_time;
};

static std::map<int32_t, MotorTracker> motor_control_map;

void motorControlCallback(const rio_control_node::Motor_Control &msg)
{
	std::lock_guard<std::mutex> lock(motor_control_mutex);
	for (int i = 0; i < msg.motors.size(); i++)
	{
		rio_control_node::Motor updated_motor;
		updated_motor.id = msg.motors[i].id;
		updated_motor.output_value = msg.motors[i].output_value;
		updated_motor.controller_type = msg.motors[i].controller_type;
		updated_motor.control_mode = msg.motors[i].control_mode;
		updated_motor.arbitrary_feedforward = msg.motors[i].arbitrary_feedforward;

		MotorTracker updated_tracked_motor;
		updated_tracked_motor.motor = updated_motor;
		updated_tracked_motor.active_time = ros::Time::now() + ros::Duration(MOTOR_CONTROL_TIMEOUT);

		motor_control_map[msg.motors[i].id] = updated_tracked_motor;
	}
}

void motor_transmit_loop()
{
	void *publisher = zmq_socket(context, ZMQ_RADIO);

	int rc = zmq_connect(publisher, ROBOT_CONNECT_STRING);

	if (rc < 0)
	{
		ROS_INFO("Failed to initialize motor publisher");
	}

	char buffer[10000];

	memset(buffer, 0, 10000);

	ros::Rate rate(100);

	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(motor_control_mutex);
			static ck::MotorControl motor_control;
			motor_control.clear_motors();
			motor_control.Clear();

			std::vector<std::map<int32_t, MotorTracker>::iterator> timed_out_motor_list;

			for (std::map<int32_t, MotorTracker>::iterator i = motor_control_map.begin();
				 i != motor_control_map.end();
				 i++)
			{
				ck::MotorControl::Motor *new_motor = motor_control.add_motors();

				new_motor->set_arbitrary_feedforward((*i).second.motor.arbitrary_feedforward);
				new_motor->set_control_mode((ck::MotorControl_Motor_ControlMode)(*i).second.motor.control_mode);
				new_motor->set_controller_type((ck::MotorControl_Motor_ControllerType)(*i).second.motor.controller_type);
				new_motor->set_id((*i).second.motor.id);
				new_motor->set_output_value((*i).second.motor.output_value);

				if ((*i).second.active_time < ros::Time::now())
				{
					timed_out_motor_list.push_back(i);
				}
			}

			for (std::vector<std::map<int32_t, MotorTracker>::iterator>::iterator i = timed_out_motor_list.begin();
				 i != timed_out_motor_list.end();
				 i++)
			{
				motor_control_map.erase((*i));
			}

			bool serialize_status = motor_control.SerializeToArray(buffer, 10000);

			if (!serialize_status)
			{
				ROS_INFO("Failed to serialize motor status!!");
			}
			else
			{
				zmq_msg_t message;
				zmq_msg_init_size(&message, motor_control.ByteSizeLong());
				memcpy(zmq_msg_data(&message), buffer, motor_control.ByteSizeLong());
				zmq_msg_set_group(&message, "motorcontrol");
				// std::cout << "Sending message..." << std::endl;
				zmq_msg_send(&message, publisher, 0);
				zmq_msg_close(&message);
			}
		}

		rate.sleep();
	}
}

constexpr unsigned int str2int(const char *str, int h = 0)
{
	return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
}

void process_motor_status(zmq_msg_t &message)
{
	static ros::Publisher motor_status_pub = node->advertise<rio_control_node::Motor_Status>("MotorStatus", 1);
	static ck::MotorStatus status;

	void *data = zmq_msg_data(&message);
	bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
	if (parse_result)
	{

		rio_control_node::Motor_Status motor_status;

		for (int i = 0; i < status.motors_size(); i++)
		{
			const ck::MotorStatus::Motor &motor = status.motors(i);
			rio_control_node::Motor_Info motor_info;

			motor_info.id = motor.id();
			motor_info.sensor_position = motor.sensor_position();
			motor_info.sensor_velocity = motor.sensor_velocity();
			motor_info.bus_voltage = motor.bus_voltage();
			motor_info.bus_current = motor.bus_current();
			motor_info.stator_current = motor.stator_current();

			motor_status.motors.push_back(motor_info);
		}
		motor_status_pub.publish(motor_status);
	}
}

void process_joystick_status(zmq_msg_t &message)
{
	static ros::Publisher joystick_pub = node->advertise<rio_control_node::Joystick_Status>("JoystickStatus", 1);
	static ck::JoystickStatus status;

	void *data = zmq_msg_data(&message);
	bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
	if (parse_result)
	{

		rio_control_node::Joystick_Status joystick_status;

		for (int i = 0; i < status.joysticks_size(); i++)
		{
			const ck::JoystickStatus::Joystick &joystick = status.joysticks(i);
			rio_control_node::Joystick stick;

			stick.index = joystick.index();

			uint32_t buttons = joystick.buttons();
			for (int j = 0; j < 32; j++)
			{
				stick.buttons.push_back((buttons >> j) & 0x0001);
			}

			for (int j = 0; j < joystick.axes_size(); j++)
			{
				stick.axes.push_back(joystick.axes(j));
			}

			for (int j = 0; j < joystick.povs_size(); j++)
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

	void *data = zmq_msg_data(&message);
	bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));

	if (parse_result)
	{
		rio_control_node::Robot_Status robot_status;
		robot_status.alliance = status.alliance();
		robot_status.robot_state = status.robot_state();
		robot_status.match_time = status.match_time();
		robot_status.game_data = status.game_data().c_str();
		robot_status_pub.publish(robot_status);
	}
}

void robot_receive_loop()
{
	void *subscriber = zmq_socket(context, ZMQ_DISH);

	int rc = zmq_bind(subscriber, "udp://*:5801");
	rc = zmq_join(subscriber, "robotstatus");
	rc = zmq_join(subscriber, "joystickstatus");
	rc = zmq_join(subscriber, "motorstatus");

	// ck::RobotStatus status;
	char buffer[10000];

	memset(buffer, 0, 10000);

	ROS_INFO("WOOGITY WOOGITY WOO %d %d %d", context, subscriber, rc);

	while (ros::ok())
	{
		zmq_msg_t message;
		zmq_msg_init(&message);
		zmq_msg_recv(&message, subscriber, 0);

		std::string message_group(zmq_msg_group(&message));

		switch (str2int(message_group.c_str()))
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
		case str2int("motorstatus"):
		{
			process_motor_status(message);
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
	ros::init(argc, argv, "rio_control_node");
	// GOOGLE_PROTOBUF_VERIFY_VERSION;

	context = zmq_ctx_new();

	ros::NodeHandle n;

	node = &n;

	std::thread rioReceiveThread(robot_receive_loop);
	std::thread motorSendThread(motor_transmit_loop);
	std::thread motorConfigSendThread(motor_config_transmit_loop);

	ros::Subscriber motorControl = node->subscribe("MotorControl", 100, motorControlCallback);
	ros::Subscriber motorConfig = node->subscribe("MotorConfiguration", 100, motorConfigCallback);

	ros::spin();

	return 0;
}