
#include "rio_control_node.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include <atomic>
#include <list>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "RobotStatus.pb.h"
#include "JoystickStatus.pb.h"
#include "SolenoidControl.pb.h"
#include "SolenoidStatus.pb.h"
#include "MotorControl.pb.h"
#include "MotorStatus.pb.h"
#include "MotorConfiguration.pb.h"
#include "IMUConfig.pb.h"
#include "IMUData.pb.h"
#include "EncoderConfig.pb.h"
#include "EncoderData.pb.h"
#include "LEDControl.pb.h"
#include <signal.h>

#include <nav_msgs/Odometry.h>
#include <ck_ros_base_msgs_node/Joystick_Status.h>
#include <ck_ros_base_msgs_node/Robot_Status.h>
#include <ck_ros_base_msgs_node/Motor_Control.h>
#include <ck_ros_base_msgs_node/Motor_Status.h>
#include <ck_ros_base_msgs_node/IMU_Data.h>
#include <ck_ros_base_msgs_node/IMU_Sensor_Data.h>
#include <ck_ros_base_msgs_node/Encoder_Data.h>
#include <ck_ros_base_msgs_node/Encoder_Sensor_Data.h>
#include <ck_ros_base_msgs_node/Encoder_Config.h>
#include <ck_ros_base_msgs_node/Encoder_Configuration.h>
#include <ck_ros_base_msgs_node/Motor_Configuration.h>
#include <ck_ros_base_msgs_node/Cal_Override_Mode.h>
#include <ck_ros_base_msgs_node/Solenoid_Control.h>
#include <ck_ros_base_msgs_node/Solenoid_Status.h>
#include <ck_ros_base_msgs_node/LED_Animation.h>
#include <ck_ros_base_msgs_node/LED_Color.h>
#include <ck_ros_base_msgs_node/LED_Control.h>
#include <ck_ros_base_msgs_node/LED_Control_Data.h>

#include <network_tables_node/NTSetBool.h>
#include <network_tables_node/NTSetDouble.h>
#include <network_tables_node/NTSetString.h>

#include <math.h>

#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>

#define ROBOT_CONNECT_STRING "udp://10.1.95.2:5801"
// #define ROBOT_CONNECT_STRING "udp://10.1.95.99:5801"	//DISABLE ROBOT DRIVE

#define STR_PARAM(s) #s
#define CKSP(s) ckgp(STR_PARAM(s))
std::string ckgp(std::string instr)
{
	std::string retVal = ros::this_node::getName();
	retVal += "/" + instr;
	return retVal;
}

ros::ServiceClient nt_setbool_client;
ros::ServiceClient nt_setdouble_client;
ros::ServiceClient nt_setstring_client;

void *context;
std::atomic<bool> sigintCalled;

ros::NodeHandle *node;

constexpr float MOTOR_CONTROL_TIMEOUT = 1.0;

std::mutex override_mode_mutex;
std::mutex motor_config_mutex;

class MotorConfigTracker
{
public:
	ck_ros_base_msgs_node::Motor_Config motor;
};

static std::map<int32_t, MotorConfigTracker> motor_config_map;

static OverrideModeStruct overrideModeS = NORMAL_OPERATION_MODE;

static std::vector<float> gear_ratio_to_output_shaft;
static std::vector<float> motor_ticks_per_revolution;
static std::vector<float> motor_ticks_velocity_sample_window;

static bool compressor_enabled_in_auto = false;

static std::vector<int> motor_ids_to_override_encoder;
static std::vector<int> remote_encoder_ids;
static std::vector<double> rotations_offset_to_zero;
static std::map<int, int> motor_remote_encoder_mappings;
static std::map<int, double> motor_remote_encoder_offsets;

ros::ServiceClient &getNTSetBoolSrv()
{
	if (!nt_setbool_client)
	{
		nt_setbool_client = node->serviceClient<network_tables_node::NTSetBool>("nt_setbool", true);
	}
	return nt_setbool_client;
}

ros::ServiceClient &getNTSetDoubleSrv()
{
	if (!nt_setdouble_client)
	{
		nt_setdouble_client = node->serviceClient<network_tables_node::NTSetDouble>("nt_setdouble", true);
	}
	return nt_setdouble_client;
}

ros::ServiceClient &getNTSetStringSrv()
{
	if (!nt_setstring_client)
	{
		nt_setstring_client = node->serviceClient<network_tables_node::NTSetString>("nt_setstring", true);
	}
	return nt_setstring_client;
}

void load_config_params()
{
	bool received_data = false;
	received_data = node->getParam(CKSP(gear_ratio_to_output_shaft), gear_ratio_to_output_shaft);
	if (!received_data)
	{
		ROS_ERROR("COULD NOT LOAD GEAR RATIOS, using 1.0");
		for (uint32_t i = 0;
			 i < 20;
			 i++)
		{
			gear_ratio_to_output_shaft.push_back(1.0);
		}
	}

	received_data = node->getParam(CKSP(motor_ticks_per_revolution), motor_ticks_per_revolution);
	if (!received_data)
	{
		ROS_ERROR("COULD NOT LOAD ENCODER TICK COUNT, using 2048");
		for (uint32_t i = 0;
			 i < 20;
			 i++)
		{
			motor_ticks_per_revolution.push_back(2048.0);
		}
	}

	received_data = node->getParam(CKSP(motor_ticks_velocity_sample_window), motor_ticks_velocity_sample_window);
	if (!received_data)
	{
		ROS_ERROR("COULD NOT LOAD ENCODER SAMPLE WINDOW, using 0.1");
		for (uint32_t i = 0;
			 i < 20;
			 i++)
		{
			motor_ticks_velocity_sample_window.push_back(0.1);
		}
	}

	received_data = node->getParam(CKSP(motor_ids_to_override_encoder), motor_ids_to_override_encoder) &&
					node->getParam(CKSP(remote_encoder_ids), remote_encoder_ids) &&
					node->getParam(CKSP(rotations_offset_to_zero), rotations_offset_to_zero);
	if (!received_data)
	{
		ROS_ERROR("COULD NOT LOAD REMOTE ENCODER IDS");
	}
	else if (motor_ids_to_override_encoder.size() != remote_encoder_ids.size() || motor_ids_to_override_encoder.size() != rotations_offset_to_zero.size())
	{
		ROS_ERROR("REMOTE ENCODER CONFIGURATION ERROR m: %ld, r: %ld, o: %ld", motor_ids_to_override_encoder.size(), remote_encoder_ids.size(), rotations_offset_to_zero.size());
	}
	else
	{
		for (size_t i = 0; i < motor_ids_to_override_encoder.size(); i++)
		{
			motor_remote_encoder_mappings[motor_ids_to_override_encoder[i]] = remote_encoder_ids[i];
			motor_remote_encoder_offsets[motor_ids_to_override_encoder[i]] = rotations_offset_to_zero[i];
		}
	}

	received_data = node->getParam(CKSP(compressor_enabled_in_auto), compressor_enabled_in_auto);
	if (!received_data)
	{
		ROS_ERROR("COULD NOT COMPRESSOR IN AUTO PARAM. DEFAULTING TO FALSE.");
		compressor_enabled_in_auto = false;
	}
}

void modeOverrideCallback(const ck_ros_base_msgs_node::Cal_Override_Mode &msg)
{
	std::lock_guard<std::mutex> lock(override_mode_mutex);
	overrideModeS.overrideMode = (OVERRIDE_MODE)msg.operation_mode;
	overrideModeS.heartbeatTimeout = DEFAULT_OVERRIDE_TIMEOUT;
}

void process_override_heartbeat_thread()
{
	ros::Rate rate(OVERRIDE_HEARTBEAT_RATE);

	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(override_mode_mutex);
			overrideModeS.heartbeatTimeout = std::fmax(--overrideModeS.heartbeatTimeout, 0);
			if (overrideModeS.heartbeatTimeout <= 0)
			{
				overrideModeS.overrideMode = OVERRIDE_MODE::NORMAL_OPERATION;
			}
		}
		rate.sleep();
	}
}

double convertNativeUnitsToPosition(double nativeUnits, int motorID)
{
	return nativeUnits / motor_ticks_per_revolution[motorID - 1] / gear_ratio_to_output_shaft[motorID - 1];
}

double convertNativeUnitsToVelocity(double nativeUnits, int motorID)
{
	return (nativeUnits / motor_ticks_per_revolution[motorID - 1] / gear_ratio_to_output_shaft[motorID - 1] / motor_ticks_velocity_sample_window[motorID - 1]) * 60.0;
}

void processMotorConfigMsg(const ck_ros_base_msgs_node::Motor_Configuration &msg)
{
	std::lock_guard<std::mutex> lock(motor_config_mutex);
	for (size_t i = 0; i < msg.motors.size(); i++)
	{
		ck_ros_base_msgs_node::Motor_Config updated_motor;
		updated_motor = msg.motors[i];

		MotorConfigTracker updated_tracked_motor;
		updated_tracked_motor.motor = updated_motor;

		motor_config_map[msg.motors[i].id] = updated_tracked_motor;
	}
}

void motorTuningConfigCallback(const ck_ros_base_msgs_node::Motor_Configuration &msg)
{
	if (overrideModeS.overrideMode == OVERRIDE_MODE::TUNING_PIDS)
	{
		processMotorConfigMsg(msg);
	}
}

void motorConfigCallback(const ck_ros_base_msgs_node::Motor_Configuration &msg)
{
	if (overrideModeS.overrideMode == OVERRIDE_MODE::NORMAL_OPERATION)
	{
		processMotorConfigMsg(msg);
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
				new_motor->set_kp((*i).second.motor.kP);
				new_motor->set_ki((*i).second.motor.kI);
				new_motor->set_kd((*i).second.motor.kD);
				new_motor->set_kf((*i).second.motor.kF);
				new_motor->set_izone((*i).second.motor.iZone);
				new_motor->set_max_i_accum((*i).second.motor.max_i_accum);
				new_motor->set_allowed_closed_loop_error((*i).second.motor.allowed_closed_loop_error);
				new_motor->set_max_closed_loop_peak_output((*i).second.motor.max_closed_loop_peak_output);
				new_motor->set_motion_cruise_velocity((*i).second.motor.motion_cruise_velocity);
				new_motor->set_motion_acceleration((*i).second.motor.motion_acceleration);
				new_motor->set_motion_s_curve_strength((*i).second.motor.motion_s_curve_strength);
				new_motor->set_forward_soft_limit((*i).second.motor.forward_soft_limit *
												  gear_ratio_to_output_shaft[(*i).second.motor.id - 1] *
												  motor_ticks_per_revolution[(*i).second.motor.id - 1]);
				new_motor->set_forward_soft_limit_enable((*i).second.motor.forward_soft_limit_enable);
				new_motor->set_reverse_soft_limit((*i).second.motor.reverse_soft_limit *
												  gear_ratio_to_output_shaft[(*i).second.motor.id - 1] *
												  motor_ticks_per_revolution[(*i).second.motor.id - 1]);
				new_motor->set_reverse_soft_limit_enable((*i).second.motor.reverse_soft_limit_enable);
				new_motor->set_feedback_sensor_coefficient((*i).second.motor.feedback_sensor_coefficient);
				new_motor->set_voltage_compensation_saturation((*i).second.motor.voltage_compensation_saturation);
				new_motor->set_voltage_compensation_enabled((*i).second.motor.voltage_compensation_enabled);
				new_motor->set_invert_type((ck::MotorConfiguration_Motor_InvertType)(*i).second.motor.invert_type);
				new_motor->set_sensor_phase_inverted((*i).second.motor.sensor_phase_inverted);
				new_motor->set_neutral_mode((ck::MotorConfiguration_Motor_NeutralMode)(*i).second.motor.neutral_mode);
				new_motor->set_open_loop_ramp((*i).second.motor.open_loop_ramp);
				new_motor->set_closed_loop_ramp((*i).second.motor.closed_loop_ramp);
				ck::MotorConfiguration_Motor_CurrentLimitConfiguration *supply_limit = new ck::MotorConfiguration_Motor_CurrentLimitConfiguration();
				supply_limit->set_enable((*i).second.motor.supply_current_limit_config.enable);
				supply_limit->set_current_limit((*i).second.motor.supply_current_limit_config.current_limit);
				supply_limit->set_trigger_threshold_current((*i).second.motor.supply_current_limit_config.trigger_threshold_current);
				supply_limit->set_trigger_threshold_time((*i).second.motor.supply_current_limit_config.trigger_threshold_time);
				new_motor->set_allocated_supply_current_limit_config(supply_limit);
				ck::MotorConfiguration_Motor_CurrentLimitConfiguration *stator_limit = new ck::MotorConfiguration_Motor_CurrentLimitConfiguration();
				stator_limit->set_enable((*i).second.motor.stator_current_limit_config.enable);
				stator_limit->set_current_limit((*i).second.motor.stator_current_limit_config.current_limit);
				stator_limit->set_trigger_threshold_current((*i).second.motor.stator_current_limit_config.trigger_threshold_current);
				stator_limit->set_trigger_threshold_time((*i).second.motor.stator_current_limit_config.trigger_threshold_time);
				new_motor->set_allocated_stator_current_limit_config(stator_limit);
				new_motor->set_forward_limit_switch_source((ck::MotorConfiguration::Motor::LimitSwitchSource)((*i).second.motor.forward_limit_switch_source));
				new_motor->set_forward_limit_switch_normal((ck::MotorConfiguration::Motor::LimitSwitchNormal)((*i).second.motor.forward_limit_switch_normal));
				new_motor->set_reverse_limit_switch_source((ck::MotorConfiguration::Motor::LimitSwitchSource)((*i).second.motor.reverse_limit_switch_source));
				new_motor->set_reverse_limit_switch_normal((ck::MotorConfiguration::Motor::LimitSwitchNormal)((*i).second.motor.reverse_limit_switch_normal));
				new_motor->set_peak_output_forward((*i).second.motor.peak_output_forward);
				new_motor->set_peak_output_reverse((*i).second.motor.peak_output_reverse);
				new_motor->set_can_network(ck::CANNetwork::RIO_CANIVORE);
				if (motor_remote_encoder_mappings.count((*i).second.motor.id))
				{
					new_motor->set_feedback_sensor_can_id(motor_remote_encoder_mappings[(*i).second.motor.id]);
				}
				new_motor->set_active_gain_slot((*i).second.motor.active_gain_slot);
				new_motor->set_kp_1((*i).second.motor.kP_1);
				new_motor->set_ki_1((*i).second.motor.kI_1);
				new_motor->set_kd_1((*i).second.motor.kD_1);
				new_motor->set_kf_1((*i).second.motor.kF_1);
			}

			bool serialize_status = motor_config.SerializeToArray(buffer, 10000);

			if (!serialize_status)
			{
				ROS_INFO("Failed to serialize motor status!!");
			}
			else
			{
				zmq_msg_t message;
				// ROS_INFO("Limit Size is : %d", (int)motor_config.ByteSizeLong());
				zmq_msg_init_size(&message, motor_config.ByteSizeLong());
				memcpy(zmq_msg_data(&message), buffer, motor_config.ByteSizeLong());
				zmq_msg_set_group(&message, "motorconfig");
				zmq_msg_send(&message, publisher, 0);
				zmq_msg_close(&message);
			}
		}
		{
			std::lock_guard<std::mutex> lock(motor_config_mutex);
			ck_ros_base_msgs_node::Motor_Configuration overall_motor_config;
			for (std::map<int32_t, MotorConfigTracker>::iterator i = motor_config_map.begin();
				 i != motor_config_map.end();
				 i++)
			{
				overall_motor_config.motors.push_back((*i).second.motor);
			}
			static ros::Publisher overall_config_publisher = node->advertise<ck_ros_base_msgs_node::Motor_Configuration>("/MotorConfigurationFinal", 10);
			overall_config_publisher.publish(overall_motor_config);
		}

		rate.sleep();
	}
}

std::mutex solenoid_control_mutex;
class SolenoidTracker
{
public:
	ck_ros_base_msgs_node::Solenoid solenoid;
	ros::Time active_time;
};

static std::map<int32_t, SolenoidTracker> solenoid_control_map;

void solenoidControlCallback(const ck_ros_base_msgs_node::Solenoid_Control &msg)
{
	std::lock_guard<std::mutex> lock(solenoid_control_mutex);
	for (size_t i = 0; i < msg.solenoids.size(); i++)
	{
		ck_ros_base_msgs_node::Solenoid updated_solenoid;
		updated_solenoid.id = msg.solenoids[i].id;
		updated_solenoid.output_value = msg.solenoids[i].output_value;
		updated_solenoid.module_type = msg.solenoids[i].module_type;
		updated_solenoid.solenoid_type = msg.solenoids[i].solenoid_type;

		SolenoidTracker updated_tracked_solenoid;
		updated_tracked_solenoid.solenoid = updated_solenoid;
		updated_tracked_solenoid.active_time = ros::Time::now() + ros::Duration(MOTOR_CONTROL_TIMEOUT);

		solenoid_control_map[msg.solenoids[i].id] = updated_tracked_solenoid;
	}
}

void solenoid_transmit_loop()
{
	void *publisher = zmq_socket(context, ZMQ_RADIO);

	int rc = zmq_connect(publisher, ROBOT_CONNECT_STRING);

	if (rc < 0)
	{
		ROS_INFO("Failed to initialize solenoid publisher");
	}

	char buffer[10000];

	memset(buffer, 0, 10000);

	ros::Rate rate(100);

	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(solenoid_control_mutex);
			static ck::SolenoidControl solenoid_control;
			solenoid_control.clear_solenoids();
			solenoid_control.Clear();
			solenoid_control.set_compressor_is_enabled_for_auto(compressor_enabled_in_auto);

			std::vector<std::map<int32_t, SolenoidTracker>::iterator> timed_out_solenoid_list;

			for (std::map<int32_t, SolenoidTracker>::iterator i = solenoid_control_map.begin();
				 i != solenoid_control_map.end();
				 i++)
			{
				ck::SolenoidControl::Solenoid *new_solenoid = solenoid_control.add_solenoids();

				new_solenoid->set_id((*i).second.solenoid.id);
				new_solenoid->set_module_type((ck::SolenoidControl::Solenoid::ModuleType)(*i).second.solenoid.module_type);
				new_solenoid->set_solenoid_type((ck::SolenoidControl::Solenoid::SolenoidType)(*i).second.solenoid.solenoid_type);
				new_solenoid->set_output_value((ck::SolenoidControl::Solenoid::SolenoidValue)((*i).second.solenoid.output_value));

				if ((*i).second.active_time < ros::Time::now())
				{
					timed_out_solenoid_list.push_back(i);
				}
			}

			for (std::vector<std::map<int32_t, SolenoidTracker>::iterator>::iterator i = timed_out_solenoid_list.begin();
				 i != timed_out_solenoid_list.end();
				 i++)
			{
				solenoid_control_map.erase((*i));
			}

			bool serialize_status = solenoid_control.SerializeToArray(buffer, 10000);

			if (!serialize_status)
			{
				ROS_INFO("Failed to serialize solenoid status!!");
			}
			else
			{
				zmq_msg_t message;
				zmq_msg_init_size(&message, solenoid_control.ByteSizeLong());
				memcpy(zmq_msg_data(&message), buffer, solenoid_control.ByteSizeLong());
				zmq_msg_set_group(&message, "solenoidcontrol");
				// std::cout << "Sending message..." << std::endl;
				zmq_msg_send(&message, publisher, 0);
				zmq_msg_close(&message);
			}
		}

		rate.sleep();
	}
}

void process_solenoid_status(zmq_msg_t &message)
{
	static ros::Publisher solenoid_status_pub = node->advertise<ck_ros_base_msgs_node::Solenoid_Status>("SolenoidStatus", 1);
	static ck::SolenoidStatus status;

	void *data = zmq_msg_data(&message);
	bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
	if (parse_result)
	{

		ck_ros_base_msgs_node::Solenoid_Status solenoid_status;

		for (int i = 0; i < status.solenoids_size(); i++)
		{
			const ck::SolenoidStatus::Solenoid &solenoid = status.solenoids(i);
			ck_ros_base_msgs_node::Solenoid_Info solenoid_info;

			solenoid_info.id = solenoid.id();
			solenoid_info.solenoid_value = solenoid.solenoid_value();

			solenoid_status.solenoids.push_back(solenoid_info);
		}
		solenoid_status_pub.publish(solenoid_status);
	}
}

std::mutex motor_control_mutex;

class MotorTracker
{
public:
	ck_ros_base_msgs_node::Motor motor;
	ros::Time active_time;
};

static std::map<int32_t, MotorTracker> motor_control_map;

void processMotorControlMsg(const ck_ros_base_msgs_node::Motor_Control &msg)
{
	std::lock_guard<std::mutex> lock(motor_control_mutex);
	for (size_t i = 0; i < msg.motors.size(); i++)
	{
		ck_ros_base_msgs_node::Motor updated_motor;
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

void motorTuningControlCallback(const ck_ros_base_msgs_node::Motor_Control &msg)
{
	if (overrideModeS.overrideMode == OVERRIDE_MODE::TUNING_PIDS)
	{
		processMotorControlMsg(msg);
	}
}

void motorControlCallback(const ck_ros_base_msgs_node::Motor_Control &msg)
{
	if (overrideModeS.overrideMode == OVERRIDE_MODE::NORMAL_OPERATION)
	{
		processMotorControlMsg(msg);
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

				if ((*i).second.motor.control_mode == ck_ros_base_msgs_node::Motor::MOTION_MAGIC ||
					(*i).second.motor.control_mode == ck_ros_base_msgs_node::Motor::POSITION)
				{
					float offset_val = 0;
					if (motor_remote_encoder_offsets.count((*i).second.motor.id))
					{
						offset_val = motor_remote_encoder_offsets[(*i).second.motor.id];
					}
					new_motor->set_output_value(((*i).second.motor.output_value + offset_val) *
												gear_ratio_to_output_shaft[(*i).second.motor.id - 1] *
												motor_ticks_per_revolution[(*i).second.motor.id - 1]);
				}
				else if ((*i).second.motor.control_mode == ck_ros_base_msgs_node::Motor::VELOCITY)
				{
					new_motor->set_output_value((*i).second.motor.output_value *
												gear_ratio_to_output_shaft[(*i).second.motor.id - 1] *
												motor_ticks_per_revolution[(*i).second.motor.id - 1] /
												60.0 *
												motor_ticks_velocity_sample_window[(*i).second.motor.id]);
				}
				else
				{
					new_motor->set_output_value((*i).second.motor.output_value);
				}

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
		{
			std::lock_guard<std::mutex> lock(motor_control_mutex);
			ck_ros_base_msgs_node::Motor_Control overall_motor_control;
			for (std::map<int32_t, MotorTracker>::iterator i = motor_control_map.begin();
				 i != motor_control_map.end();
				 i++)
			{
				overall_motor_control.motors.push_back((*i).second.motor);
			}
			static ros::Publisher overall_control_publisher = node->advertise<ck_ros_base_msgs_node::Motor_Control>("/MotorControlFinal", 10);
			overall_control_publisher.publish(overall_motor_control);
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
	static ros::Publisher motor_status_pub = node->advertise<ck_ros_base_msgs_node::Motor_Status>("MotorStatus", 1);
	static ck::MotorStatus status;

	void *data = zmq_msg_data(&message);
	bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
	if (parse_result)
	{

		ck_ros_base_msgs_node::Motor_Status motor_status;

		for (int i = 0; i < status.motors_size(); i++)
		{
			const ck::MotorStatus::Motor &motor = status.motors(i);
			ck_ros_base_msgs_node::Motor_Info motor_info;

			motor_info.id = motor.id();
			float offset_val = 0;
			if (motor_remote_encoder_offsets.count(motor.id()))
			{
				offset_val = motor_remote_encoder_offsets[motor.id()];
			}
			motor_info.sensor_position = convertNativeUnitsToPosition(motor.sensor_position(), motor.id()) - offset_val;
			motor_info.sensor_velocity = convertNativeUnitsToVelocity(motor.sensor_velocity(), motor.id());
			motor_info.bus_voltage = motor.bus_voltage();
			motor_info.bus_current = motor.bus_current();
			motor_info.stator_current = motor.stator_current();
			motor_info.forward_limit_closed = motor.forward_limit_closed();
			motor_info.reverse_limit_closed = motor.reverse_limit_closed();
			motor_info.control_mode = (int8_t)motor.control_mode();
			motor_info.active_trajectory_arbff = motor.active_trajectory_arbff();
			motor_info.active_trajectory_position = motor.active_trajectory_position();
			motor_info.active_trajectory_velocity = motor.active_trajectory_velocity();
			motor_info.raw_closed_loop_error = motor.raw_closed_loop_error();
			motor_info.raw_error_derivative = motor.raw_error_derivative();
			motor_info.raw_integral_accum = motor.raw_integral_accum();
			motor_info.raw_output_percent = motor.raw_output_percent();
			motor_info.faults = motor.faults();
			motor_info.sticky_faults = motor.sticky_faults();
			switch (motor_info.control_mode)
			{
			case ck_ros_base_msgs_node::Motor_Info::POSITION:
			case ck_ros_base_msgs_node::Motor_Info::MOTION_MAGIC:
			case ck_ros_base_msgs_node::Motor_Info::MOTION_PROFILE:
			case ck_ros_base_msgs_node::Motor_Info::MOTION_PROFILE_ARC:
			{
				motor_info.commanded_output = convertNativeUnitsToPosition(motor.commanded_output(), motor.id());
			}
			break;
			case ck_ros_base_msgs_node::Motor_Info::VELOCITY:
			{
				motor_info.commanded_output = convertNativeUnitsToVelocity(motor.commanded_output(), motor.id());
			}
			break;
			case ck_ros_base_msgs_node::Motor_Info::CURRENT:
			case ck_ros_base_msgs_node::Motor_Info::FOLLOWER:
			case ck_ros_base_msgs_node::Motor_Info::PERCENT_OUTPUT:
			case ck_ros_base_msgs_node::Motor_Info::MUSIC_TONE:
			case ck_ros_base_msgs_node::Motor_Info::DISABLED:
			{
				motor_info.commanded_output = motor.commanded_output();
			}
			break;
			}

			motor_status.motors.push_back(motor_info);
		}
		motor_status_pub.publish(motor_status);
	}
}

void process_joystick_status(zmq_msg_t &message)
{
	static ros::Publisher joystick_pub = node->advertise<ck_ros_base_msgs_node::Joystick_Status>("JoystickStatus", 1);
	static ck::JoystickStatus status;

	void *data = zmq_msg_data(&message);
	bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));
	if (parse_result)
	{

		ck_ros_base_msgs_node::Joystick_Status joystick_status;

		for (int i = 0; i < status.joysticks_size(); i++)
		{
			const ck::JoystickStatus::Joystick &joystick = status.joysticks(i);
			ck_ros_base_msgs_node::Joystick stick;

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
	static ros::Publisher robot_status_pub = node->advertise<ck_ros_base_msgs_node::Robot_Status>("RobotStatus", 1);

	void *data = zmq_msg_data(&message);
	bool parse_result = status.ParseFromArray(data, zmq_msg_size(&message));

	if (parse_result)
	{
		ck_ros_base_msgs_node::Robot_Status robot_status;
		robot_status.alliance = status.alliance();
		robot_status.robot_state = status.robot_state();
		robot_status.match_time = status.match_time();
		robot_status.game_data = status.game_data().c_str();
		robot_status.selected_auto = status.selected_auto();
		robot_status.is_connected = status.is_connected();
		robot_status_pub.publish(robot_status);

		int minutes = floor(robot_status.match_time / 60.0);
		int seconds = floor(fmod(robot_status.match_time, 60.0));
		std::stringstream output;
		output << minutes << ":" << seconds;

		ros::ServiceClient &nt_setstring_localclient = getNTSetStringSrv();
		if (nt_setstring_localclient)
		{
			network_tables_node::NTSetString ntmsg;
			ntmsg.request.table_name = "dashboard_data";
			ntmsg.request.entry_name = "match_time";
			ntmsg.request.value = output.str();
			nt_setstring_localclient.call(ntmsg);
		}
	}
}

// wrap x -> [0,max)
template <typename T>
inline T wrapMax(T x, T max)
{
	/* integer math: (max + x % max) % max */
	return std::fmod(max + std::fmod(x, max), max);
}

// wrap x -> [min,max)
template <typename T>
inline T wrapMinMax(T x, T min, T max)
{
	return min + wrapMax(x - min, max - min);
}

template <typename T>
inline T normalize_to_2_pi(T value)
{
	return wrapMinMax<T>(value, 0, (2.0 * M_PI));
}

double smallest_traversal(double angle, double target_angle)
{
    double left = -normalize_to_2_pi(angle - target_angle);
    double right = normalize_to_2_pi(target_angle - angle);
    if(fabs(left) < fabs(right))
    {
        return left;
    }
    return right;
}

void process_imu_data(zmq_msg_t &message)
{
	static ck::IMUData imuData;

	void *data = zmq_msg_data(&message);
	bool parse_result = imuData.ParseFromArray(data, zmq_msg_size(&message));


	if (parse_result)
	{
        const ck::IMUData::IMUSensorData &imuSensorData = imuData.imu_sensor(0);

        static float last_absolute_yaw = 0;
        if(last_absolute_yaw != imuSensorData.x())
        {
            nav_msgs::Odometry odometry_data;
            odometry_data.header.stamp = ros::Time::now();
            odometry_data.header.frame_id = "odom";
            odometry_data.child_frame_id = "base_link";

            geometry::Pose empty_pose;
            odometry_data.pose.pose = geometry::to_msg(empty_pose);

            geometry::Twist twist;
            twist.angular.pitch(ck::math::deg2rad(imuSensorData.x_rps()));
            twist.angular.roll(ck::math::deg2rad(imuSensorData.y_rps()));
            twist.angular.yaw(ck::math::deg2rad(imuSensorData.z_rps()));
            odometry_data.twist.twist = geometry::to_msg(twist);

            geometry::Covariance covariance;
            covariance.yaw_var(ck::math::deg2rad(3.0));

            odometry_data.pose.covariance = geometry::to_msg(covariance);

            geometry::Pose imu_orientation;
            geometry_msgs::Quaternion quat;
            quat.w = imuSensorData.w();
            quat.x = imuSensorData.x();
            quat.y = imuSensorData.y();
            quat.z = imuSensorData.z();

            geometry::Rotation rotation = geometry::to_rotation(quat);
			float roll = rotation.pitch();
			float pitch = -rotation.roll();
			rotation.pitch(pitch);
			rotation.roll(roll);
            imu_orientation.orientation = rotation;
			odometry_data.pose.pose = geometry::to_msg(imu_orientation);

	        static ros::Publisher imu_pose_data_pub = node->advertise<nav_msgs::Odometry>("/RobotIMU", 1);
            imu_pose_data_pub.publish(odometry_data);

            last_absolute_yaw = imuSensorData.x();
        }
	}
}

void process_encoder_data(zmq_msg_t &message)
{
	static ck::EncoderData zmqEncoderData;
	static ros::Publisher encoder_data_pub = node->advertise<nav_msgs::Odometry>("/EncoderData", 1);

	void *data = zmq_msg_data(&message);
	bool parse_result = zmqEncoderData.ParseFromArray(data, zmq_msg_size(&message));

	if (parse_result)
	{
		ck_ros_base_msgs_node::Encoder_Data encoder_data;

		for (int i = 0; i < zmqEncoderData.encoder_sensor_size(); i++)
		{
			const ck::EncoderData::EncoderSensorData &zmqEncoderSensorData = zmqEncoderData.encoder_sensor(i);
			ck_ros_base_msgs_node::Encoder_Sensor_Data encoder_sensor_data;

			encoder_sensor_data.id = zmqEncoderSensorData.id();
			encoder_sensor_data.absolute_position = zmqEncoderSensorData.sensor_absolute_position();
			encoder_sensor_data.relative_position = zmqEncoderSensorData.sensor_relative_position();
			encoder_sensor_data.velocity = zmqEncoderSensorData.sensor_velocity();
			encoder_sensor_data.faulted = zmqEncoderSensorData.is_faulted();

			encoder_data.encoderData.push_back(encoder_sensor_data);
		}
		encoder_data_pub.publish(encoder_data);
	}
}

void robot_receive_loop()
{
	void *subscriber = zmq_socket(context, ZMQ_DISH);

	int rc = zmq_bind(subscriber, "udp://*:5801");
	rc = zmq_join(subscriber, "robotstatus");
	rc = zmq_join(subscriber, "imudata");
	// rc = zmq_join(subscriber, "encoderdata");
	rc = zmq_join(subscriber, "joystickstatus");
	rc = zmq_join(subscriber, "motorstatus");

	// ck::RobotStatus status;
	char buffer[10000];

	memset(buffer, 0, 10000);

	ROS_INFO("WOOGITY WOOGITY WOO %lx %lx %d", (uint64_t)context, (uint64_t)subscriber, rc);

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
		case str2int("imudata"):
		{
			process_imu_data(message);
		}
		break;
		case str2int("encoderdata"):
		{
			process_encoder_data(message);
		}
		break;
		case str2int("solenoidstatus"):
		{
			process_solenoid_status(message);
		}
		break;
		default:
			ROS_INFO("Got unrecognized message: %s", message_group.c_str());
			break;
		}

		zmq_msg_close(&message);
	}
}

void sigint_handler(int sig)
{
	(void)sig;
	sigintCalled = true;
	ros::shutdown();
}

void imu_config_thread()
{
	void *publisher = zmq_socket(context, ZMQ_RADIO);

	int rc = zmq_connect(publisher, ROBOT_CONNECT_STRING);

	if (rc < 0)
	{
		ROS_INFO("Failed to initialize imu config publisher");
	}

	char buffer[10000];

	memset(buffer, 0, 10000);

	ros::Rate rate(10);

	while (ros::ok())
	{
		ck::IMUConfig imu_config;

		ck::IMUConfig_IMUConfigData *imu_1 = imu_config.add_imu_config();
		imu_1->set_id(0);
		imu_1->set_imu_type(ck::IMUConfig::IMUConfigData::IMUType::IMUConfig_IMUConfigData_IMUType_PIGEON2);
		imu_1->set_mount_pose_axis_forward(ck::IMUConfig::IMUConfigData::AxisDirection::IMUConfig_IMUConfigData_AxisDirection_PositiveX);
		imu_1->set_mount_pose_axis_up(ck::IMUConfig::IMUConfigData::AxisDirection::IMUConfig_IMUConfigData_AxisDirection_PositiveZ);
		imu_1->set_can_network(ck::CANNetwork::RIO_CANIVORE);

		bool serialize_status = imu_config.SerializeToArray(buffer, 10000);

		if (!serialize_status)
		{
			ROS_INFO("Failed to serialize imu config!!");
		}
		else
		{
			zmq_msg_t message;
			zmq_msg_init_size(&message, imu_config.ByteSizeLong());
			memcpy(zmq_msg_data(&message), buffer, imu_config.ByteSizeLong());
			zmq_msg_set_group(&message, "imuconfig");
			// std::cout << "Sending message..." << std::endl;
			zmq_msg_send(&message, publisher, 0);
			zmq_msg_close(&message);
		}

		rate.sleep();
	}
}

std::mutex led_control_mutex;
class LEDTracker
{
public:
	ck_ros_base_msgs_node::LED_Control_Data led;
	ros::Time active_time;
};

static std::map<int32_t, LEDTracker> led_control_map;

void ledControlCallback(const ck_ros_base_msgs_node::LED_Control &msg)
{
	std::lock_guard<std::mutex> lock(led_control_mutex);
	for (size_t i = 0; i < msg.led_control.size(); i++)
	{
		LEDTracker updated_tracked_led;
		updated_tracked_led.led = msg.led_control[i];
		updated_tracked_led.active_time = ros::Time::now() + ros::Duration(MOTOR_CONTROL_TIMEOUT);

		led_control_map[msg.led_control[i].id] = updated_tracked_led;
	}
}

void led_transmit_loop(void)
{
	void *publisher = zmq_socket(context, ZMQ_RADIO);

	int rc = zmq_connect(publisher, ROBOT_CONNECT_STRING);

	if (rc < 0)
	{
		ROS_INFO("Failed to initialize led publisher");
	}

	char buffer[10000];

	memset(buffer, 0, 10000);

	ros::Rate rate(30);

	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(led_control_mutex);
			static ck::LEDControl led_control;
			led_control.clear_led_control();
			led_control.Clear();

			std::vector<std::map<int32_t, LEDTracker>::iterator> timed_out_led_list;

			for (std::map<int32_t, LEDTracker>::iterator i = led_control_map.begin();
				 i != led_control_map.end();
				 i++)
			{
				ck::LEDControl::LEDControlData *new_led = led_control.add_led_control();

				new_led->set_id((*i).second.led.id);
				new_led->set_can_network((ck::CANNetwork)(*i).second.led.can_network);
				new_led->set_led_type((ck::LEDControl::LEDControlData::LEDStripType)(*i).second.led.led_strip_type);
				new_led->set_vbat_config((ck::LEDControl::LEDControlData::VBATConfigType)((*i).second.led.vbat_config));
				new_led->set_vbat_duty_cycle((double)((*i).second.led.vbat_duty_cycle));
				new_led->set_led_control_mode((ck::LEDControl::LEDControlData::LEDControlMode)((*i).second.led.led_control_mode));
				ck::LEDControl::LEDColor* led_color = new ck::LEDControl::LEDColor();
				ck::RGBWColor* rgbw_color = new ck::RGBWColor();
				rgbw_color->set_r((*i).second.led.color.rgbw_color.R);
				rgbw_color->set_g((*i).second.led.color.rgbw_color.G);
				rgbw_color->set_b((*i).second.led.color.rgbw_color.B);
				rgbw_color->set_w((*i).second.led.color.rgbw_color.W);
				led_color->set_allocated_rgbw_color(rgbw_color);
				led_color->set_num_leds((*i).second.led.color.num_leds);
				led_color->set_start_index((*i).second.led.color.start_index);
				new_led->set_allocated_color(led_color);

				for (auto it = ((*i).second.led.animations.begin());
				 it != ((*i).second.led.animations.end());
				 it++)
				 {
					ck::LEDAnimation* animation = new_led->add_animation();
					animation->set_index((*it).index);
					animation->set_brightness((*it).brightness);
					animation->set_speed((*it).speed);
					animation->set_num_led((*it).num_led);
					ck::RGBWColor* animation_color = new ck::RGBWColor();
					animation_color->set_r((*it).color.R);
					animation_color->set_g((*it).color.G);
					animation_color->set_b((*it).color.B);
					animation_color->set_w((*it).color.W);
					animation->set_allocated_color(animation_color);
					animation->set_animation_type((ck::LEDAnimation::AnimationType)(*it).animation_type);
					animation->set_direction((ck::LEDAnimation::Direction)(*it).direction);
					animation->set_offset((*it).offset);
					animation->set_slot((*it).slot);
					animation->set_morse_message((*it).morse_message);
				 }

				if ((*i).second.active_time < ros::Time::now())
				{
					timed_out_led_list.push_back(i);
				}
			}

			for (std::vector<std::map<int32_t, LEDTracker>::iterator>::iterator i = timed_out_led_list.begin();
				 i != timed_out_led_list.end();
				 i++)
			{
				led_control_map.erase((*i));
			}

			bool serialize_status = led_control.SerializeToArray(buffer, 10000);

			if (!serialize_status)
			{
				ROS_INFO("Failed to serialize led status!!");
			}
			else
			{
				zmq_msg_t message;
				zmq_msg_init_size(&message, led_control.ByteSizeLong());
				memcpy(zmq_msg_data(&message), buffer, led_control.ByteSizeLong());
				zmq_msg_set_group(&message, "ledcontrol");
				// std::cout << "Sending message..." << std::endl;
				zmq_msg_send(&message, publisher, 0);
				zmq_msg_close(&message);
			}
		}

		rate.sleep();
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
	ros::init(argc, argv, "ck_ros_base_msgs_node", ros::init_options::NoSigintHandler);
	// GOOGLE_PROTOBUF_VERIFY_VERSION;

	context = zmq_ctx_new();

	ros::NodeHandle n;
	sigintCalled = false;
	signal(SIGINT, sigint_handler);

	node = &n;

	load_config_params();

	std::thread rioReceiveThread(robot_receive_loop);
	std::thread motorSendThread(motor_transmit_loop);
	std::thread solenoidSendThread(solenoid_transmit_loop);
	std::thread motorConfigSendThread(motor_config_transmit_loop);
	std::thread processOverrideHeartbeat(process_override_heartbeat_thread);
	std::thread imuConfigThread(imu_config_thread);
	std::thread ledControlThread(led_transmit_loop);

	ros::Subscriber motorControl = node->subscribe("MotorControl", 100, motorControlCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber motorConfig = node->subscribe("MotorConfiguration", 100, motorConfigCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber modeOverride = node->subscribe("OverrideMode", 10, modeOverrideCallback, ros::TransportHints().tcpNoDelay());

	ros::Subscriber solenoidControl = node->subscribe("SolenoidControl", 100, solenoidControlCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber ledControl = node->subscribe("RioLedControl", 100, ledControlCallback, ros::TransportHints().tcpNoDelay());

	ros::Subscriber modeTuningConfig = node->subscribe("MotorTuningConfiguration", 100, motorTuningConfigCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber modeTuningControl = node->subscribe("MotorTuningControl", 100, motorTuningControlCallback, ros::TransportHints().tcpNoDelay());

	ros::spin();

	rioReceiveThread.join();
	motorSendThread.join();
	solenoidSendThread.join();
	motorConfigSendThread.join();
	processOverrideHeartbeat.join();
	imuConfigThread.join();
	ledControlThread.join();

	if (sigintCalled)
	{
		// Send sigterm due to blocking zmq call when exiting roslaunch
		kill(getpid(), SIGTERM);
	}

	return 0;
}