/***************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file navigator.h
 * Helper class to access missions
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "datalinkloss.h"
#include "enginefailure.h"
#include "follow_target.h"
#include "geofence.h"
#include "gpsfailure.h"
#include "land.h"
#include "loiter.h"
#include "mission.h"
#include "navigator_mode.h"
#include "rcloss.h"
#include "rtl.h"
#include "takeoff.h"

#include <controllib/block/BlockParam.hpp>
#include <controllib/blocks.hpp>
#include <navigator/navigation.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/uORB.h>
#include <uORB/topics/rst_insert_global_syn_task_debug.h>

/**
 * Number of navigation modes that need on_active/on_inactive calls
 */
#define NAVIGATOR_MODE_ARRAY_SIZE 10

class Navigator : public control::SuperBlock
{
public:
	Navigator();
	~Navigator();
	Navigator(const Navigator &) = delete;
	Navigator operator=(const Navigator &) = delete;

	/**
	 * Start the navigator task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display the navigator status.
	 */
	void		status();

	/**
	 * Load fence from file
	 */
	void		load_fence_from_file(const char *filename);

	/**
	 * Publish the geofence result
	 */
	void		publish_geofence_result();

	void		publish_vehicle_cmd(vehicle_command_s *vcmd);

	/**
	 * Generate an artificial traffic indication
	 *
	 * @param distance Horizontal distance to this vehicle
	 * @param direction Direction in earth frame from this vehicle in radians
	 * @param traffic_heading Travel direction of the traffic in earth frame in radians
	 * @param altitude_diff Altitude difference, positive is up
	 * @param hor_velocity Horizontal velocity of traffic, in m/s
	 * @param ver_velocity Vertical velocity of traffic, in m/s
	 */
	void		fake_traffic(const char *callsign, float distance, float direction, float traffic_heading, float altitude_diff,
				     float hor_velocity, float ver_velocity);

	/**
	 * Check nearby traffic for potential collisions
	 */
	void		check_traffic();

	/**
	 * Setters
	 */
	void		set_can_loiter_at_sp(bool can_loiter) { _can_loiter_at_sp = can_loiter; }
	void		set_position_setpoint_triplet_updated() { _pos_sp_triplet_updated = true; }
	void		set_mission_result_updated() { _mission_result_updated = true; }

	/**
	 * Getters
	 */
	struct fw_pos_ctrl_status_s *get_fw_pos_ctrl_status() { return &_fw_pos_ctrl_status; }
	struct home_position_s *get_home_position() { return &_home_pos; }
	struct mission_result_s *get_mission_result() { return &_mission_result; }
	struct position_setpoint_triplet_s *get_position_setpoint_triplet() { return &_pos_sp_triplet; }
	struct position_setpoint_triplet_s *get_reposition_triplet() { return &_reposition_triplet; }
	struct position_setpoint_triplet_s *get_takeoff_triplet() { return &_takeoff_triplet; }
	struct vehicle_global_position_s *get_global_position() { return &_global_pos; }
	struct vehicle_land_detected_s *get_land_detected() { return &_land_detected; }
	struct vehicle_local_position_s *get_local_position() { return &_local_pos; }
	struct vehicle_status_s *get_vstatus() { return &_vstatus; }

	const vehicle_roi_s &get_vroi() { return _vroi; }

	bool home_alt_valid() { return (_home_pos.timestamp > 0 && _home_pos.valid_alt); }
	bool home_position_valid() { return (_home_pos.timestamp > 0 && _home_pos.valid_alt && _home_pos.valid_hpos); }

	int		get_onboard_mission_sub() { return _onboard_mission_sub; }
	int		get_offboard_mission_sub() { return _offboard_mission_sub; }

	Geofence	&get_geofence() { return _geofence; }

	bool		get_can_loiter_at_sp() { return _can_loiter_at_sp; }
	float		get_loiter_radius() { return _param_loiter_radius.get(); }

	/**
	 * Returns the default acceptance radius defined by the parameter
	 */
	float		get_default_acceptance_radius();

	/**
	 * Get the acceptance radius
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float		get_acceptance_radius();

	/**
	 * Get the altitude acceptance radius
	 *
	 * @return the distance from the target altitude before considering the waypoint reached
	 */
	float		get_altitude_acceptance_radius();

	/**
	 * Get the cruising speed
	 *
	 * @return the desired cruising speed for this mission
	 */
	float		get_cruising_speed();

	/**
	 * Set the cruising speed
	 *
	 * Passing a negative value or leaving the parameter away will reset the cruising speed
	 * to its default value.
	 *
	 * For VTOL: sets cruising speed for current mode only (multirotor or fixed-wing).
	 *
	 */
	void		set_cruising_speed(float speed = -1.0f);

	/**
	 * Reset cruising speed to default values
	 *
	 * For VTOL: resets both cruising speeds.
	 */
	void		reset_cruising_speed();


	/**
	 *  Set triplets to invalid
	 */
	void 		reset_triplets();

	/**
	 * Get the target throttle
	 *
	 * @return the desired throttle for this mission
	 */
	float		get_cruising_throttle();

	/**
	 * Set the target throttle
	 */
	void		set_cruising_throttle(float throttle = -1.0f) { _mission_throttle = throttle; }

	/**
	 * Get the acceptance radius given the mission item preset radius
	 *
	 * @param mission_item_radius the radius to use in case the controller-derived radius is smaller
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float		get_acceptance_radius(float mission_item_radius);

	orb_advert_t	*get_mavlink_log_pub() { return &_mavlink_log_pub; }

	void		increment_mission_instance_count() { _mission_result.instance_count++; }

	void 		set_mission_failure(const char *reason);

	// MISSION
	bool		is_planned_mission() const { return _navigation_mode == &_mission; }
	bool		on_mission_landing() { return _mission.landing(); }
	bool		start_mission_landing() { return _mission.land_start(); }

	// RTL
	bool		mission_landing_required() { return _rtl.mission_landing_required(); }

	bool		abort_landing();

	// Param access
	float		get_loiter_min_alt() const { return _param_loiter_min_alt.get(); }
	bool		force_vtol() const { return _vstatus.is_vtol && !_vstatus.is_rotary_wing && _param_force_vtol.get(); }

private:

	bool		_task_should_exit{false};	/**< if true, sensor task should exit */
	int		_navigator_task{-1};		/**< task handle for sensor task */

	int		_fw_pos_ctrl_status_sub{-1};	/**< notification of vehicle capabilities updates */
	int		_global_pos_sub{-1};		/**< global position subscription */
	int		_gps_pos_sub{-1};		/**< gps position subscription */
	int		_home_pos_sub{-1};		/**< home position subscription */
	int		_land_detected_sub{-1};		/**< vehicle land detected subscription */
	int		_local_pos_sub{-1};		/**< local position subscription */
	int		_offboard_mission_sub{-1};	/**< offboard mission subscription */
	int		_onboard_mission_sub{-1};	/**< onboard mission subscription */
	int		_param_update_sub{-1};		/**< param update subscription */
	int		_sensor_combined_sub{-1};	/**< sensor combined subscription */
	int		_traffic_sub{-1};		/**< traffic subscription */
	int		_vehicle_command_sub{-1};	/**< vehicle commands (onboard and offboard) */
	int		_vstatus_sub{-1};		/**< vehicle status subscription */

	orb_advert_t	_geofence_result_pub{nullptr};
	orb_advert_t	_mavlink_log_pub{nullptr};	/**< the uORB advert to send messages over mavlink */
	orb_advert_t	_mission_result_pub{nullptr};
	orb_advert_t	_pos_sp_triplet_pub{nullptr};
	orb_advert_t	_vehicle_cmd_ack_pub{nullptr};
	orb_advert_t	_vehicle_cmd_pub{nullptr};
	orb_advert_t	_vehicle_roi_pub{nullptr};

	// Subscriptions
	fw_pos_ctrl_status_s				_fw_pos_ctrl_status{};	/**< fixed wing navigation capabilities */
	home_position_s					_home_pos{};		/**< home position for RTL */
	mission_result_s				_mission_result{};
	sensor_combined_s				_sensor_combined{};	/**< sensor values */
	vehicle_global_position_s			_global_pos{};		/**< global vehicle position */
	vehicle_gps_position_s				_gps_pos{};		/**< gps position */
	vehicle_land_detected_s				_land_detected{};	/**< vehicle land_detected */
	vehicle_local_position_s			_local_pos{};		/**< local vehicle position */
	vehicle_status_s				_vstatus{};		/**< vehicle status */

	// Publications
	geofence_result_s				_geofence_result{};
	position_setpoint_triplet_s			_pos_sp_triplet{};	/**< triplet of position setpoints */
	position_setpoint_triplet_s			_reposition_triplet{};	/**< triplet for non-mission direct position command */
	position_setpoint_triplet_s			_takeoff_triplet{};	/**< triplet for non-mission direct takeoff command */
	vehicle_roi_s					_vroi{};		/**< vehicle ROI */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence	_geofence;			/**< class that handles the geofence */
	bool		_geofence_violation_warning_sent{false}; /**< prevents spaming to mavlink */

	bool		_can_loiter_at_sp{false};			/**< flags if current position SP can be used to loiter */
	bool		_pos_sp_triplet_updated{false};		/**< flags if position SP triplet needs to be published */
	bool 		_pos_sp_triplet_published_invalid_once{false};	/**< flags if position SP triplet has been published once to UORB */
	bool		_mission_result_updated{false};		/**< flags if mission result has seen an update */

	NavigatorMode	*_navigation_mode{nullptr};		/**< abstract pointer to current navigation mode class */
	Mission		_mission;			/**< class that handles the missions */
	Loiter		_loiter;			/**< class that handles loiter */
	Takeoff		_takeoff;			/**< class for handling takeoff commands */
	Land		_land;			/**< class for handling land commands */
	RTL 		_rtl;				/**< class that handles RTL */
	RCLoss 		_rcLoss;				/**< class that handles RTL according to OBC rules (rc loss mode) */
	DataLinkLoss	_dataLinkLoss;			/**< class that handles the OBC datalink loss mode */
	EngineFailure	_engineFailure;			/**< class that handles the engine failure mode (FW only!) */
	GpsFailure	_gpsFailure;			/**< class that handles the OBC gpsfailure loss mode */
	FollowTarget	_follow_target;

	NavigatorMode *_navigation_mode_array[NAVIGATOR_MODE_ARRAY_SIZE];	/**< array of navigation modes */

	// navigator parameters
	control::BlockParamFloat _param_loiter_radius;	/**< loiter radius for fixedwing */
	control::BlockParamFloat _param_acceptance_radius;	/**< acceptance for takeoff */
	control::BlockParamFloat _param_fw_alt_acceptance_radius;	/**< acceptance radius for fixedwing altitude */
	control::BlockParamFloat _param_mc_alt_acceptance_radius;	/**< acceptance radius for multicopter altitude */
	control::BlockParamInt _param_force_vtol;	/**< acceptance radius for multicopter altitude */
	control::BlockParamInt _param_traffic_avoidance_mode;	/**< avoiding other aircraft is enabled */

	// non-navigator parameters
	control::BlockParamFloat _param_loiter_min_alt;

	float _mission_cruising_speed_mc{-1.0f};
	float _mission_cruising_speed_fw{-1.0f};
	float _mission_throttle{-1.0f};

	struct global_syn_task_s{
		uint64_t cmd_start_time;
		vehicle_command_s cmd;
		vehicle_status_s status{};
	};

	struct insert_task_debug_msg_s{
		uint8_t task_type;
		int32_t start_task_time_interval;
		uint8_t error_type;
		uint8_t task_count;
		bool whether_record_msg;
		uint16_t vehicle_command;
	};

	enum new_task_type_e{
		no_task = 0,
		immediate_task,
		deferred_task,
	};

	enum new_task_error_type_e{
		no_error = 0,
		task_startup_time_range_error,
		no_state_available,
		global_syn_time_no_start,
	};

	typedef enum VEHICLE_MODE_FLAG
	{
		VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
		VEHICLE_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
		VEHICLE_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
		VEHICLE_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
		VEHICLE_MODE_FLAG_STABILIZE_ENABLED=16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
		VEHICLE_MODE_FLAG_HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
		VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled. | */
		VEHICLE_MODE_FLAG_SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
		VEHICLE_MODE_FLAG_ENUM_END=129, /*  | */
	} VEHICLE_MODE_FLAG;

	enum PX4_CUSTOM_MAIN_MODE {
		PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
		PX4_CUSTOM_MAIN_MODE_ALTCTL,
		PX4_CUSTOM_MAIN_MODE_POSCTL,
		PX4_CUSTOM_MAIN_MODE_AUTO,
		PX4_CUSTOM_MAIN_MODE_ACRO,
		PX4_CUSTOM_MAIN_MODE_OFFBOARD,
		PX4_CUSTOM_MAIN_MODE_STABILIZED,
		PX4_CUSTOM_MAIN_MODE_RATTITUDE,
		PX4_CUSTOM_MAIN_MODE_SIMPLE /* unused, but reserved for future use */
	};

	enum PX4_CUSTOM_SUB_MODE_AUTO {
		PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
		PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
		PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
		PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
		PX4_CUSTOM_SUB_MODE_AUTO_RTL,
		PX4_CUSTOM_SUB_MODE_AUTO_LAND,
		PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
		PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET
	};

	uint8_t _last_nav_status = 0;
    
	bool _gps_syn_time_update = false;
	bool _nav_status_update = false;
	bool _nav_status_update_valid = true;
	bool _global_syn_vehicle_command_update = false;

	bool _get_immediate_new_task = false;

	uint8_t _status_from_cmd;
	global_syn_task_s _global_syn_task_linked_list;
	global_syn_task_s _next_global_syn_task{};

	bool request_one_task = true;
	vehicle_status_s	_carry_out_status{};

	bool _get_status = false;
	uint8_t _task_count = 0;

	insert_task_debug_msg_s _insert_task_debug_msg;

	global_syn_task_s _global_syn_task{};

	bool     _carry_out_global_syn_cmd = false;

	vehicle_command_s _global_syn_cmd;

	orb_advert_t	_insert_global_syn_task_debug_pub{nullptr};
	rst_insert_global_syn_task_debug_s _insert_syn_task_debug;

	// update subscriptions
	void		fw_pos_ctrl_status_update(bool force = false);
	void		global_position_update();
	void		gps_position_update();
	void		home_position_update(bool force = false);
	void		local_position_update();
	void		params_update();
	void		sensor_combined_update();
	void		vehicle_land_detected_update();
	void		vehicle_status_update();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main task.
	 */
	void		task_main();

	/**
	 * Publish a new position setpoint triplet for position controllers
	 */
	void		publish_position_setpoint_triplet();

	/**
	 * Publish the mission result so commander and mavlink know what is going on
	 */
	void		publish_mission_result();

	void		publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result);

	void rst_swarm_link_global_syn_task_scheduler(vehicle_command_s cmd, vehicle_status_s vstatus,
					insert_task_debug_msg_s *insert_task_debug_msg, bool *carry_out_global_syn_cmd, global_syn_task_s *global_syn_task);

	uint64_t calculate_global_syn_time(bool gps_syn_time_update, vehicle_gps_position_s gps_pos);

	void task_insert(vehicle_command_s cmd, vehicle_status_s vstatus, uint64_t global_syn_time, uint8_t *task_count,
			insert_task_debug_msg_s *insert_task_debug_msg, global_syn_task_s *global_syn_task, bool *get_immediate_new_task);

	uint8_t from_cmd_get_status(vehicle_command_s cmd);

	void task_query(global_syn_task_s _global_syn_task_linked_list, global_syn_task_s *global_syn_task);

	void task_carry_out(global_syn_task_s global_syn_task, 
							uint64_t global_syn_time, bool *carry_out);
};
#endif
