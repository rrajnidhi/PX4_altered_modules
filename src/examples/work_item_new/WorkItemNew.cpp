/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "WorkItemNew.hpp"

WorkItemNew::WorkItemNew() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemNew::~WorkItemNew()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemNew::init()
{
	// execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // at 200 Hz rate for a fast PWM servo
	ScheduleOnInterval(25000_us); // at 40 Hz rate for a slow PWM servo
	// ScheduleOnInterval(1000000_us); // at 1 Hz rate

	return true;
}

void WorkItemNew::Run()
{

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//  grab latest vehicle odometry data from which roll can be extracted
	if (_vehicle_odometry_sub.updated())
	{
		vehicle_odometry_s vehicle_odometry{};
		if (_vehicle_odometry_sub.copy(&vehicle_odometry))
		{
			double w= vehicle_odometry.q[0];
			double x= vehicle_odometry.q[1];
			double y= vehicle_odometry.q[2];
			double z= vehicle_odometry.q[3];
			_roll=atan2(2.0 * (z * y + w * x) , 1.0 - 2.0 * (x * x + y * y));
			// mavlink_log_critical(&_mavlink_log_pub, "roll is %f",double(_roll*180/3.14));
		}
	}


	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated())
	{
		vehicle_status_s vehicle_status;
		if (_vehicle_status_sub.copy(&vehicle_status))
		{
			if(vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
			{
				_is_nav_state_mission=true;
				// mavlink_log_critical(&_mavlink_log_pub, "roll is %f",double(_roll*180/3.14));
				// publish value to servo from here, it can be either through PWM or I2C directly or through UORB topics
				/* advertise indexed debug value */
				struct debug_value_s dbg_ind;
				dbg_ind.ind = 42;
				dbg_ind.value = 0.5f;
				orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);
				orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);
			}
			else
			{
				_is_nav_state_mission=false;
				struct debug_value_s dbg_ind;
				dbg_ind.ind = 42;
				dbg_ind.value = 0.0f;
				orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);
				orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);
			}
		}
	}


	// Example
	//  publish some data
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);


	perf_end(_loop_perf);
}

int WorkItemNew::task_spawn(int argc, char *argv[])
{
	WorkItemNew *instance = new WorkItemNew();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemNew::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemNew::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemNew::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
A simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_new", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_new_main(int argc, char *argv[])
{
	return WorkItemNew::main(argc, argv);
}
