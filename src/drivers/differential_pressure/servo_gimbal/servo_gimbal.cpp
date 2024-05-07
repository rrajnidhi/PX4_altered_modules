/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#include "servo_gimbal.hpp"

using namespace time_literals;

SERVO_GIMBAL::SERVO_GIMBAL(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

SERVO_GIMBAL::~SERVO_GIMBAL()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);
}

int SERVO_GIMBAL::probe()
{
	// I assume probing wont be required, but this has to be checked first
	_retries = 1;

	// for (int i = 0; i < 10; i++) {
	// 	// perform 2 x Read_DF3 (Data Fetch 3 Bytes)
	// 	//  1st read: require status = Normal Operation. Good Data Packet
	// 	//  2nd read: require status = Stale Data, data should match first read
	// 	uint8_t data_1[3] {};
	// 	int ret1 = transfer(nullptr, 0, &data_1[0], sizeof(data_1));

	// 	uint8_t data_2[3] {};
	// 	int ret2 = transfer(nullptr, 0, &data_2[0], sizeof(data_2));

	// 	if (ret1 == PX4_OK && ret2 == PX4_OK) {
	// 		// Status bits
	// 		const uint8_t status_1 = (data_1[0] & 0b1100'0000) >> 6;
	// 		const uint8_t status_2 = (data_2[0] & 0b1100'0000) >> 6;

	// 		if ((status_1 == (uint8_t)STATUS::Normal_Operation)
	// 		    && (status_2 == (uint8_t)STATUS::Stale_Data)
	// 		    && (data_1[2] == data_1[2])) {

	// 			_retries = 1; // enable retries during operation
	// 			return PX4_OK;

	// 		} else {
	// 			PX4_ERR("status: %X status: %X", status_1, status_2);
	// 		}

	// 	} else {
	// 		px4_usleep(1000); // TODO
	// 	}
	// }

	// return PX4_ERROR;
}

int SERVO_GIMBAL::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

void SERVO_GIMBAL::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);
}

void SERVO_GIMBAL::RunImpl()
{
	switch (_state) {
	case STATE::MEASURE: {
			// Send the command to begin a measurement (Read_MR)
			uint8_t cmd = ADDR_READ_MR;

			if (transfer(&cmd, 1, nullptr, 0) == OK) {
				_timestamp_sample = hrt_absolute_time();
				_state = STATE::READ;
				ScheduleDelayed(2_ms);

			} else {
				perf_count(_comms_errors);
				_state = STATE::MEASURE;
				ScheduleDelayed(10_ms); // try again in 10 ms
			}
		}

		break;

	case STATE::WRITE:
		// enter the code for transfering data to i2c device
		// fetch from topic
		if (_debug_value_sub.copy(&debug_value))
		{
			data = debug_value.value;
		}
		else
		{
			data =  Nan;   //not sure if sending NaN is appropriate
		}
		uint8_t cmd = ADDR_READ_MR;

			if (transfer(&cmd, &data, nullptr, 0) == OK) {
				// sucess in transmitting
			}
			else{
				// report error in transmitting
				PX4_ERROR;
			}



		_state = STATE::MEASURE;
		ScheduleDelayed(10_ms);
		break;
	}
}
