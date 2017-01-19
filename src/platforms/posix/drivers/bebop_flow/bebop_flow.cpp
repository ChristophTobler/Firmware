/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file bebop_flow.cpp
 *
 * This is a wrapper around the Parrot Bebop's downward-facing camera and integrates
 * an optical flow computation.
 */

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>

#include "video_device.h"
#include "dump_pgm.h"
#include <mt9v117/MT9V117.hpp>
#include <drivers/drv_hrt.h>
#include "OpticalFlow/include/flow_px4.hpp"

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/vehicle_attitude.h>

#include <conversion/rotation.h>

#define FLOW_OUTPUT_RATE 20
#define FOCAL_LENGTH_X 412.3540 //from 320x240 resolution
#define FOCAL_LENGTH_Y 306.5525 //from 320x240 resolution
#define FLOW_SEARCH_SIZE 8

extern "C" { __EXPORT int bebop_flow_main(int argc, char *argv[]); }

using namespace DriverFramework;

namespace bebop_flow
{
orb_advert_t _flow_topic;
int _orb_class_instance;
struct gyro {
	float pitchrate;
	float rollrate;
	float yawrate;
};
struct flow_ang {
	float x;
	float y;
};

MT9V117 *image_sensor = nullptr;				 // I2C image sensor
VideoDevice *g_dev = nullptr;            // interface to the video device
volatile bool _task_should_exit = false; // flag indicating if bebop flow task should exit
static bool _is_running = false;         // flag indicating if bebop flow  app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread
volatile unsigned int _trigger = 0;			 // Number of images to write as pgm

const char *dev_name = "/dev/video0";		 // V4L video device

int start();
int stop();
int info();
int trigger(int count);
int clear_errors();
void usage();
int publish_flow(int quality, struct flow_ang angular_flow, int dt, struct gyro gyro_integral);
void task_main(int argc, char *argv[]);

void task_main(int argc, char *argv[])
{
	_is_running = true;
	int ret = 0;
	struct frame_data frame;
	memset(&frame, 0, sizeof(frame));
	uint32_t timeout_cnt = 0;

	//subscribe to sensor_combined for gyro measurements
	struct vehicle_attitude_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	struct gyro gyro_int = {0.0f, 0.0f, 0.0f};
	struct flow_ang angular_flow = {0.0f, 0.0f};

	_flow_topic = nullptr;
	_orb_class_instance = -1;

	//initialize flow
	OpticalFlowPX4 *_optical_flow = new OpticalFlowPX4(FOCAL_LENGTH_X, FOCAL_LENGTH_Y, FLOW_OUTPUT_RATE,
			VIDEO_DEVICE_CROP_WIDTH, VIDEO_DEVICE_CROP_HEIGHT, FLOW_SEARCH_SIZE);

	// Main loop
	while (!_task_should_exit) {

		ret = g_dev->get_frame(frame);

		if (ret < 0) {
			PX4_ERR("Get Frame failed");
			continue;

		} else if (ret == 1) {
			// No image in buffer
			usleep(1000);
			++timeout_cnt;

			if (timeout_cnt > 1000) {
				PX4_WARN("No frames received for 1 sec");
				timeout_cnt = 0;
			}

			continue;
		}

		timeout_cnt = 0;

		// Write images into a file
		if (_trigger > 0) {
			PX4_INFO("Trigger camera");

			dump_pgm(frame.data, frame.bytes, frame.seq, frame.timestamp);
			--_trigger;
		}

		/***************************************************************
		*
		* Optical Flow computation
		*
		**************************************************************/
		static uint32_t lasttimestamp;
		uint32_t deltatime = (frame.timestamp - lasttimestamp);

		// get gyro measurements from vehicle_attitude to fill opticalFlow_msgs
		static bool updated;
		orb_check(vehicle_attitude_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &sensor);
			//integrate gyro at same rate
			gyro_int.pitchrate += sensor.rollspeed * deltatime / 1000000.0f;
			gyro_int.rollrate += sensor.pitchspeed * deltatime / 1000000.0f;
			gyro_int.yawrate += sensor.yawspeed * deltatime / 1000000.0f;
		}

		lasttimestamp = frame.timestamp;

		int dt_us = 0;
		//calculate angular flow
		int quality = _optical_flow->calcFlow((uint8_t *)frame.data, frame.timestamp, dt_us, angular_flow.x, angular_flow.y);

		//calcFlow() will return -1 if it is still accumulating flow -> ouput rate
		if (quality >= 0) {
			publish_flow(quality, angular_flow, dt_us, gyro_int);
			gyro_int = {0.0f, 0.0f, 0.0f};
		}

		ret = g_dev->put_frame(frame);

		if (ret < 0) {
			PX4_ERR("Put Frame failed");
		}
	}

	_is_running = false;
}

int start()
{
	if (_is_running) {
		PX4_WARN("bebop_flow already running");
		return -1;
	}

	// Prepare the I2C device
	image_sensor = new MT9V117(IMAGE_DEVICE_PATH);

	if (image_sensor == nullptr) {
		PX4_ERR("failed instantiating image sensor object");
		return -1;
	}

	int ret = image_sensor->start();

	if (ret != 0) {
		PX4_ERR("Image sensor start failed");
		return ret;
	}

	// Start the video device
	g_dev = new VideoDevice(dev_name, 6);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating video device object");
		return -1;
	}

	ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("Video device start failed");
		return ret;
	}

	/* start the task */
	_task_handle = px4_task_spawn_cmd("bebop_flow",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	// Stop bebop flow task
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	_task_should_exit = false;
	_trigger = 0;

	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	if (image_sensor == nullptr) {
		PX4_ERR("Image sensor not running");
		return -1;
	}

	ret = image_sensor->stop();

	if (ret != 0) {
		PX4_ERR("Image sensor driver  could not be stopped");
		return ret;
	}

	delete g_dev;
	delete image_sensor;
	g_dev = nullptr;
	image_sensor = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	int ret = g_dev->print_info();

	if (ret != 0) {
		PX4_ERR("Unable to print info");
		return ret;
	}

	return 0;
}

int trigger(int count)
{
	if (_is_running) {
		_trigger = count;

	} else {
		PX4_WARN("bebop_flow is not running");
	}

	return OK;
}

void usage()
{
	PX4_INFO("Usage: bebop_flow 'start', 'info', 'stop', 'trigger [-n #]'");
}

int publish_flow(int quality, struct flow_ang ang_flow, int dt, struct gyro gyro_integral)
{
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));

	flow.sensor_id = 2.0;
	flow.timestamp = hrt_absolute_time();
	flow.time_since_last_sonar_update = 0;
	flow.frame_count_since_last_readout = 0; // ?
	flow.integration_timespan = dt;

	flow.ground_distance_m = 0;
	flow.gyro_temperature = 0;
	flow.gyro_x_rate_integral = gyro_integral.pitchrate;
	flow.gyro_y_rate_integral = gyro_integral.rollrate;
	flow.gyro_z_rate_integral = gyro_integral.yawrate;
	flow.pixel_flow_x_integral = ang_flow.x;
	flow.pixel_flow_y_integral = ang_flow.y;
	flow.quality = quality;

	/* rotate measurements according to parameter */
	enum Rotation flow_rot;
	param_get(param_find("SENS_FLOW_ROT"), &flow_rot);

	float zeroval = 0.0f;
	rotate_3f(flow_rot, flow.pixel_flow_x_integral, flow.pixel_flow_y_integral, zeroval);
	rotate_3f(flow_rot, flow.gyro_x_rate_integral, flow.gyro_y_rate_integral, flow.gyro_z_rate_integral);

	if (_flow_topic == nullptr) {
		_flow_topic = orb_advertise_multi(ORB_ID(optical_flow), &flow,
						  &_orb_class_instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_topic, &flow);
	}

	return OK;
}

} /* bebop flow namespace*/

int
bebop_flow_main(int argc, char *argv[])
{
	int ch;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;
	unsigned int trigger_count = 1;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'n':
			trigger_count = atoi(myoptarg);
			break;

		default:
			bebop_flow::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		bebop_flow::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ret = bebop_flow::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = bebop_flow::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = bebop_flow::info();
	}

	else if (!strcmp(verb, "trigger")) {
		ret = bebop_flow::trigger(trigger_count);
	}

	else {
		bebop_flow::usage();
		return 1;
	}

	return ret;
}
