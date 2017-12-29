/****************************************************************************
 *
 * Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file df_ltc2946_wrapper.cpp
 * Driver to access the LTC2946 of the DriverFramework.
 *
 * @author Christoph Tobler <christoph@px4.io>
 */


#include <string>
#include <px4_config.h>
#include <systemlib/err.h>

#include <ltc2946/LTC2946.hpp>



extern "C" { __EXPORT int df_ltc2946_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfLtc2946Wrapper : public LTC2946
{
public:
	DfLtc2946Wrapper();
	~DfLtc2946Wrapper();

	int		start();
	int		stop();

private:
	int _publish(struct ltc2946_sensor_data &data);

};

DfLtc2946Wrapper::DfLtc2946Wrapper() :
	LTC2946(LTC2946_DEVICE_PATH)
{
}

DfLtc2946Wrapper::~DfLtc2946Wrapper()
{
}

int DfLtc2946Wrapper::start()
{
	int ret;

	/* Init device and start sensor. */
	ret = init();

	//
	if (ret != 0) {
		PX4_ERR("LTC2946 init fail: %d", ret);
		return ret;
	}

	ret = LTC2946::start();

	if (ret != 0) {
		PX4_ERR("LTC2946 start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfLtc2946Wrapper::stop()
{
	/* Stop sensor. */
	int ret = LTC2946::stop();

	if (ret != 0) {
		PX4_ERR("LTC2946 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfLtc2946Wrapper::_publish(struct ltc2946_sensor_data &data)
{
	// TODO
	return 0;
}


namespace df_ltc2946_wrapper
{

DfLtc2946Wrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
	PX4_WARN("starting LTC2946");
	g_dev = new DfLtc2946Wrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfLtc2946Wrapper object");
		return -1;

	} else {
		PX4_INFO("started LTC2946");
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfLtc2946Wrapper start failed");
		return ret;
	}

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
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

	return 0;
}

void
usage()
{
	PX4_WARN("Usage: df_ltc2946_wrapper 'start', 'info', 'stop'");
}

} // namespace df_ltc2946_wrapper


int
df_ltc2946_wrapper_main(int argc, char *argv[])
{
	int ret = 0;

	if (argc <= 1) {
		df_ltc2946_wrapper::usage();
		return 1;
	}

	const char *verb = argv[1];


	if (!strcmp(verb, "start")) {
		ret = df_ltc2946_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_ltc2946_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_ltc2946_wrapper::info();
	}

	else {
		df_ltc2946_wrapper::usage();
		return 1;
	}

	return ret;
}
