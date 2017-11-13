/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file scr2000.cpp
 * Driver for the murata SCR2000 series MEMS gyroscope combined with single SCR2100x2(X and Y) and SCR2200(Z).
 * Note: This is virtually sensor driver, read sensor data from uORB msgs (not physical bus such as SPI, I2C...)
 */

#include <board_config.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_log.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <mathlib/mathlib.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/device/device.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_x.h>
#include <uORB/topics/sensor_gyro_y.h>
#include <uORB/topics/sensor_gyro_z.h>


class SCR2000
{
public:
	SCR2000();
	~SCR2000();

	void			print_info();

	void 			print_status();

	/**
	 * Start automatic measurement.
	 */
	int				start();
	void			stop();

private:
	struct gyro_report _gyro_report;

	int 	_gyro_x_sub;			/**< gyro axis x sub */
	int 	_gyro_y_sub;			/**< gyro axis y sub */
	int 	_gyro_z_sub;			/**< gyro axis z sub */

	orb_advert_t		_gyro_pub;

	volatile bool 	_task_should_exit;		/**< if true, sensor task should exit */
	int 			_main_task;			/**< task handle for sensor task */
	
	gyro_x_report 	_grp_x = {};
	gyro_y_report 	_grp_y = {};
	gyro_z_report 	_grp_z = {};

	static void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			task_main();

};

namespace scr2000
{
	SCR2000 *g_dev;
}

SCR2000::SCR2000() :
	_gyro_x_sub(-1),
	_gyro_y_sub(-1),
	_gyro_z_sub(-1),
	_gyro_pub(nullptr),
	_task_should_exit(false),
	_main_task(-1)
{
	memset(&_gyro_report, 0, sizeof(_gyro_report));
	memset(&_grp_x, 0, sizeof(_grp_x));
	memset(&_grp_y, 0, sizeof(_grp_y));
	memset(&_grp_z, 0, sizeof(_grp_z));
}

SCR2000::~SCR2000()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	scr2000::g_dev = nullptr;	
}


int
SCR2000::start()
{

	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("scr2000",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 6,
					   2000,
					   (px4_main_t)&SCR2000::task_main_trampoline,
					   nullptr);


	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
SCR2000::stop()
{
	if (scr2000::g_dev != nullptr) {
		delete (scr2000::g_dev);
	}	
}

void
SCR2000::task_main_trampoline(int argc, char *argv[])
{
	scr2000::g_dev->task_main();
}

void
SCR2000::task_main()
{
	// Polling sources
	_gyro_z_sub = orb_subscribe(ORB_ID(sensor_gyro_z));

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _gyro_z_sub;
	fds[0].events = POLLIN;

	_gyro_x_sub = orb_subscribe(ORB_ID(sensor_gyro_x));
	_gyro_y_sub = orb_subscribe(ORB_ID(sensor_gyro_y));

	_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &_gyro_report);
	bool updated = false;

	while (!_task_should_exit) {

		/* wait for up to 50ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(sensor_gyro_z), _gyro_z_sub, &_grp_z);

			orb_check(_gyro_x_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_gyro_x), _gyro_x_sub, &_grp_x);
			}

			orb_check(_gyro_y_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_gyro_y), _gyro_y_sub, &_grp_y);
			}

			if (_grp_x.timestamp == 0 ||
			    _grp_y.timestamp == 0 ||
			    _grp_z.timestamp == 0) {
				// reject until we have valid data
				continue;
			}

			_gyro_report.timestamp       = _grp_z.timestamp;
			_gyro_report.integral_dt     = _grp_z.integral_dt;
			_gyro_report.error_count     = _grp_z.error_count;

			_gyro_report.x               = _grp_x.x;
			_gyro_report.y               = _grp_y.y;
			_gyro_report.z               = _grp_z.z;
			
			_gyro_report.x_integral      = _grp_x.x_integral;
			_gyro_report.y_integral      = _grp_y.y_integral;
			_gyro_report.z_integral      = _grp_z.z_integral;
			
			_gyro_report.temperature     = _grp_z.temperature;
			
			_gyro_report.range_rad_s     = _grp_z.range_rad_s;
			_gyro_report.scaling         = _grp_z.scaling;
			
			_gyro_report.x_raw           = _grp_x.x_raw;
			_gyro_report.y_raw           = _grp_y.y_raw ;
			_gyro_report.z_raw           = _grp_z.z_raw ;
			
			_gyro_report.temperature_raw = _grp_z.temperature_raw;
			
			_gyro_report.device_id       = _grp_z.device_id;

			orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &_gyro_report);
		}
	}

	orb_unsubscribe(_gyro_x_sub);
	orb_unsubscribe(_gyro_y_sub);
	orb_unsubscribe(_gyro_z_sub);
	orb_unadvertise(_gyro_pub);

	PX4_INFO("Exiting.");
	_main_task = -1;
}

void
SCR2000::print_info()
{
	// perf_print_counter(_sample_perf);
}

void
SCR2000::print_status()
{
	warnx("single read");
	warnx("time:     %lld", _gyro_report.timestamp);
	warnx("gyro  x:  \t%8.4f\trad/s", (double)_gyro_report.x);
	warnx("gyro  y:  \t%8.4f\trad/s", (double)_gyro_report.y);
	warnx("gyro  z:  \t%8.4f\trad/s", (double)_gyro_report.z);
	warnx("temperature:  \t%8.4f\tcels degree", (double)_gyro_report.temperature);
	warnx("gyro  x:  \t%d\traw 0x%0x", (short)_gyro_report.x_raw, (unsigned short)_gyro_report.x_raw);
	warnx("gyro  y:  \t%d\traw 0x%0x", (short)_gyro_report.y_raw, (unsigned short)_gyro_report.y_raw);
	warnx("gyro  z:  \t%d\traw 0x%0x", (short)_gyro_report.z_raw, (unsigned short)_gyro_report.z_raw);
	warnx("temperature:  \t%d\traw 0x%0x", (short)_gyro_report.temperature_raw, (unsigned short)_gyro_report.temperature_raw);
	warnx("gyro range: %8.4f rad/s (%8.4f deg/s)", (double)_gyro_report.range_rad_s,
	      (double)(_gyro_report.range_rad_s / 9.81f));

	errx(0, "PASS");
}


extern "C" { __EXPORT int scr2000_main(int argc, char *argv[]); }

int
scr2000_main(int argc, char *argv[])
{

	if (argc < 2) {
		warnx("usage: scr2000 {start|stop|status|info}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (scr2000::g_dev != nullptr) {
			warnx("already running");
			return 0;
		}

		/* create the driver */
		scr2000::g_dev = new SCR2000();

		if (scr2000::g_dev == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != scr2000::g_dev->start()) {
			delete scr2000::g_dev;
			scr2000::g_dev = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}
	
	if (!strcmp(argv[1], "info")) {
		scr2000::g_dev->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		scr2000::g_dev->stop();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		scr2000::g_dev->print_status();
		return 0;
	}

	warnx("unrecognized command");

	return 1;
}
