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
 * @file scr2100_x.cpp
 * Driver for the murata SCR2100_X MEMS gyro connected via SPI.
 */

#include <px4_config.h>
#include <px4_defines.h>

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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#define SCR2100_X_DEVICE_PATH	"/dev/scr2100_x"

/*gyro sensitivities*/
#define GYRO_SENSITIVITY 50.0f // LSB/dps

/*Standard requests*/
#define REQ_READ_RATE 			0x040000f7
#define REQ_READ_TEMP 			0x1c0000e3
#define REQ_WRITE_FLT_60 		0xfc200006
#define REQ_WRITE_FLT_10 		0xfc1000c7
#define REQ_READ_STAT_SUM 		0x7c0000b3
#define REQ_READ_RATE_STAT1 	0x240000c7
#define REQ_READ_RATE_STAT2 	0x280000cd
#define REQ_READ_COM_STAT1 		0x6c0000ab

// Special requests
#define REQ_HARD_RESET 			0xD8000431

/*Frame field masks*/
#define OPCODE_FIELD_MASK		0xFC000000
#define RS_FIELD_MASK			0x03000000
#define DATA_FIELD_MASK			0x00FFFF00
#define CRC_FIELD_MASK			0x000000FF

/*return status definition*/
#define RS_STARTUP			0
#define RS_NO_ERROR			1
#define RS_ERROR			3

/*set user sensor poll rate, according to datasheet should be set 2000Hz*/
#define SENSOR_POLLRATE_USER				1000
#define SCR2100_X_DEFAULT_FILTER_FREQ		200
// #define SCR2100_X_MAX_OUTPUT_RATE			2300

extern "C" { __EXPORT int scr2100_x_main(int argc, char *argv[]); }

class SCR2100_X : public device::SPI
{
public:
	SCR2100_X(int bus, spi_dev_e device);
	virtual ~SCR2100_X();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	struct hrt_call		_call;
	unsigned		_call_interval;

	ringbuffer::RingBuffer		*_reports;

	struct gyro_calibration_s	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;
	orb_advert_t		_gyro_topic;
	// int			_class_instance;

	unsigned		_current_range;

	perf_counter_t		_sample_perf;

	math::LowPassFilter2p	_gyro_filter_x;

	// Integrator		_gyro_int;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();


	/**
	 * Send request to sensor and read back the response for previous request.
	 * 
	 * @param request 	the 32-bit request to send
	 * @return 			the response for previous request
	 */
	
	uint32_t		send_request(uint32_t request);

	/**
	 * Set the lowpass filter of the driver
	 *
	 * @param samplerate	The current samplerate
	 * @param frequency	The cutoff frequency for the lowpass filter
	 */
	void			set_driver_lowpass_filter(float samplerate, float bandwidth);

};

SCR2100_X::SCR2100_X(int bus, spi_dev_e device) :
	SPI("SCR2100_X", SCR2100_X_DEVICE_PATH, bus, device, SPIDEV_MODE0, 8000000),
	_call_interval(0),
	_reports(nullptr),
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(nullptr),
	_current_range(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "scr2100_x_read")),
	_gyro_filter_x(SENSOR_POLLRATE_USER, SCR2100_X_DEFAULT_FILTER_FREQ)
{
	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_SCR2100_X;

	// enable debug() calls
	_debug_enabled = true;

	// default scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
}

SCR2100_X::~SCR2100_X()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// if (_class_instance != -1) {
	// 	unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_instance);
	// }

	/* delete the perf counter */
	perf_free(_sample_perf);
}

int
SCR2100_X::init()
{
	int ret = PX4_ERROR;
	uint32_t response_rate_stat1;
	uint32_t response_rate_stat2;
	uint32_t response_stat_sum;
	uint32_t response_com_stat1;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_x_report));

	if (_reports == nullptr) {
		goto out;
	}

	// _class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	/* set new range scaling factor */
	_current_range = 300;
	_gyro_range_rad_s = _current_range / 180.0f * M_PI_F;
	_gyro_range_scale = 1 / (GYRO_SENSITIVITY * 180.0f) * M_PI_F;

	/*to do:reset sensor use pull down PIN_EXTRESN*/


	/* Wait 25 ms until the SCR2100 is accessible via SPI */
	usleep(25000);
	/* Set output filter to 60 Hz */
	send_request(REQ_WRITE_FLT_60);
	/*wait 595 ms in case the output filter is set to 60 Hz, 725 ms if the filter is set to 10 Hz*/
	usleep(595000); 
	/*Clear status registers*/
	send_request(REQ_READ_RATE_STAT1);
	send_request(REQ_READ_RATE_STAT2);
	send_request(REQ_READ_COM_STAT1);
	send_request(REQ_READ_STAT_SUM);

	response_stat_sum   = send_request(REQ_READ_RATE_STAT1);
	response_rate_stat1 = send_request(REQ_READ_RATE_STAT2);
	response_rate_stat2 = send_request(REQ_READ_COM_STAT1);
	response_com_stat1  = send_request(REQ_READ_COM_STAT1);

	ret = OK;
	if (((response_rate_stat1 & 0x00C03F00) != 0x00C03F00) || (response_rate_stat1 == 0xFFFFFFFF))
	ret = PX4_ERROR;
	if (((response_rate_stat2 & 0x0001FF00) != 0x0001FF00) || (response_rate_stat2 == 0xFFFFFFFF))
	ret = PX4_ERROR;
	if (((response_com_stat1 & 0x00F87700) != 0x00F87700) || (response_com_stat1 == 0xFFFFFFFF))
	ret = PX4_ERROR;
	if (((response_stat_sum & 0x00004100) != 0x00004100) || (response_stat_sum == 0xFFFFFFFF))
	ret = PX4_ERROR;

	send_request(REQ_READ_TEMP);

	/* advertise sensor topic, measure manually to initialize valid report */

	measure();

	struct gyro_x_report grp;
	_reports->get(&grp);

	/* measurement will have generated a report, publish */
	_gyro_topic = orb_advertise(ORB_ID(sensor_gyro_x), &grp);

	if (_gyro_topic == nullptr) {
		DEVICE_DEBUG("failed to create sensor_gyro_x publication");
	}

	// ret = OK;

out:
	return ret;
}

int
SCR2100_X::probe()
{
	/* dummy read to ensure SPI state machine is sane */
	// send_request(REQ_READ_WHOAMI);//

	// if (read_reg(ADDR_CHIP_ID) == CHIP_ID) {
	// 	return OK;
	// }

	// return -EIO;
	return OK;
}

ssize_t
SCR2100_X::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct gyro_x_report);
	struct gyro_x_report *grp = reinterpret_cast<struct gyro_x_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_reports->get(grp)) {
				ret += sizeof(*grp);
				grp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(grp)) {
		ret = sizeof(*grp);
	}

	return ret;
}

int
SCR2100_X::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				/* With internal low pass filters enabled, 250 Hz is sufficient */
				return ioctl(filp, SENSORIOCSPOLLRATE, 250);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 400) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);


					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement */
		return -EINVAL;

	case GYROIOCSSAMPLERATE:	/* sensor sample rate is not (really) adjustable */
		return -EINVAL;

	case GYROIOCGSAMPLERATE:
		return -EINVAL;	

	case GYROIOCSLOWPASS:
		// return set_lowpass(arg);
		return -EINVAL;

	case GYROIOCGLOWPASS:
		return -EINVAL;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		// return set_range(arg);
		return -EINVAL;

	case GYROIOCGRANGE:
		return _current_range;

	case GYROIOCSELFTEST:
		return OK;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}


uint32_t
SCR2100_X::send_request(uint32_t request)
{
	uint32_t response;
	uint8_t cmd[4];
	uint8_t resp[4];
	
	cmd[0] = (uint8_t)((request & 0xff000000 ) >> 24);
	cmd[1] = (uint8_t)((request & 0x00ff0000 ) >> 16);
	cmd[2] = (uint8_t)((request & 0x0000ff00 ) >> 8);
	cmd[3] = (uint8_t)( request & 0x000000ff );

	transfer(cmd, resp, sizeof(cmd));

	response = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) | ((uint32_t)resp[2] << 8) | ((uint32_t)resp[3]);
	/*NOTE: SPI TLH time must be at least 172 ns, now delay 1us*/
	up_udelay(1); 
	return response;
}

void
SCR2100_X::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_gyro_filter_x.set_cutoff_frequency(samplerate, bandwidth);
}

void
SCR2100_X::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 
				1000, 
				_call_interval, 
				(hrt_callout)&SCR2100_X::measure_trampoline, this);
}

void
SCR2100_X::stop()
{
	hrt_cancel(&_call);
}

void
SCR2100_X::measure_trampoline(void *arg)
{
	SCR2100_X *dev = (SCR2100_X *)arg;

	/* make another measurement */
	dev->measure();
}

void
SCR2100_X::measure()
{
	struct gyro_x_report report;

	/* start the performance counter */
	perf_begin(_sample_perf);

	memset(&report, 0, sizeof(report));
	
	/*
	 * Adjust and scale results to SI units.
	 *
	 * Note that we ignore the "new data" bits.  At any time we read, each
	 * of the axis measurements are the "most recent", even if we've seen
	 * them before.  There is no good way to synchronise with the internal
	 * measurement flow without using the external interrupt.
	 */
	report.timestamp = hrt_absolute_time();
	report.error_count = 0;

	/*
	 * y of board is x of sensor and x of board is -y of sensor
	 * perform only the axis assignment here.
	 */	
	report.temperature_raw = (send_request(REQ_READ_RATE) & DATA_FIELD_MASK) >> 8;	
	report.x_raw           = (send_request(REQ_READ_TEMP) & DATA_FIELD_MASK) >> 8;	
	
	/*SCR2100_X axis x is is negtive to PX4 sensor axis z*/
	// report.x_raw           = ((report.x_raw == -32768) ? 32767 : -report.x_raw);

	float xraw_f = report.x_raw;

	float xin = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;

	report.x = _gyro_filter_x.apply(xin);

	// math::Vector<3> aval(xin, yin, zin);
	// math::Vector<3> aval_integrated;

	// bool gyro_notify = _gyro_int.put(report.timestamp, aval, aval_integrated, report.integral_dt);
	// report.x_integral = aval_integrated(0);
	// report.y_integral = aval_integrated(1);
	// report.z_integral = aval_integrated(2);

	report.temperature = 60.0f + report.temperature_raw/14.5f;
	report.scaling     = _gyro_range_scale;
	report.range_rad_s  = _gyro_range_rad_s;

	/* return device ID */
	report.device_id = _device_id.devid;

	_reports->force(&report);

	// if (gyro_notify) {
	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	if (_gyro_topic != nullptr && !(_pub_blocked)) {
		orb_publish(ORB_ID(sensor_gyro_x), _gyro_topic, &report);
	}		
	// }

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
SCR2100_X::print_info()
{
	perf_print_counter(_sample_perf);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace scr2100_x
{

SCR2100_X	*g_dev;

void	start();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr) {
		errx(0, "already started");
	}

	/* create the driver */
	g_dev = new SCR2100_X(1, (spi_dev_e)PX4_SPIDEV_SCR_X);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SCR2100_X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_USER) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd = -1;
	struct gyro_x_report g_report;
	ssize_t sz;

	/* get the driver */
	fd = open(SCR2100_X_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'scr2100_x start' if the driver is not running)",
		    SCR2100_X_DEVICE_PATH);

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
		err(1, "reset to manual polling");
	}

	/* do a simple demand read */
	sz = read(fd, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		err(1, "immediate gyro read failed");
	}

	warnx("single read");
	warnx("time:     %lld", g_report.timestamp);
	warnx("gyro  x:  \t%8.4f\trad/s", (double)g_report.x);
	warnx("temperature:  \t%8.4f\tcels degree", (double)g_report.temperature);
	warnx("gyro  x:  \t%d\traw 0x%0x", (short)g_report.x_raw, (unsigned short)g_report.x_raw);
	warnx("temperature:  \t%d\traw 0x%0x", (short)g_report.temperature_raw, (unsigned short)g_report.temperature_raw);
	warnx("gyro range: %8.4f rad/s (%8.4f g)", (double)g_report.range_rad_s,
	      (double)((g_report.range_rad_s / M_PI_F) * 180.0f));

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_USER) < 0) {
		err(1, "reset to default polling");
	}

	// reset();
	close(fd);

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SCR2100_X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_USER) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "SCR2100_X: driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}


} // namespace

int
scr2100_x_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.

	 */
	if (!strcmp(argv[1], "start")) {
		scr2100_x::start();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		scr2100_x::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		scr2100_x::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info")) {
		scr2100_x::info();
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
