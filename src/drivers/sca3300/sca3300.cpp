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
 * @file sca3300.cpp
 * Driver for the murata SCA3300 MEMS accelerometer connected via SPI.
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
#include <drivers/drv_accel.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#define SCA3300_DEVICE_PATH	"/dev/sca3300"

/*accelerometer sensitivities*/
// #define ACC_SENSITIVITY_1_5G	5400
// #define ACC_SENSITIVITY_3G		2700
// #define ACC_SENSITIVITY_6G		1350

/*Standard requests*/
#define REQ_READ_ACC_X		0x040000F7
#define REQ_READ_ACC_Y		0x080000FD
#define REQ_READ_ACC_Z		0x0C0000FB
#define REQ_READ_STO		0x100000E9
#define REQ_READ_TEMP		0x140000EF
#define REQ_READ_STAT_SUM 	0x180000E5
#define REQ_WRITE_SW_RST	0xB4002098
#define REQ_WRITE_MODE1		0xB400001F
#define REQ_WRITE_MODE2		0xB4000102
#define REQ_WRITE_MODE3		0xB4000225
#define REQ_WRITE_MODE4		0xB4000338
#define REQ_READ_WHOAMI		0x40000091

/*used for engineering sample chip*/
#define REQ_WRITE_10HZ_ACT	0xF00242C8

/*Frame field masks*/
#define OPCODE_FIELD_MASK	0xFC000000
#define RS_FIELD_MASK		0x03000000
#define DATA_FIELD_MASK		0x00FFFF00
#define CRC_FIELD_MASK		0x000000FF

/*return status definition*/
#define RS_STARTUP			0
#define RS_NO_ERROR			1
#define RS_ERROR			3

/*set user sensor poll rate, according to datasheet should be set 2000Hz*/
#define SENSOR_POLLRATE_USER			1000
#define SCA3300_DEFAULT_FILTER_FREQ		200
#define SCA3300_MAX_OUTPUT_RATE			1000

extern "C" { __EXPORT int sca3300_main(int argc, char *argv[]); }

class SCA3300 : public device::SPI
{
public:
	SCA3300(int bus, spi_dev_e device);
	virtual ~SCA3300();

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

	struct accel_calibration_s	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int			_class_instance;

	unsigned		_current_range;

	perf_counter_t		_sample_perf;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

	Integrator		_accel_int;

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

SCA3300::SCA3300(int bus, spi_dev_e device) :
	SPI("SCA3300", SCA3300_DEVICE_PATH, bus, device, SPIDEV_MODE0, 8000000),
	_call_interval(0),
	_reports(nullptr),
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_class_instance(-1),
	_current_range(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "sca3300_read")),
	_accel_filter_x(SENSOR_POLLRATE_USER, SCA3300_DEFAULT_FILTER_FREQ),
	_accel_filter_y(SENSOR_POLLRATE_USER, SCA3300_DEFAULT_FILTER_FREQ),
	_accel_filter_z(SENSOR_POLLRATE_USER, SCA3300_DEFAULT_FILTER_FREQ),
	_accel_int(1000000 / SCA3300_MAX_OUTPUT_RATE, true)
{
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_SCA3300;

	// enable debug() calls
	_debug_enabled = true;

	// default scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;
}

SCA3300::~SCA3300()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
}

int
SCA3300::init()
{
	int ret = PX4_ERROR;
	uint8_t rsdata;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);
	/* Wait (10ms) for memory reading and signal path settling. */
	usleep(10000);

	/* Send SPI command 0xF00242C8 to activate 10Hz in mode4*/
	send_request(REQ_WRITE_10HZ_ACT);

	/* Select operating mode2, ±6g range*/
	send_request(REQ_WRITE_MODE2);
	/* set new range scaling factor */
	_current_range = 6;
	_accel_range_m_s2 = _current_range * 9.80665f;
	_accel_range_scale = _accel_range_m_s2 / 8100.0f;

	/* Wait (5ms) for signal path settling */
	usleep(5000);

	/*Read status (0x180000E5) two times to clear error bits occurred during mode setting.*/
	send_request(REQ_READ_STAT_SUM);
	send_request(REQ_READ_STAT_SUM);

	/*Read status (0x180000E5) again and ensure that all errors are cleared*/
	send_request(REQ_READ_STAT_SUM);

	rsdata = (send_request(REQ_READ_ACC_X) & RS_FIELD_MASK) >> 24;

	/*if Off frame protocol RS bits =01b, then start-up successful */
	if (rsdata == RS_NO_ERROR) {
		ret = OK;
	} else {
		ret = PX4_ERROR;
	}

	/* advertise sensor topic, measure manually to initialize valid report */

	measure();

	struct accel_report arp;
	_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise(ORB_ID(sensor_accel), &arp);

	if (_accel_topic == nullptr) {
		DEVICE_DEBUG("failed to create sensor_accel publication");
	}

	// ret = OK;

out:
	return ret;
}

int
SCA3300::probe()
{
	/* dummy read to ensure SPI state machine is sane */
	send_request(REQ_READ_WHOAMI);

	// if (read_reg(ADDR_CHIP_ID) == CHIP_ID) {
	// 	return OK;
	// }

	// return -EIO;
	return OK;
}

ssize_t
SCA3300::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct accel_report);
	struct accel_report *arp = reinterpret_cast<struct accel_report *>(buffer);
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
			if (_reports->get(arp)) {
				ret += sizeof(*arp);
				arp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(arp)) {
		ret = sizeof(*arp);
	}

	return ret;
}

int
SCA3300::ioctl(struct file *filp, int cmd, unsigned long arg)
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

					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
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

	case ACCELIOCSSAMPLERATE:	/* sensor sample rate is not (really) adjustable */
		return -EINVAL;

	case ACCELIOCGSAMPLERATE:
		return -EINVAL;	

	case ACCELIOCSLOWPASS:
		// return set_lowpass(arg);
		return -EINVAL;

	case ACCELIOCGLOWPASS:
		return -EINVAL;

	case ACCELIOCSSCALE:
		/* copy scale in */
		memcpy(&_accel_scale, (struct accel_calibration_s *) arg, sizeof(_accel_scale));
		return OK;

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		// return set_range(arg);
		return -EINVAL;

	case ACCELIOCGRANGE:
		return _current_range;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}


uint32_t
SCA3300::send_request(uint32_t request)
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
	/*NOTE: SPI TLH time must be at least 10 us*/
	up_udelay(10); 
	return response;
}

void
SCA3300::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
SCA3300::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 
				1000, 
				_call_interval, 
				(hrt_callout)&SCA3300::measure_trampoline, this);
}

void
SCA3300::stop()
{
	hrt_cancel(&_call);
}

void
SCA3300::measure_trampoline(void *arg)
{
	SCA3300 *dev = (SCA3300 *)arg;

	/* make another measurement */
	dev->measure();
}

void
SCA3300::measure()
{
	struct accel_report report;

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
	report.x_raw           = (send_request(REQ_READ_ACC_Y) & DATA_FIELD_MASK) >> 8;	
	report.y_raw           = (send_request(REQ_READ_ACC_Z) & DATA_FIELD_MASK) >> 8;
	report.z_raw           = (send_request(REQ_READ_TEMP) & DATA_FIELD_MASK) >> 8;
	report.temperature_raw = (send_request(REQ_READ_ACC_X) & DATA_FIELD_MASK) >> 8;	
	
	/**
	 * SCA3300 axis x is is negtive to PX4 sensor axis x
	 * SCA3300 axis y is is negtive to PX4 sensor axis y
	 * SCA3300 axis z is is negtive to PX4 sensor axis z
	 */
	report.x_raw           = ((report.x_raw == -32768) ? 32767 : -report.x_raw);
	report.y_raw           = ((report.y_raw == -32768) ? 32767 : -report.y_raw);
	report.z_raw           = ((report.z_raw == -32768) ? 32767 : -report.z_raw);

	float xraw_f = report.x_raw;
	float yraw_f = report.y_raw;
	float zraw_f = report.z_raw;

	float xin = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float yin = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float zin = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	report.x = _accel_filter_x.apply(xin);
	report.y = _accel_filter_y.apply(yin);
	report.z = _accel_filter_z.apply(zin);

	// math::Vector<3> aval(xin, yin, zin);
	// math::Vector<3> aval_integrated;

	// bool accel_notify = _accel_int.put(report.timestamp, aval, aval_integrated, report.integral_dt);
	// report.x_integral = aval_integrated(0);
	// report.y_integral = aval_integrated(1);
	// report.z_integral = aval_integrated(2);

	report.temperature = -273.0f + report.temperature_raw/18.9f;
	report.scaling     = _accel_range_scale;
	report.range_m_s2  = _accel_range_m_s2;

	/* return device ID */
	report.device_id = _device_id.devid;

	_reports->force(&report);

	// if (accel_notify) {
		/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	if (_accel_topic != nullptr && !(_pub_blocked)) {
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &report);
	}		
	// }

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
SCA3300::print_info()
{
	perf_print_counter(_sample_perf);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace sca3300
{

SCA3300	*g_dev;

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
	g_dev = new SCA3300(1, (spi_dev_e)PX4_SPIDEV_SCA);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SCA3300_DEVICE_PATH, O_RDONLY);

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
	struct accel_report a_report;
	ssize_t sz;

	/* get the driver */
	fd = open(SCA3300_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'sca3300 start' if the driver is not running)",
		    SCA3300_DEVICE_PATH);

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
		err(1, "reset to manual polling");
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		err(1, "immediate acc read failed");
	}

	warnx("single read");
	warnx("time:     %lld", a_report.timestamp);
	warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
	warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
	warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
	warnx("temperature:  \t%8.4f\tcels degree", (double)a_report.temperature);
	warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
	warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
	warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
	warnx("temperature:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);
	warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
	      (double)(a_report.range_m_s2 / 9.81f));

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
	int fd = open(SCA3300_DEVICE_PATH, O_RDONLY);

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
		errx(1, "SCA3300: driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}


} // namespace

int
sca3300_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.

	 */
	if (!strcmp(argv[1], "start")) {
		sca3300::start();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		sca3300::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		sca3300::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info")) {
		sca3300::info();
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
