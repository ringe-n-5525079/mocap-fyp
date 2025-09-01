#include "serial_module.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <termios.h>
#include <errno.h>

SerialModule::SerialModule()
    : ModuleParams(nullptr)   // <-- initialize ModuleParams
{
}

int SerialModule::print_status()
{
	PX4_INFO("Running, fd=%d", _fd);
	return 0;
}

int SerialModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SerialModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("serial_module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SerialModule *SerialModule::instantiate(int argc, char *argv[])
{
	return new SerialModule();
}

void SerialModule::run()
{
	// Open Telem2 (ttyS2)
	_fd = ::open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("Failed to open /dev/ttyS2 (%d)", errno);
		return;
	}

	// Configure baud rate
	struct termios config;

	if (tcgetattr(_fd, &config) < 0) {
		PX4_ERR("tcgetattr failed");
		::close(_fd);
		return;
	}

	cfsetispeed(&config, B115200);
	cfsetospeed(&config, B115200);
	config.c_cflag |= (CLOCAL | CREAD);   // enable receiver
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;                // 8 data bits
	config.c_cflag &= ~PARENB;            // no parity
	config.c_cflag &= ~CSTOPB;            // 1 stop bit
	config.c_cflag &= ~CRTSCTS;           // no flow control
	config.c_lflag = 0;                   // raw input
	config.c_oflag = 0;                   // raw output
	config.c_iflag = 0;

	tcflush(_fd, TCIFLUSH);

	if (tcsetattr(_fd, TCSANOW, &config) < 0) {
		PX4_ERR("tcsetattr failed");
		::close(_fd);
		return;
	}

	PX4_INFO("Serial port /dev/ttyS2 opened at 115200 baud");

	char buf[128];

	while (!should_exit()) {
		int n = ::read(_fd, buf, sizeof(buf) - 1);

		if (n > 0) {
			buf[n] = '\0';  // null terminate
			PX4_INFO("Recv: %s", buf);
		}

		px4_usleep(10000); // 10 ms
	}

	::close(_fd);
	_fd = -1;
}

void SerialModule::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

int SerialModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Reads and prints serial data from Telem2 (/dev/ttyS2) at 115200 baud.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("serial_module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int serial_module_main(int argc, char *argv[])
{
	return SerialModule::main(argc, argv);
}
