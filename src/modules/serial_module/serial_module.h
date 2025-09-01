#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

extern "C" __EXPORT int serial_module_main(int argc, char *argv[]);

class SerialModule : public ModuleBase<SerialModule>, public ModuleParams
{
public:
	SerialModule();
	~SerialModule() override = default;

	static int task_spawn(int argc, char *argv[]);
	static SerialModule *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	void parameters_update(bool force = false);

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	int _fd{-1};  // serial port file descriptor
};
