/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/log.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	// PX4_INFO("Hello Sky!");
	// return OK;

	PX4_INFO("Serial Reader module started!");

	for (int i = 0; i < 5; i++) {
		PX4_INFO("Loop iteration: %d", i);
		px4_usleep(500000); // 0.5 seconds
	}

	PX4_INFO("Serial Reader module finished.");
	return 0;

}
