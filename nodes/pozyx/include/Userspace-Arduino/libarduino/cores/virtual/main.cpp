#include <virtual_main.h>
#include <time.h>

int main(void)
{
	clock_gettime(CLOCK_MONOTONIC, &prog_start_time);	
	init();
	setup();
	for (;;)
		loop();

	return 0;
}
