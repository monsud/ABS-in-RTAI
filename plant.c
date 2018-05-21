//------------------- PLANT.C ---------------------- 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <sys/io.h>
#include <signal.h>
#include "parameters.h"
#define CPUMAP 0x1

//emulates the plant to be controlled
//if the control is -2, a break (with skating) is emulated

static RT_TASK *main_Task;
static RT_TASK *loop_Task;
static int keep_on_running = 1;

static pthread_t wheel_thr[NUM_OF_WHEELS];
static RTIME expected;
static RTIME sampl_interv;

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;

static void * wheel_loop(void * par) {

	int i = (int) par;
	
	if (!(loop_Task = rt_task_init_schmod(nam2num("WHEEL")+i, 2+i, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT PERIODIC TASK\n");
		exit(1);
	}

	unsigned int iseed = (unsigned int)rt_get_time();
  	srand (iseed);

	sensor[i] = 100; actuator[i] = 0;
	expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(loop_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	int count = 1;
	int noise = 0;
	long brake = 0; // braking counter
	int skate = 0;
	int when_to_decrease = 1 + (int)( 10.0 * rand() / ( RAND_MAX + 1.0 ));
	while (keep_on_running)
	{
		if (skate<2) { // if skating, no decrease nor noise added
			if (count%when_to_decrease==0) {
				if (sensor[i]>0) sensor[i]--; // the sensed data smoothly decreases...
				when_to_decrease = 1 + (int)( 10.0 * rand() / ( RAND_MAX + 1.0 ));
				count = when_to_decrease;
			}
			// add noise between -1 and 1
			if (count%3 == 0) {	
				noise = -1 + (int)(3.0 * rand() / ( RAND_MAX + 1.0 ));
				if (sensor[i]>0) sensor[i] += noise;
			}
		}
		
		// reaction to control
		if (count%2 == 0) {
			if (actuator[i] == 0) {
				brake = 0; skate = 0; }			
			else if (actuator[i] == 1) {
				sensor[i]++; brake = 0; skate = 0;  }
			else if ((actuator[i] == -1) && (sensor[i]>0)) {
				sensor[i]--; brake = 0; skate = 0;  }
			else if ((actuator[i] == -2) && (sensor[i]>0)) { 
				// emulates braking and skating
				brake++;
				if (brake < 10) { // at the beginning, brake takes effect
					if(sensor[i]>2) sensor[i]-=3; else sensor[i]=0;
				} else { //after, skating begins
					skate = (int)( 10.0 * rand() / ( RAND_MAX + 1.0 ));
					if (skate < 2) { 
						if(sensor[i]>2) sensor[i]-=3; 
						else sensor[i]=0;
					}
				}
			}
		}
		count++;
		rt_task_wait_period();
	}
	rt_task_delete(loop_Task);
	return 0;
}

int main(void)
{
	printf("The plant STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MNTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//attach to data shared with the controller
	sensor = rtai_malloc(SEN_SHM, NUM_OF_WHEELS*sizeof(int));
	actuator = rtai_malloc(ACT_SHM, NUM_OF_WHEELS*sizeof(int));

	sampl_interv = nano2count(TICK_TIME);
	int i;
	for (i=0; i < NUM_OF_WHEELS; i++) {
		pthread_create(&wheel_thr[i], NULL, wheel_loop, (void*)i);
	}
	while (keep_on_running) {
		for (i=0; i < NUM_OF_WHEELS; i++) {
			printf("Wheel %d: %d\t",i,sensor[i]);
		}
		printf("\n");
		rt_sleep(10000000);
	}

	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_task_delete(main_Task);
 	printf("The Plant is STOPPED\n");
	return 0;
}




