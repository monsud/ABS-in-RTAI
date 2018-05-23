//------------------- CONTROLLER.C ---------------------- 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_msg.h>
#include <sys/io.h>
#include <signal.h>
#include "parameters.h"
#define CPUMAP 0x1

//emulates the plant to be controlled

static RT_TASK *main_Task;
static RT_TASK *read_Task;
static RT_TASK *filter_Task;
static RT_TASK *control_Task;
static RT_TASK *write_Task;
static int keep_on_running = 1;

static pthread_t read_thread;
static pthread_t filter_thread;
static pthread_t control_thread;
static pthread_t write_thread;
static RTIME sampl_interv;

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;
int* reference;

int buffer2[BUF_SIZE];
int head = 0;
int tail = 0;

SEM* space_avail;
SEM* meas_avail;


static void * acquire_loop2(void * par) {
	
	if (!(read_Task = rt_task_init_schmod(nam2num("READER"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(read_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	while (keep_on_running)
	{
		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail);
		
		buffer2[head] = sensor[1];
		head = (head+1) % BUF_SIZE;

		rt_sem_signal(meas_avail);

		rt_task_wait_period();
	}
	rt_task_delete(read_Task);
	return 0;
}

static void * filter_loop2(void * par) {

	if (!(filter_Task = rt_task_init_schmod(nam2num("FILTER"), 2, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	int cnt = BUF_SIZE;
	unsigned int sum = 0;
	unsigned int avg = 0;
	while (keep_on_running)
	{
		// FILTERING (average)
		rt_sem_wait(meas_avail);

		sum += buffer2[tail];
		tail = (tail+1) % BUF_SIZE;

		rt_sem_signal(space_avail);
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg = sum/BUF_SIZE;
			sum = 0;
			// sends the average measure to the controller
			rt_send(control_Task, avg);		
		}
		rt_task_wait_period();
	}
	rt_task_delete(filter_Task);
	return 0;
}

static void * control_loop2(void * par) {

	if (!(control_Task = rt_task_init_schmod(nam2num("CONTROL"), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task, expected, BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();

	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	while (keep_on_running)
	{
		// receiving the average plant state from the filter
		rt_receive(0, &plant_state);

		// computation of the control law
		error = (*reference) - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0){
			if ((*reference) == 0) control_action = 4; //ABS
			else control_action = 2;
			}
		else control_action = 3;

		// sending the control action to the actuator
		rt_send(write_Task, control_action);

		rt_task_wait_period();

	}
	rt_task_delete(control_Task);
	return 0;
}

static void * actuator_loop2(void * par) {

	if (!(write_Task = rt_task_init_schmod(nam2num("WRITE"), 4, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task, expected, BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();

	unsigned int control_action = 0;
	int cntr = 0;
	while (keep_on_running)
	{
		// receiving the control action from the controller
		rt_receive(0, &control_action);
		
		switch (control_action) {
			case 1: cntr = 1; break;
			case 2:	cntr = -1; break;
			case 3:	cntr = 0; break;
			case 4: cntr = -2; break; //implemento la frenata
			default: cntr = 0;
		}
		
		actuator[1] = cntr;

		rt_task_wait_period();
	}
	rt_task_delete(write_Task);
	return 0;
}

int main(void)
{
	printf("The controller is STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MAINTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//attach to data shared with the controller
	sensor = rtai_malloc(SEN_SHM, sizeof(int));
	actuator = rtai_malloc(ACT_SHM, sizeof(int));
	reference = rtai_malloc(REFSENS, sizeof(int));

	(*reference) = 110;

	space_avail = rt_typed_sem_init(SPACE_SEM, BUF_SIZE, CNT_SEM | PRIO_Q);
	meas_avail = rt_typed_sem_init(MEAS_SEM, 0, CNT_SEM | PRIO_Q);
	
	sampl_interv = nano2count(CNTRL_TIME);
	
	// CONTROL THREADS 
	pthread_create(&read_thread, NULL, acquire_loop2, NULL);
	pthread_create(&filter_thread, NULL, filter_loop2, NULL);
	//for (int i=0;i<NUM_WHEELS;i++){
		pthread_create(&control_thread, NULL, control_loop2, NULL);
	//}
	//pthread_create(&control_thread2, NULL, control_loop, NULL);
	pthread_create(&write_thread, NULL, actuator_loop2, NULL);

	while (keep_on_running) {
		//for (int i=0; i < NUM_OF_WHEELS; i++) {
			printf("Control 1: %d\n",actuator[1]);
		//}
		rt_sleep(10000000);
	}

	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);
	rt_sem_delete(meas_avail);
	rt_sem_delete(space_avail);
	rt_task_delete(main_Task);
 	printf("The controller is STOPPED\n");
	return 0;
}




