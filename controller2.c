//------------------- CONTROLLER_2.C ---------------------- 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_msg.h>
#include <rtai_mbx.h>
#include <rtai_netrpc.h>
#include <sys/io.h>
#include <signal.h>
#include "parameters.h"
#define CPUMAP 0x1

static int state_1 = 0;
static int state_2 = 0;

static RT_TASK *main_Task2;
static RT_TASK *read_Task2;
static RT_TASK *filter_Task2;
static RT_TASK *control_Task2;
static RT_TASK *write_Task2;

static int keep_on_running = 1;

static pthread_t read_thread2;
static pthread_t filter_thread2;
static pthread_t control_thread2;
static pthread_t write_thread2;
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
MBX* msg_1;
MBX* msg_2;

static void * acquire_loop(void * par) {
	
	if (!(read_Task2 = rt_task_init_schmod(nam2num("2 READER"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK 2\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(read_Task2, expected, sampl_interv);
	rt_make_hard_real_time();

	while (keep_on_running)
	{
		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail);
		
		buffer2[head] = (sensor[1]);
		head = (head+1) % BUF_SIZE;

		rt_sem_signal(meas_avail);

		rt_task_wait_period();
	}
	rt_task_delete(read_Task2);
	return 0;
}

static void * filter_loop(void * par) {

	if (!(filter_Task2 = rt_task_init_schmod(nam2num("FLTR 2"), 2, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK 2\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task2, expected, sampl_interv);
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
			rt_send(control_Task2, avg);		
		}
		rt_task_wait_period();
	}
	rt_task_delete(filter_Task2);
	return 0;
}

static void * control_loop(void * par) {

	if (!(control_Task2 = rt_task_init_schmod(nam2num("CTRL 2"), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK 2\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	RTIME delay = nano2count(DELAY_TIME);
	rt_task_make_periodic(control_Task2, expected, BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();

	unsigned int plant_state = 0;
	unsigned int prev_plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	while (keep_on_running)
	{
		// receiving the average plant state from the filter
		rt_receive(filter_Task2, &plant_state);
		
		if(*reference == 0){
			// rilevazione del bloccaggio
			if((plant_state == prev_plant_state) && (plant_state != 0)){
				state_2 = 1;
			} else state_2 = 0;
			
			rt_mbx_receive_timed(msg_1, (void *)&state_1, sizeof(int), delay);
			
			rt_mbx_send_if(msg_2, (void *)&state_2, sizeof(int));
			
			//printf("stato1: %d\tstato2: %d\n", stato_1, stato_2);
			
			// se una delle due ruote è bloccata, mollo il freno su entrambe
			if((state_1 == 1) || (state_2 == 1) || (plant_state == 0)) control_action = 3;
			else control_action = 4;
		} else{
			// computation of the control law
			error = (*reference) - plant_state;
		
			if (error > 0) control_action = 1;
			else if (error < 0) control_action = 2;
			else control_action = 3;
		}
		prev_plant_state = plant_state;
		state_1 = 0;
		state_2 = 0;
		
		// sending the control action to the actuator
		rt_send(write_Task2, control_action);

		rt_task_wait_period();

	}
	rt_task_delete(control_Task2);
	return 0;
}

static void * actuator_loop(void * par) {

	if (!(write_Task2 = rt_task_init_schmod(nam2num("2 WRITE"), 4, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK 2\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task2, expected, BUF_SIZE*sampl_interv);
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
			case 4: cntr = -2; break;
			default: cntr = 0;
		}
		
		(actuator[1]) = cntr;

		rt_task_wait_period();
	}
	rt_task_delete(write_Task2);
	return 0;
}

int main(void)
{
	printf("The controller 2 is STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task2 = rt_task_init_schmod(nam2num("MNTK2"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK 2\n");
		exit(1);
	}

	//attach to data shared with the controller
	sensor = rtai_malloc(SEN_SHM, NUM_OF_WHEELS*sizeof(int));
	actuator = rtai_malloc(ACT_SHM, NUM_OF_WHEELS*sizeof(int));
	reference = rtai_malloc(REFSENS, sizeof(int));

	(*reference) = 110;

	space_avail = rt_typed_sem_init(SPACE_SEM_2, BUF_SIZE, CNT_SEM | PRIO_Q);
	meas_avail = rt_typed_sem_init(MEAS_SEM_2, 0, CNT_SEM | PRIO_Q);
	msg_1 = rt_typed_named_mbx_init("msg1", MSG_SIZE, PRIO_Q);
	msg_2 = rt_typed_named_mbx_init("msg2", MSG_SIZE, PRIO_Q);
	
	sampl_interv = nano2count(CNTRL_TIME);
	
	// CONTROL THREADS 
	pthread_create(&read_thread2, NULL, acquire_loop, NULL);
	pthread_create(&filter_thread2, NULL, filter_loop, NULL);
	pthread_create(&control_thread2, NULL, control_loop, NULL);
	pthread_create(&write_thread2, NULL, actuator_loop, NULL);

	while (keep_on_running) {
		//printf("Control: %d\tStato1: %d\tStato2: %d\n",(actuator[1]), stato_1, stato_2);
		printf("Control: %d\n", (actuator[1]));
		rt_sleep(500000000);
	}

	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);
	rt_sem_delete(meas_avail);
	rt_sem_delete(space_avail);
	rt_named_mbx_delete(msg_1);
	rt_named_mbx_delete(msg_2);
	rt_task_delete(main_Task2);
 	printf("The controller 2 is STOPPED\n");
	return 0;
}