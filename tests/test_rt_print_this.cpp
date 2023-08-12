#include "tools/PrintThis.hpp"

#ifdef __XENO__

#include <sys/mman.h>
#include <native/task.h>
#include <rtdk.h> // Contains the definition of rt_printf()

void
run_rt(void * arg){

	PrintThis verbosity;

	int Nmax = 10000000;
	int i = 0;
	char buff[200];

	/* Perform auto-init of rt_print buffers if the task doesn't do so */
	rt_print_auto_init(1);

	rt_task_set_mode(0, T_WARNSW, NULL);

	while(true){

		rt_task_sleep(1000000LL);

		sprintf(buff,"[RT-Safe] This is my flap in vinegar 1989: %f\n",10*i/(double)Nmax);
		verbosity.print(buff);

		++i;
	}

}

#endif 

void
run(void){

	PrintThis verbosity;

	int Nmax = 10000000;
	int i = 0;
	char buff[200];

	while(i<Nmax){

		sprintf(buff,"This is my flap in vinegar 1989: %f\n",10*i/(double)Nmax);
		verbosity.print(buff);

		++i;
	}

	return;
}

int main(int argc, char** argv){

  #ifdef __XENO__

	  RT_TASK my_task;

  	// Declare Task:
    mlockall(MCL_CURRENT | MCL_FUTURE);

		rt_task_spawn(&my_task, "My_task", 0, 11, 0, run_rt, NULL);

	#else
	  run();
  #endif


  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// rtprint.c
///////////////////


// #include <stdio.h>
// #include <sys/mman.h>
// #include <native/task.h>
// #include <rtdk.h>

// void task2_func(void *arg)
// {
// 	int i = 0;

// 	rt_printf("This triggers auto-init of rt_print for the "
// 		  "calling thread.\n"
// 		  "A last switch to secondary mode can occure here, "
// 		  "but future invocations of rt_printf are safe.\n");

// 	rt_task_set_mode(0, T_WARNSW, NULL);

// 	while (1) {
// 		rt_task_sleep(3333333LL);
// 		rt_fprintf(stderr, "%s: #%d Yet another RT printer - "
// 			   "but to stderr.\n", rt_print_buffer_name(), ++i);
// 	}
// }

// int main(int argc, char **argv)
// {
// 	RT_TASK task1, task2;
// 	int i = 0;

// 	mlockall(MCL_CURRENT|MCL_FUTURE);

// 	/* Perform auto-init of rt_print buffers if the task doesn't do so */
// 	rt_print_auto_init(1);

// 	/* Initialise the rt_print buffer for this task explicitly */
// 	rt_print_init(4096, "Task 1");

// 	rt_task_shadow(&task1, "Task 1", 10, 0);
// 	rt_task_spawn(&task2, "Task 2", 0, 11, 0, task2_func, NULL);

// 	/* To demonstrate that rt_printf is safe */
// 	rt_task_set_mode(0, T_WARNSW, NULL);

// 	while (1) {
// 		rt_task_sleep(5000000LL);
// 		rt_printf("%s: #%d Hello RT world!\n",
// 			  rt_print_buffer_name(), ++i);
// 	}
// }
