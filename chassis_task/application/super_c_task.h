#ifndef SUPER_C_TASK_H
#define SUPER_C_TASK_H
#include "pid.h"
extern void super_c_task(void const * arguement);

typedef struct superC{
	fp32 cap_percent ;
	
}cap_measure_t;
extern cap_measure_t cap_measure; 
#endif
