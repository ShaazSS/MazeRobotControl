/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-B06
 */

#include <xdc/std.h>

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle bluetoothIT;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle Timer50ms;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle pidSemaphore;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle pidTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle deadendRobot;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle Timer40ms;

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

