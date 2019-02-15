// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without modification, are permitted (subject to the limitations in the disclaimer below) 
// provided that the following conditions are met:
// Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, 
// this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// Neither the name of Qualcomm Technologies, Inc. nor the names of its contributors may be used to endorse or promote products derived 
// from this software without specific prior written permission.
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
// BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Modified by Arm

#ifndef _OSAL_H_
#define _OSAL_H_

#include "a_types.h"
#include "athdefs.h"
#include <string.h>

#ifdef   _RTE_
#include "RTE_Components.h"
#endif

/*
 * Global macros
 */
#include "cmsis_os2.h"
#include <athdefs.h>

/* Priority setting */
/* Driver Task priority should be properly defined at compile time by the user */
#ifdef RTE_CMSIS_RTOS2
#ifndef QOSAL_DRIVER_TASK_PRIORITY
#define QOSAL_DRIVER_TASK_PRIORITY              (osPriorityAboveNormal)
#endif
#ifndef QOSAL_HIGHER_THAN_DRIVER_TASK_PRIORITY
#define QOSAL_HIGHER_THAN_DRIVER_TASK_PRIORITY  (QOSAL_DRIVER_TASK_PRIORITY + 1)
#endif
#endif

#define MUTEX_STRUCT            osSemaphoreId_t
#define QOSAL_EVENT_STRUCT      osEventFlagsId_t
#define QOSAL_NULL_TASK_ID      NULL
#define QOSAL_TICK_STRUCT       uint32_t
#define QOSAL_TICK_STRUCT_PTR   void*
#define QOSAL_TASK_ID           osThreadId_t
#define UNUSED_ARGUMENT(arg)    ((void)arg)
#define QOSAL_EVENT_AUTO_CLEAR  0x01
typedef struct time_struct
{
   /* The number of seconds in the time.  */
   uint32_t     SECONDS;
   /* The number of milliseconds in the time. */
   uint32_t     MILLISECONDS;
} TIME_STRUCT;

#define BSP_ALARM_FREQUENCY 1024

typedef osThreadId_t            qosal_task_handle;
typedef osEventFlagsId_t        *qosal_event_handle;
typedef osSemaphoreId_t         *qosal_mutex_handle;
typedef void*                   qosal_sema_handle;

/*
*This universal OS Layer function is used to start the OS
*/
/* Description:
 * This function is used to start the RTOS 
 * 
 * Params: None
 *
 * Returns: None
 */
A_VOID qosal_start(A_VOID);

/* 
 * Description: This function is used for creating an RTOS task 
 * 
 * Params: Task entry function
 *         Function arguments
 *         Task name
 *         Stack size 
 *         Task priority 
 *         Task handle
 *         Blocking Flag
 * Returns:  A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_task_create
                        (
                           A_VOID  Task(A_UINT32), 
                           char *task_name, 
                           int stack_size, A_VOID *param, 
                           unsigned long task_priority, 
                           qosal_task_handle *task_handle,
                           A_BOOL blocked  
                        );

/* 
 * Description: This function is used to get the current priority of the task 
 * 
 * Params: Task handle          
 *         Task priority 
 * 
 * Returns: A_OK if Success, A_ERROR if Fail 
 */
A_STATUS qosal_task_get_priority(qosal_task_handle task_handle, 
                                          A_UINT32 *priority_ptr);


/* 
 * Description: This function is used to set the current priority of the task 
 * 
 * Params: Task handle          
 *         Old priority 
 *         New priority
 * Returns: A_OK if Success, A_ERROR if Fail 
 */
A_STATUS qosal_task_set_priority(qosal_task_handle task_handle, 
                                          A_UINT32 new_priority, 
                                          A_VOID *priority_ptr);

/* 
 * Description: This function is used to get the current task ID
 * 
 * Params: None
 *          
 * Returns: Task handle
 */
qosal_task_handle qosal_task_get_handle(A_VOID);

/* 
 * Description: This function is used to destroy the task
 * 
 * Params: task_hanlde
 *          
 * Returns: A_OK on Success, A_ERROR on fail
 */
A_STATUS qosal_task_destroy(qosal_task_handle task_handle);

/* 
 * Description: This function is used to suspend the active task
 * 
 * Params: Task handle
 *          
 * Returns: None
 */
A_VOID qosal_task_suspend(qosal_task_handle *task_handle);

/* 
 * Description: This function is used to unblock the task
 * 
 * Params: Task handle
 *          
 * Returns: None
 */
A_VOID qosal_task_resume(qosal_task_handle *task_handle);

/******************************************************************************
 *
 * Memory Management APIs
 *
 *****************************************************************************/
/*
 * Description: This function is used for initializing the memory blocks
 *
 * Params: None
 *          
 * Returns: None
 */
A_VOID qosal_malloc_init(A_VOID);

/*
 * Description: This function is used to get the memory block size
 *
 * Params: Address, Size
 *          
 * Returns: Size of memory block 
 */
A_UINT32 qosal_get_size(A_VOID* addr);

/*
 * Description: This function is used for allocating the memory block of 
 * requested size
 *
 * Params: Size
 *          
 * Returns: Address of allocatated memory block
 */
A_VOID* qosal_malloc(A_UINT32 size);

/*
 * Description: This function is used for freeing of the memory block
 *
 * Params: Address
 *          
 * Returns: None
*/
/*Clear a memory pool*/
A_STATUS  qosal_free(A_VOID* addr);

#define QOSAL_MALLOC(size)           qosal_malloc(size)
#define QOSAL_FREE(addr)             qosal_free(addr)

/******************************************************************************
 * Timer Ticks APIs   
 *****************************************************************************/
/*
 * Description: This function is used for delay the OS task for specific time 
 * in milli secs
 *
 * Params: msec
 *          
 * Returns: None 
*/
A_VOID qosal_msec_delay(A_ULONG mSec);

/*
 * Description: This function is used for delay the OS task for specific time 
 * in micro secs
 *
 * Params: uSec
 *          
 * Returns: None 
*/
A_VOID qosal_usec_delay(A_UINT32 uSec);

/*
 * Description: This function is used to count timer ticks 
 *
 * Params: count
 *          
 * Returns: A_OK if Success, A_ERROR if Fail 
*/

A_STATUS qosal_time_get_ticks(A_UINT32 *count);


/*
 * Description: This function is used to get time per sec  
 *
 * Params: None
 *          
 * Returns: Time ticks per sec
 */
A_ULONG qosal_time_get_ticks_per_sec(A_VOID);

/*
 * Description: This function is used nvalidating all the data cache entries.
 *
 * Params: None
 *          
 * Returns: None
*/
A_VOID qosal_dcache_invalidate(A_VOID);

/*
 * Description: This function is used flushing the data cache.
 *
 * Params: None
 *          
 * Returns: None
*/
A_VOID qosal_dcache_flush(A_VOID);

/*
 * Description: This function is used for time delay 
 *
 * Params: Delay value
 *          
 * Returns: None
 */
A_VOID qosal_time_delay_ticks(A_ULONG val);

/******************************************************************************
*
* Interrupt Control APIs
*
******************************************************************************/
/*
 * Description: This function is used for disabling the external MCU interrupts
 *
 * Params: None
 *          
 * Returns: None
 */
A_VOID qosal_intr_disable (A_VOID);

/*
 * Description: This function is used for enabling the external MCU interrupts
 *
 * Params: None
 *          
 * Returns: None
 */
A_VOID qosal_intr_enable (A_VOID);


/*****************************************************************************
*
* Event Handling APIs
*
******************************************************************************/
/*
 * Description: This function is used for waiting for an event 
 *
 * Params: Event pointer, Bits to Wait for, Ticks to Wait for
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_UINT32  qosal_wait_for_event(qosal_event_handle event_ptr, 
                                             A_UINT32 bitsToWaitFor, 
                                             A_UINT32 all, 
                                             A_UINT32 var1, 
                                             A_UINT32 ticksToWait);
/*
 * Description: This function is used for set an event 
 *
 * Params: Event pointer, Bits to Set
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_set_event(qosal_event_handle event_ptr, A_UINT32 bitsToSet);


/*
 * Description: This function is used for creating an event
 *
 * Params: Event pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_create_event(qosal_event_handle event_ptr);

/*
 * Description: This function is used for setting auto clearing of event bits in event group
 *
 * Params: Event pointer, flags
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_set_event_auto_clear(qosal_event_handle event_ptr, A_UINT32 flags);

/*
 * Description: This function is used for clearing the event 
 *
 * Params: Event pointer, BitsToClear
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
*/
A_STATUS qosal_clear_event(qosal_event_handle event_ptr, A_UINT32 bitsToClear);

/*
 * Description:  This function is used for deleting an event   
 *
 * Params: Event pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_delete_event(qosal_event_handle event_ptr);


/*****************************************************************************
 *
 * Task Synchronization APIs (Mutex)
 *
 *****************************************************************************/
/*
 * Description: This function is used for initialization of the mutex 
 *
 * Params: Mutex pointer, Attr_ptr
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */

A_STATUS qosal_mutex_init(qosal_mutex_handle mutex_ptr);
                              

/*
 * Description: This function is used for aquiring the mutex lock
 *
 * Params: Mutex pointer, tick_count
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_mutex_acquire(qosal_mutex_handle mutex_lock, A_ULONG tick_count);

/*
 * Description: This function is used for releasing the mutex lock
 *
 * Params: Mutex pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_mutex_release(qosal_mutex_handle mutex_ptr);

/*
 * Description: This function is used for deleting the mutex 
 *
 * Params: Mutex pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail 
*/
A_STATUS qosal_mutex_destroy(qosal_mutex_handle mutex_ptr);

/*****************************************************************************
 *
 * Time delay APIs
 *
 *****************************************************************************/
/*
 * Description: This function is used to delay for given time ms 
 *
 * Params: msec
 *          
 * Returns: None
*/
A_VOID qosal_time_delay(A_UINT32 msec);

/*
 * Description: This function is used to get the elapsed time from tick time
 *
 * Params: ptr
 *          
 * Returns: None
*/
A_VOID qosal_time_get_elapsed(TIME_STRUCT* time);
   
/********************************************************************
*               Task Synchronization APIs (Semaphores)
********************************************************************/
//TBD 

A_STATUS qosal_sema_init(qosal_sema_handle *sem_ptr, 
                         A_UINT32 maxCount);
A_STATUS qosal_sema_get(qosal_sema_handle *sem_lock, A_ULONG tick_count);

A_STATUS qosal_sema_put(qosal_sema_handle *sem_lock);

A_STATUS qosal_sema_destroy(qosal_sema_handle *sem_lock);

#if 0 //TBD
   
/*********************************************************************
*                  Kernel Log APIs
*********************************************************************/  

A_UINT32 qosal_klog_create(A_UINT32 size, A_UINT32 flags);   
A_VOID qosal_klog_control(A_UINT32 size ,A_BOOL flags);
A_BOOL qosal_klog_dispaly(A_VOID);
A_UINT32 qosal_klog_create_at(A_UINT32 max_size, A_UINT32 flags, A_VOID* ptr);
/*****************************************************************************/
#endif
#endif          //_OSAL_H_
