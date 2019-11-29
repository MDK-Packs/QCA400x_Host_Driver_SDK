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
//
// Modified by Arm

#include <osal.h>

#if defined(RTE_CMSIS_RTOS2)

#include "stdlib.h"

static qosal_task_handle DriverWakeEvent = NULL; // Needded for Auto Clear

/******************************************************************************/
/* OS Layer Wrapper function implementation */
/******************************************************************************/
/****************
 * OS Task API's
 ****************/

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
A_VOID qosal_start() {
}

/* Description:
 * This function is used to get the OS return/error code
 * 
 * Params: OS error code
 *
 * Returns: A_OK if Success, A_ERROR if Fail 
 */
A_STATUS qosal_get_error_code(A_UINT32 os_err) {
  return A_OK;
}


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
A_STATUS qosal_task_create ( A_VOID  Task(A_UINT32),
                             char *task_name,
                             int stack_size, A_VOID *param,
                             unsigned long task_priority,
                             qosal_task_handle *task_handle,
                             A_BOOL auto_start) {
  A_STATUS       osal_ret = A_OK;
  osThreadAttr_t thread_attr;

  if (task_priority <= (unsigned long)(osPriorityRealtime7)) {
    thread_attr.name       = task_name;
    thread_attr.attr_bits  = 0;
    thread_attr.cb_mem     = NULL;
    thread_attr.cb_size    = 0;
    thread_attr.stack_mem  = NULL;
    thread_attr.stack_size = stack_size;
    thread_attr.priority   = (osPriority_t)task_priority;
    thread_attr.tz_module  = 0;

    *task_handle = osThreadNew((osThreadFunc_t)Task, param, &thread_attr);
    if(*task_handle == NULL) {
      osal_ret = A_ERROR;
    }
  }
  return osal_ret;
}

/* 
 * Description: This function is used to get the current priority of the task 
 * 
 * Params: Task handle          
 *         Task priority 
 * 
 * Returns: A_OK if Success, A_ERROR if Fail 
 */
A_STATUS qosal_task_get_priority(qosal_task_handle task_handle,
                                         A_UINT32 *priority_ptr) {
  A_STATUS osal_ret = A_OK;
  osPriority_t priority;

  priority = osThreadGetPriority (task_handle);
  if(priority == osPriorityError) {
    osal_ret = A_ERROR;
  }
  *priority_ptr = (A_UINT32)priority;

  return osal_ret;
}

/* 
 * Description: This function is used to set the current priority of the task 
 * 
 * Params: Task handle          
 *         Old priority 
 *         New priority
 * Returns: A_OK if Success, A_ERROR if Fail 
 */
A_STATUS qosal_task_set_priority(qosal_task_handle task_handle,
                                     A_UINT32      new_priority,
                                     A_VOID       *priority_ptr) {
  A_STATUS osal_ret = A_OK;
  osPriority_t priority;

  priority = osThreadGetPriority (task_handle);
  if(priority == osPriorityError) {
   osal_ret = A_ERROR;
  }
  *((A_UINT32 *)priority_ptr) = priority;

  if (new_priority <= (unsigned long)(osPriorityRealtime7)) {
    if(osThreadSetPriority (task_handle, (osPriority_t)new_priority) != osOK) {
      osal_ret = A_ERROR;
    }
  } else {
    osal_ret = A_ERROR;
  }
  return osal_ret;
}

/* 
 * Description: This function is used to get the current task ID
 * 
 * Params: None
 *          
 * Returns: Task handle
 */
qosal_task_handle qosal_task_get_handle(A_VOID) {
  return osThreadGetId();
}

/* 
 * Description: This function is used to destroy the task
 * 
 * Params: task_handle
 *          
 * Returns: A_OK if Success, A_ERROR if Fail 
 */
A_STATUS qosal_task_destroy(qosal_task_handle task_handle) {
  A_STATUS osal_ret = A_OK;

  if (osThreadTerminate(task_handle) != osOK) {
    osal_ret = A_ERROR;
  }
  return osal_ret;
}

/* 
 * Description: This function is used to suspend the active task
 * 
 * Params: Task handle
 *          
 * Returns: None  
 */
A_VOID qosal_task_suspend(qosal_task_handle *task_handle) {
  osThreadSuspend(task_handle);
}
   
/* 
 * Description: This function is used to unblock the task
 * 
 * Params: Task handle
 *          
 * Returns: None 
 */

A_VOID qosal_task_resume(qosal_task_handle *task_handle) {
  osThreadResume(task_handle);
}

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
A_VOID qosal_malloc_init(A_VOID) {
}

/*
 * Description: This function is used to get the memory block size
 *
 * Params: Address, Size
 *          
 * Returns: Size of memory block 
 */
A_UINT32 qosal_get_size(A_VOID* addr) {
  return 0;
}

/*
 * Description: This function is used for allocating the memory block of 
 * requested size
 *
 * Params: Size
 *          
 * Returns: Address of allocatated memory block
 */
A_VOID* qosal_malloc(A_UINT32 size) {
  return malloc(size);
}

/*
 * Description: This function is used for freeing of the memory block
 *
 * Params: Address
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
*/
/*Clear a memory pool*/
A_STATUS  qosal_free(A_VOID* addr) {
    free(addr);
    return A_OK;
}

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
A_VOID qosal_msec_delay(A_ULONG mSec) {
  osDelay(mSec);
  return;
}

/*
 * Description: This function is used for delay the OS task for specific time 
 * in micro secs
 *
 * Params: uSec
 *          
 * Returns: None 
*/

A_VOID qosal_usec_delay(A_UINT32 uSec) {
  uint32_t count_delay;
  uint32_t ms_delay;
  uint32_t start;

  if (uSec < 1000) {
    count_delay = (osKernelGetSysTimerFreq() / 1000000) * uSec;
    start = osKernelGetSysTimerCount();
    while ((osKernelGetSysTimerCount() - start) < count_delay);
  } else {
    ms_delay = uSec / 1000;
    if (uSec - (ms_delay * 1000) > 500) {
      ms_delay++;
    }
    osDelay(ms_delay);
  }
  return;
}

/*
 * Description: This function is used to get absolute time in ticks 
 *
 * Params: count
 *          
 * Returns: A_OK if Success, A_ERROR if Fail 
 */

A_STATUS qosal_time_get_ticks(A_UINT32 *count) {
  *count = osKernelGetSysTimerCount();
  return A_OK;
}

/*
 * Description: This function is used to get time per sec  
 *
 * Params: None
 *          
 * Returns: Time ticks per sec
 */
A_ULONG qosal_time_get_ticks_per_sec(A_VOID) {
  return osKernelGetSysTimerFreq ();
}

/*
 * Description: This function is used flushing the data cache.
 *
 * Params: None
 *          
 * Returns: None
*/
A_VOID qosal_dcache_flush(A_VOID) {
}

/*
 * Description: This function is used nvalidating all the data cache entries.
 *
 * Params: None
 *          
 * Returns: None
*/
A_VOID qosal_dcache_invalidate(A_VOID) {
}
  
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
A_VOID qosal_intr_disable (A_VOID) {
//  __disable_irq();
}

/*
 * Description: This function is used for enabling the external MCU interrupts
 *
 * Params: None
 *          
 * Returns: None
 */
A_VOID qosal_intr_enable (A_VOID) {
//  __enable_irq();
}

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

A_UINT32 qosal_wait_for_event(qosal_event_handle event_ptr,
                                        A_UINT32 bitsToWaitFor, 
                                        A_UINT32 all, 
                                        A_UINT32 var1, 
                                        A_UINT32 ticksToWait) {
  A_UINT32 osal_ret = A_OK;
  A_UINT32 opt;

  if(all) {
    opt = osFlagsWaitAll;
  } else {
    opt = osFlagsWaitAny;
  }
  if (event_ptr != DriverWakeEvent) {
    opt |= osFlagsNoClear;
  }

  if (osEventFlagsWait(*event_ptr, bitsToWaitFor, opt, ticksToWait) & 0x80000000) {
    osal_ret = (uint32_t)A_ERROR;
  }
  return osal_ret;
}

/*
 * Description: This function is used for set an event 
 *
 * Params: Event pointer, Bits to Set
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_set_event(qosal_event_handle event_ptr, A_UINT32 bitsToSet) {
  A_STATUS osal_ret = A_OK;

  if (osEventFlagsSet(*event_ptr, bitsToSet) & 0x80000000) {
    osal_ret = A_ERROR;
  }
  return osal_ret;
}

/*
 * Description: This function is used for creating an event
 *
 * Params: Event pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_create_event(qosal_event_handle event_ptr) {
  A_STATUS osal_ret = A_OK;
  *event_ptr = osEventFlagsNew(NULL);

  if (event_ptr == NULL) {
    osal_ret = A_ERROR;
  }
  return osal_ret;
}

/*
 * Description: This function is used for setting auto clearing of event bits in event group
 *
 * Params: Event pointer, flags
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_set_event_auto_clear(qosal_event_handle event_ptr, A_UINT32 flags) {
  if (flags) {
    // Auto clear is only required by driverWakeEvent
    DriverWakeEvent = event_ptr;
  }
  return A_OK;
}

/*
 * Description: This function is used for clearing the event 
 *
 * Params: Event pointer, BitsToClear
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
*/
A_STATUS qosal_clear_event(qosal_event_handle event_ptr, A_UINT32 bitsToClear) {
  A_STATUS osal_ret = A_OK;

  if (osEventFlagsClear(*event_ptr, bitsToClear) & 0x80000000) {
    osal_ret = A_ERROR;
  }
  return osal_ret;
}

/*
 * Description:  This function is used for deleting an event   
 *
 * Params: Event pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_delete_event(qosal_event_handle event_ptr) {
  A_STATUS osal_ret = A_OK;

  if (osEventFlagsDelete(*event_ptr) != osOK) {
    osal_ret = A_ERROR;
  } 
  return osal_ret;
}

/*****************************************************************************
 *
 * Task Synchronization APIs (Mutex)
 *
 *****************************************************************************/
/*
 * Description: This function is used for initialization of the mutex 
 *
 * Params: Mutex pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_mutex_init(qosal_mutex_handle mutex_ptr) {
  A_STATUS osal_ret = A_OK;

  *mutex_ptr = osSemaphoreNew(1, 1, NULL);
  if (*mutex_ptr == NULL) {
    osal_ret = A_ERROR;
   }
  return osal_ret;
}

/*
 * Description: This function is used for aquiring the mutex lock
 *
 * Params: Mutex pointer, tick_count
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_mutex_acquire(qosal_mutex_handle mutex_lock, A_ULONG tick_count) {
  A_STATUS osal_ret = A_OK;
  if (osSemaphoreAcquire(*mutex_lock, tick_count) != osOK) {
    osal_ret = A_ERROR;
  }
  return osal_ret; 
}

/*
 * Description: This function is used for releasing the mutex lock
 *
 * Params: Mutex pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail
 */
A_STATUS qosal_mutex_release(qosal_mutex_handle mutex_ptr) {
  A_STATUS osal_ret = A_OK;

  if (osSemaphoreRelease(*mutex_ptr) != osOK) {
    osal_ret = A_ERROR;
  }
  return osal_ret;
}

/*
 * Description: This function is used for deleting the mutex 
 *
 * Params: Mutex pointer
 *          
 * Returns: A_OK if Success, A_ERROR if Fail 
*/
A_STATUS qosal_mutex_destroy(qosal_mutex_handle mutex_ptr) {
  A_STATUS osal_ret = A_OK;

  if (osSemaphoreDelete(*mutex_ptr) != osOK) {
    osal_ret = A_ERROR;
  }
  return osal_ret;
}

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
A_VOID qosal_time_delay(A_UINT32 msec) {
  osDelay(msec);
}

/*
 * Description: This function is used to get the elapsed time from tick time
 *
 * Params: Ptr to Time struct
 *          
 * Returns: None
*/
A_VOID qosal_time_get_elapsed(TIME_STRUCT* time) {
  return;
}

/*********************************************************************
 *                 Kernel Log APIs
*********************************************************************/                    
/*
 * Description: This function is used to Creates the kernel logs
 *
* Params: size    : size of the log, 
           flags  :   1 (When the log is full, oldest entries are overwritten.)
-                     0 (When the log is full, no more entries are written; default.)
 *          
* Returns: A_OK for success, A_ERROR for failure 
*/
A_UINT32 qosal_klog_create(A_UINT32 size, A_UINT32 flags) {
  return A_OK;
}

/*
 * Description: Controls logging in kernel log
 *
* Params:   bit_mask    :    Which bits of the kernel log control variable to modify.
           set_bits     :   TRUE ((Bits set in bit_mask are set in the control variable)
-                           FALSE (Bits set in bit_mask are cleared in the control variable)
 *          
* Returns: None
*/

A_VOID qosal_klog_control(A_UINT32 bit_mask, A_BOOL set_bits) {
}

/*
 * Description: Displays the oldest entry in kernel log and delete this entry
 *
* Params: None
 *          
* Returns: A_OK for success, A_ERROR for failure 
*/

A_BOOL qosal_klog_dispaly(A_VOID) {
  return 0;
}
/*
 * Description: This function is used to Creates the kernel logs
 *
* Params: size    :   size of the log, 
          flags   :   1 (When the log is full, oldest entries are overwritten.)
-                     0 (When the log is full, no more entries are written; default.)
          ptr     :   Where in memory is the log to start
 *          
* Returns: A_OK for success, A_ERROR for failure 
*/

A_UINT32 qosal_klog_create_at(A_UINT32 max_size, A_UINT32 flags, A_VOID* ptr) {
  return 0;
}

#endif


/*******************************************************************************/
