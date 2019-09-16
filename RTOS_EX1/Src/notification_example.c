void vANInterruptHandler( void )
{
  BaseType_t xHigherPriorityTaskWoken;

  /* Clear the interrupt. */
  prvClearInterruptSource();

  /* xHigherPriorityTaskWoken must be initialised to pdFALSE.
  If calling vTaskNotifyGiveFromISR() unblocks the handling
  task, and the priority of the handling task is higher than
  the priority of the currently running task, then
  xHigherPriorityTaskWoken will be automatically set to pdTRUE. */
  xHigherPriorityTaskWoken = pdFALSE;

  /* Unblock the handling task so the task can perform any processing
  necessitated by the interrupt.  xHandlingTask is the task's handle, which was
  obtained when the task was created.  vTaskNotifyGiveFromISR() also increments
  the receiving task's notification value. */
  vTaskNotifyGiveFromISR( xHandlingTask, &xHigherPriorityTaskWoken );

  /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
  The macro used to do this is dependent on the port and may be called
  portEND_SWITCHING_ISR. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/* A task that blocks waiting to be notified that the peripheral needs servicing. */
void vHandlingTask( void *pvParameters )
{
  BaseType_t xEvent;
  const TickType_t xBlockTime = pdMS_TO_TICKS( 500 );
  uint32_t ulNotifiedValue;

  for( ;; )
  {
    /* Block to wait for a notification.  Here the RTOS task notification
    is being used as a counting semaphore.  The task's notification value
    is incremented each time the ISR calls vTaskNotifyGiveFromISR(), and
    decremented each time the RTOS task calls ulTaskNotifyTake() - so in
    effect holds a count of the number of outstanding interrupts.  The first
    parameter is set to pdFALSE, so the notification value is only decremented
    and not cleared to zero, and one deferred interrupt event is processed
    at a time.  See example 2 below for a more pragmatic approach. */
    ulNotifiedValue = ulTaskNotifyTake( pdFALSE, xBlockTime );

    if( ulNotifiedValue > 0 )
    {
      /* Perform any processing necessitated by the interrupt. */
      xEvent = xQueryPeripheral();
      if( xEvent != NO_MORE_EVENTS )
      {
        vProcessPeripheralEvent( xEvent );
      }
    }
    else
    {
      /* Did not receive a notification within the expected time. */
      vCheckForErrorConditions();
    }
  }
}