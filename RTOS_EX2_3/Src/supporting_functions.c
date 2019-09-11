/* Standard includes. */
#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

void vPrintString( const char *pcString )
{
	/* Print the string, using a critical section as a crude method of mutual exclusion. */
	taskENTER_CRITICAL();
	{
		printf( "%s", pcString );
		fflush( stdout );
	}
	taskEXIT_CRITICAL();
}

void vPrintStringAndNumber( const char *pcString, uint32_t ulValue )
{
	/* Print the string, using a critical section as a crude method of mutual exclusion. */
	taskENTER_CRITICAL();
	{
		printf( "%s %lu\r\n", pcString, ulValue );
		fflush( stdout );
	}
	taskEXIT_CRITICAL();
}

void vPrintTwoStrings( const char *pcString1, const char *pcString2 )
{
	/* Print the string, using a critical section as a crude method of mutual exclusion. */
	vTaskSuspendAll();
	{
		printf( "At time %lu: %s %s\r\n", xTaskGetTickCount(), pcString1, pcString2 );
	}
	xTaskResumeAll();
}