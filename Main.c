#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ch32f20x_iwdg.h"
#include "printf/printf.h"


/* Global define */
#define TASK1_TASK_PRIO     5
#define TASK1_STK_SIZE      256
#define TASK2_TASK_PRIO     5
#define TASK2_STK_SIZE      256

/* Global Variable */
TaskHandle_t Task1Task_Handler;
TaskHandle_t Task2Task_Handler;



/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.1 and GPIOB.8
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure={0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitTypeDef GPIO_InitStructure2={0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure2.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure2);
}


/*********************************************************************
 * @fn      task1_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */
void task1_task(void *pvParameters)
{
    UBaseType_t msticks=0;
    msticks = pdMS_TO_TICKS(500);
    while(1)
    {
        IWDG_ReloadCounter();
        printf_("task1 entry\r\n");
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        vTaskDelay(msticks);
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        vTaskDelay(msticks);
    }
}

/*********************************************************************
 * @fn      task2_task
 *
 * @brief   task2 program.
 *
 * @param  *pvParameters - Parameters point of task2
 *
 * @return  none
 */
void task2_task(void *pvParameters)
{		
    UBaseType_t msticks=0;
    msticks = pdMS_TO_TICKS(1000);
    while(1)
    {
        IWDG_ReloadCounter();
        printf_("task2 entry\r\n");
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        vTaskDelay(msticks);
        GPIO_SetBits(GPIOB, GPIO_Pin_8);
        vTaskDelay(msticks);
    }
}

void IWDG_Feed_Init( u16 prer, u16 rlr )
{
    IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
    IWDG_SetPrescaler( prer );
    IWDG_SetReload( rlr );
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    IWDG_Feed_Init( IWDG_Prescaler_256, 0xffff );
    IWDG_ReloadCounter();
    SystemCoreClockUpdate();
    USART_Printf_Init(115200);
    printf_("SystemClk:%d\r\n",SystemCoreClock);
    printf_( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf_("FreeRTOS Kernel Version:%s\r\n",tskKERNEL_VERSION_NUMBER);
    GPIO_Toggle_INIT();
    /* create two task */
    xTaskCreate((TaskFunction_t )task2_task,
                        (const char*    )"task2",
                        (uint16_t       )TASK2_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TASK2_TASK_PRIO,
                        (TaskHandle_t*  )&Task2Task_Handler);

    xTaskCreate((TaskFunction_t )task1_task,
                    (const char*    )"task1",
                    (uint16_t       )TASK1_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )TASK1_TASK_PRIO,
                    (TaskHandle_t*  )&Task1Task_Handler);
    vTaskStartScheduler();

    while(1)
    {
        printf_("shouldn't run at here!!\n");
    }
}



