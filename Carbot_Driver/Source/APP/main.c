#include "stm32f10x.h"
#include "bsp.h"
#include "app.h"


int main(void)
{
	SystemInit();
	Bsp_Init();
	printf("\n");
	App_Init();

	delay_ms(2);
	printf("\nHello World!\n");     // 打印信息, 提示程序初始化完成。

	/* 启动FreeRTOS的任务列表，不再执行此函数后的内容 */
	App_Start_FreeRTOS();

	while (1)
		;
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
	/* Infinite loop */
	while (1)
		;
}
#endif
