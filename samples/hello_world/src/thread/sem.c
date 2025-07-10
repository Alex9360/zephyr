#include <stdio.h>
#include <zephyr/kernel.h>
#include <stdlib.h>

#define MY_STACK_SIZE 500
#define MY_PRIORITY_LOW 5
#define MY_PRIORITY_HIGH 0

struct k_sem my_sem;
K_SEM_DEFINE(my_sem, 1, 1);
void add(uint8_t a,uint8_t b)
{
uint8_t c=a+b;
printf("%d",c);
}
static void my_entry_point_1(void * thread, void * counter, void *)
{
	while(1)
	{
		add(1,thread);
	}
		 if (k_sem_take(&my_sem, K_MSEC(50)) != 0)
		 {
			//while(1)
			for(uint8_t i =0 ; i<4 ; i++)
			{
				if(!(*(uint8_t *)counter & 1))
				{
					k_sleep(K_MSEC(1));
					printf("Thread - %d - EVEN : %d\n", (uint32_t)thread, (*(uint8_t*)counter)++);
					k_sleep(K_MSEC(1000));
				}
				else
				{
					k_sleep(K_MSEC(2));
					printf("Thread - %p - ODD  : %d\n", (uint8_t *)thread, (*(uint8_t*)counter)++);
					k_sleep(K_MSEC(5000));
				}
				k_sem_give(&my_sem);
			}

		 }
		 else
		 {
			printf("Input data not available!\n");
			k_msleep(1000);
		 }
	}
}

K_THREAD_STACK_DEFINE(my_stack_area_1, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(my_stack_area_2, MY_STACK_SIZE);

struct k_thread my_thread_data_1;
struct k_thread my_thread_data_2;

int main(void)
{
        void * counter = calloc(1,1);
        k_thread_create(&my_thread_data_1, my_stack_area_1, MY_STACK_SIZE, my_entry_point_1, (void *)1, counter, NULL, MY_PRIORITY_LOW, 0, K_NO_WAIT);
        k_thread_create(&my_thread_data_2, my_stack_area_2, MY_STACK_SIZE, my_entry_point_1, (void *)2, counter, NULL, MY_PRIORITY_LOW, 0, K_NO_WAIT);
        return 0;
}

