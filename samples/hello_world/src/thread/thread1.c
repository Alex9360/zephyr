#include<zephyr/kernel.h>
#define STACK_SIZE 500
#define MY_PRIORITY 0

static void entry_point_1(void*arg1,void* arg2,void *arg3)
{
        while(1)
	{
		if(k_sem_take((struct k_sem*)arg1,K_MSEC(500))==0)
		{
			printf("\nthread 1 is running");
			k_msleep(500);
			k_sem_give((struct k_sem*)arg1);
		}
	}
}

static void entry_point_2(void *arg1,void* arg2,void* arg3)      
{ 
	while(1)
   	{	   
		if(k_sem_take((struct k_sem*)arg1,K_MSEC(10))==0)
		{
			printf("\nthread 2 is running");
			k_msleep(100);
			k_sem_give((struct k_sem*)arg1);
		}
	}
}

K_THREAD_STACK_DEFINE(stack_area1,STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_area2,STACK_SIZE);

int main()
{
	struct k_sem sem;
	struct k_thread thread1;
       	struct k_thread thread2;
       	
	k_sem_init(&sem,1,1);       
	k_thread_create(&thread1,stack_area1,STACK_SIZE,entry_point_1,(void*)&sem,NULL,NULL,MY_PRIORITY,0,K_NO_WAIT);
        k_thread_create(&thread2,stack_area2,STACK_SIZE,entry_point_2,(void*)&sem,NULL,NULL,MY_PRIORITY,0,K_NO_WAIT);
	
	return 0;
}


/*
#include<zephyr/kernel.h>
#define STACK_SIZE 500
#define MY_PRIORITY 0

static void entry_point_1(void*arg1,void* arg2,void *arg3)
{
        while(1)
	{
		k_sem_take((struct k_sem*)arg1,K_FOREVER);
		printf("\nthread 1 is running");
		k_msleep(5000);
		k_sem_give((struct k_sem*)arg1);
	}
}

static void entry_point_2(void *arg1,void* arg2,void* arg3)      
{ 
	while(1)
   	{	   
		k_sem_take((struct k_sem*)arg1,K_FOREVER);
		printf("\nthread 2 is running");
		k_msleep(1000);
		k_sem_give((struct k_sem*)arg1);
	}(.venv) alexpandi@linumiz:~/zephyrproject/zeph
}

K_THREAD_STACK_DEFINE(stack_area1,STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_area2,STACK_SIZE);

int main()
{
	struct k_sem sem;
	struct k_thread thread1;
       	struct k_thread thread2;
       	
	k_sem_init(&sem,1,1);       
	k_thread_create(&thread1,stack_area1,STACK_SIZE,entry_point_1,(void*)&sem,NULL,NULL,MY_PRIORITY,0,K_NO_WAIT);
        k_thread_create(&thread2,stack_area2,STACK_SIZE,entry_point_2,(void*)&sem,NULL,NULL,MY_PRIORITY,0,K_NO_WAIT);
	
	return 0;
}
*/
