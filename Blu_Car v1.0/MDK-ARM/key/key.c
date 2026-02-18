#include "main.h"                  // Device header
#include "key.h"  
#define down 0
#define up 1
#define key GPIOB
#define keypin GPIO_PIN_12
#define debouncetime 2
extern uint8_t singleflag;
extern uint8_t doubleflag;
extern uint8_t longflag;

uint8_t KeyState(void)
{	static uint8_t laststate=up;
	uint8_t currentstate=HAL_GPIO_ReadPin(key,keypin);
	if(currentstate==down&&laststate==up)
	{	laststate=currentstate;
		return down;
	}else if(currentstate==up&&laststate==down)
	{	laststate=currentstate;
		return up;
	}
}

void Keyscan(void)
{	uint32_t currentime=HAL_GetTick();
	static uint32_t presstime;
	static uint8_t state=free_state;
	static uint16_t count;
	switch(state)
	{
		case free_state:
			if(KeyState()==down)
			{	count++;
				if(count==debouncetime){
					count=0;
					if(KeyState()==down){
						presstime=currentime;
						state=singlepress;
					}
				}
			}				

			break;
		case singlepress:			
			if(currentime-presstime>1000)
			{					
				presstime=0;
				state=longpress;
			}else if(KeyState()==up)
			{		count++;
					if(count==debouncetime){
						count=0;						
		
							presstime=currentime;
							state=doublepress;
						
					}
			}

			break;
		
		case doublepress:
			if(currentime-presstime<200)
			{				
				if(KeyState()==down)
				{	count++;
					if(count==debouncetime){
						count=0;
						if(KeyState()==down){
							
							doubleflag=1;	
							presstime=0;
							state=releasewait;
						}
					}
				}				
			}else 
			{	presstime=0;
				singleflag=1;
				state=free_state;
			}
			
			break;
		
		case longpress:
			longflag=1;
			state=releasewait;
		
			break;
			
		case releasewait:
			if(KeyState()==up)
			{		count++;
					if(count==debouncetime){
						count=0;						
							state=free_state;	
					}
			}
			
			break;
			
		default: 
			state=free_state;
            break;
    }
	
}

