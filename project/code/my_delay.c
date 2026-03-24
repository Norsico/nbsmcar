#include "my_delay.h"

// 延时s
void my_delay_s(uint8 s){
	uint8 i ;
	uint8 j ;
	for(i=0;i<s;i++){
		for(j=0;j<10;j++){
		system_delay_ms(100);
		}
	}
}