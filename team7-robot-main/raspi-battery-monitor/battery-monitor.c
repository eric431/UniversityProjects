#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>

const int pin_100 = 17;
const int pin_75 = 27;
const int pin_50 = 22;
const int pin_25 = 23;

int main(void){
	wiringPiSetupGpio();
	pinMode(pin_100, INPUT);
	pinMode(pin_75, INPUT);
	pinMode(pin_50, INPUT);
	pinMode(pin_25, INPUT);
	
	printf("Battery monitor started at pid %d, press CTRL+C to quit.\n", getpid());

	while(1){
		int pin100 = digitalRead(pin_100);	
		int pin75 = digitalRead(pin_75);	
		int pin50 = digitalRead(pin_50);	
		int pin25 = digitalRead(pin_25);	


		if(pin25 && pin50 && pin75 && pin100){
			printf("Battery at 100%%.\n");
		}
		else if (pin25 && pin50 && pin75 && !pin100){
			printf("Battery at 75%%.\n");
		}
		else if (pin25 && pin50 && !pin75 && !pin100){
			printf("Battery at 50%%.\n");
		}
		else if (pin25 && !pin50 && !pin75 && !pin100){
			printf("Battery at 25%%.\n");
		}
		else {
			printf("Battery read error.\n");
		}
		delay(1000);
	}
	return 0;
}

