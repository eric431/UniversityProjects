#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>

const int pin_100 = 17;
const int pin_75 = 27;
const int pin_50 = 22;
const int pin_25 = 23;

void readPins();

int main(void){

	wiringPiSetupGpio();

	wiringPiISR(pin_100, INT_EDGE_BOTH, &readPins);
	wiringPiISR(pin_75,  INT_EDGE_BOTH, &readPins);
	wiringPiISR(pin_50,  INT_EDGE_BOTH, &readPins);
	wiringPiISR(pin_25,  INT_EDGE_BOTH, &readPins);

	printf("Battery monitor started at pid %d, press CTRL+C to quit.\n", getpid());
	
	readPins();

	for(;;){

		sleep(UINT_MAX);
	}

	return 0;
}

void readPins(){
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
}
