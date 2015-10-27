#include "WiringPi-master/wiringPi/wiringPi.h"
#include "WiringPi-master/wiringPi/softServo.h"

void moveServo(int rot);


int main(void)
{
wiringPiSetup();
softServoSetup(0,1,2,3,4,5,6,7);

for(int i = 0; i<5 ; i++){
	moveServo(-1);
}

moveServo(1);
moveServo(-1);
moveServo(1);

delay(5000);

return 0;
}

void moveServo(int rot){
switch(rot)
{
case -1:
	softServoWrite(0,300);
	delay(50);
	softServoWrite(0,425);
	break;
case 0:
	softServoWrite(0,425);
	delay(50);
	break;
case 1:
	softServoWrite(0,700);
	delay(50);
	softServoWrite(0,425);
	break;
default:
	softServoWrite(0,425);
	delay(50);
	break;
}
}


