#include <stdio.h>
#include <pigpio.h>

/* BCM numbering */
#define SONAR_TRIGGER 23
#define SONAR_ECHO    24

#define PWM_TRIGGER   18

#define MOTOR1E 12
#define MOTOR1A 20
#define MOTOR1B 21

#define MOTOR2E 19
#define MOTOR2A 6
#define MOTOR2B 5

static int intial_dir = 1;
static uint32_t distance=0,avg=0,start=0;
pthread_t *p1,*p2;
uint8_t i;
uint8_t right_d=0;
uint8_t center_d=0;
uint8_t left_d=0;
uint8_t obstacle=0;

uint32_t left_check=0;   
uint32_t right_check =0;
static uint32_t eye_set=0;
/* forward prototypes */
void forward(uint8_t);
void reverse(void);
void leftturn(void);
void rightturn(void);
void leftturn_abt(void);
void rightturn_abt(void);
void stop(void);
void accel(void);
void brake(void);

void clear(int a);
void sonarTrigger(void);
void servo() ;
void *dc_motor(void *arg);
void sonarEcho(int gpio, int level, uint32_t tick);
int startx,starty=0;
int sx=5,sy=5;
int gx=0,gy=0;
int map_min=0,map_max=6;
void print();
int find_path(int a, int b, int d);
char map[6][6]={ 
    {'g','.','#','#','#','#'},
    {'#','.','#','#','#','#'},
    {'#','.','#','#','#','#'},
    {'#','.','.','.','.','.'},
    {'#','#','#','#','#','.'},
    {'#','#','#','#','#','s'}};
void print()
{    
    for(sx=0;sx<6;sx++)
    {
        for(sy=0;sy<6;sy++)
        {
            printf(" %c", map[sx][sy]);
        }
        printf("\n");
    }
    printf("\n");
    //sleep(1);
}
int find_path(int a, int b, int d)
{
    if((a>map_max) || (b>map_max) || (a<0) || (b<0))
        return 1;

    if((a==gx) && (b==gy))
        return 0;

    if(map[a][b]=='#')
        return 1;
    
    if(map[a][b]=='+')
        return 1;

    map[a][b]='+';
    printf("Going %d\n", d);
    print(); 
   
//getchar(); 
    if((d-intial_dir) < 0) 
    {
	rightturn();
    } 
    else if ((d-intial_dir) > 0)
    {
	leftturn();
    }
    else
    {
    printf("Continue\n");
    forward(25);

    sleep(1);
    }
    intial_dir = d;

    if(find_path(a-1,b,1) == 0)
    {
        return 0;
    }
    if(find_path(a,b+1,2) == 0)
    {
        return 0;
    }
    if(find_path(a+1,b,3) == 0)
    {
        return 0;
    }
    if(find_path(a,b-1,4) == 0)
    {
        return 0;
    }

    map[a][b]='x';
    return 1;

}

int main(int argc, char *argv[])
{
    startx=sx;starty=sy;

    if (gpioInitialise()<0) return 1;
    gpioSetSignalFunc(2, clear);

    //    p1 = gpioStartThread(servo, "thread 1"); sleep(3);

    gpioSetMode(SONAR_TRIGGER, PI_OUTPUT);
    gpioWrite  (SONAR_TRIGGER, PI_OFF);
    gpioSetMode(SONAR_ECHO,    PI_INPUT);

    gpioSetMode(MOTOR1E,  PI_OUTPUT);
    gpioSetMode(MOTOR1A,  PI_OUTPUT);
    gpioSetMode(MOTOR1B,  PI_OUTPUT);

    gpioSetMode(MOTOR2E,  PI_OUTPUT);
    gpioSetMode(MOTOR2A,  PI_OUTPUT);
    gpioSetMode(MOTOR2B,  PI_OUTPUT);

    /* update sonar 20 times a second, timer #0 */
    gpioSetTimerFunc(0, 10, sonarTrigger); /* every 50ms */

    /* monitor sonar echos */
    gpioSetAlertFunc(SONAR_ECHO, sonarEcho);
    p2 = gpioStartThread(dc_motor, "thread 2"); sleep(3);
    while (1)
    {
	sleep(5);
    }

    gpioTerminate();
    return 0;
}

void clear(int a)
{
    printf("\n\nInside GPIO reset \n");
    gpioStopThread(p1); sleep(1);
    gpioStopThread(p2); sleep(1);
    gpioSetAlertFunc(SONAR_ECHO, NULL);
    gpioSetTimerFunc(0, 20, NULL); /* every 50ms */
    sleep(2);
    gpioWrite  (SONAR_TRIGGER, PI_OFF);
    gpioWrite  (SONAR_ECHO, PI_OFF);
    gpioWrite  (PWM_TRIGGER, PI_OFF);
    gpioSetMode(SONAR_TRIGGER, PI_INPUT);
    gpioSetMode(SONAR_ECHO,    PI_INPUT);
    gpioSetMode(PWM_TRIGGER, PI_INPUT);
    gpioSetMode(MOTOR1E,  PI_INPUT);
    gpioSetMode(MOTOR1A,  PI_INPUT);
    gpioSetMode(MOTOR1B,  PI_INPUT);
    gpioSetMode(MOTOR2E,  PI_INPUT);
    gpioSetMode(MOTOR2A,  PI_INPUT);
    gpioSetMode(MOTOR2B,  PI_INPUT);
    gpioWrite(MOTOR1E, PI_LOW);
    gpioWrite(MOTOR2E, PI_LOW);
    sleep(1);
    printf("Mode %d\n",gpioGetMode(SONAR_TRIGGER));
    printf("MOde %d\n",gpioGetMode(SONAR_ECHO));
    printf("MOde %d\n",gpioGetMode(PWM_TRIGGER));
    gpioTerminate();
    printf("Exiting GPIO Reset\n");
exit(1);
}
void sonarTrigger(void)
{
    /* trigger a sonar reading */

    gpioWrite(SONAR_TRIGGER, PI_ON);
    gpioDelay(10); /* 10us trigger pulse */
    gpioWrite(SONAR_TRIGGER, PI_OFF);
}

void servo()
{
    uint16_t pw=0;   
#if 0
    gpioSetMode(PWM_TRIGGER, PI_OUTPUT);
    gpioSetPWMfrequency(PWM_TRIGGER,400);
    gpioSetPWMrange(PWM_TRIGGER,    2500);
    obstacle=1;
    {
	pw=1000;
	gpioServo(18, pw);
	printf("%d\n", pw);
	sleep(1); /* 1/10th second delay */
	left_d = distance;

	pw=1300;
	gpioServo(18, pw);
	printf("%d\n", pw);
	sleep(1); /* 1/10th second delay */
	center_d = distance;

	pw=1800;
	gpioServo(18,pw);
	printf("%d\n", pw);
	sleep(1);
	right_d = distance;

	pw=1300;
	gpioServo(18,pw);
	printf("%d\n", pw);
	sleep(1);
    }
#endif
}   

void forward(uint8_t pwm)
{
if(0)    
{
        printf("Forward on %d\n", pwm);
	gpioPWM(MOTOR2E, pwm +200);
	gpioPWM(MOTOR1E, pwm +200);
    }

    gpioWrite(MOTOR1A, PI_HIGH);
    gpioWrite(MOTOR1B, PI_LOW);

    gpioWrite(MOTOR2A, PI_HIGH);
    gpioWrite(MOTOR2B, PI_LOW);
        
    printf("Forward on %d\n", pwm);
    gpioWrite(MOTOR1E, PI_HIGH);
    gpioPWM(MOTOR2E, 252);
}
void reverse(void)
{
    printf("Reversing\n");
    gpioPWM(MOTOR2E, 240);
    gpioPWM(MOTOR1E, 240);

    gpioWrite(MOTOR1B, PI_HIGH);
    gpioWrite(MOTOR1A, PI_LOW);

    gpioWrite(MOTOR2B, PI_HIGH);
    gpioWrite(MOTOR2A, PI_LOW);

}
void leftturn_abt(void)
{
    printf("Turning LEFT\n");
    gpioPWM(MOTOR2E, 240);
    gpioPWM(MOTOR1E, 240);
    gpioWrite(MOTOR1E, PI_LOW);
    sleep(3);
    left_check=1;
    right_check=0;
    gpioWrite(MOTOR1E, PI_HIGH);
}
void rightturn_abt(void)
{
    printf("Turning RIGHT\n");
    gpioPWM(MOTOR2E, 210);
    gpioPWM(MOTOR1E, 210);
    gpioWrite(MOTOR2E, PI_LOW);
    sleep(3);
    left_check=1;
    right_check=1;
    gpioWrite(MOTOR2E, PI_HIGH);
}

void leftturn(void)
{
   gpioWrite(MOTOR1A, PI_LOW);
   gpioWrite(MOTOR1B, PI_HIGH);
   gpioWrite(MOTOR2E, PI_HIGH);
   gpioWrite(MOTOR1E, PI_HIGH);
   usleep(800000);
   
}
void rightturn(void)
{
   gpioWrite(MOTOR2A, PI_LOW);
   gpioWrite(MOTOR2B, PI_HIGH);
   gpioWrite(MOTOR2E, PI_HIGH);
   gpioWrite(MOTOR1E, PI_HIGH);
   usleep(800000);
}
void stop(void)
{
    gpioWrite(MOTOR1E, PI_LOW);
    gpioWrite(MOTOR2E, PI_LOW);
}

void sonarEcho(int gpio, int level, uint32_t tick)
{
    static uint32_t startTick, firstTick=0, distance_s;

    if (!firstTick) firstTick = tick;
    if (level == PI_ON)
    {
	startTick = tick;
    }
    else if (level == PI_OFF)
    {
	distance_s = (tick - startTick)/100;
	//	printf("%u ms, %d cm \n ", (tick-firstTick)/1000000, distance_s);
	//  printf(" %d cm \n ", distance_s);
    }
	eye_set=1;
    if((tick-firstTick)/1000000 > 5)
	eye_set=1;

    if(start<10)
    {
	avg+=distance_s;
	start++;
    }
    else
    {
	start=0;
	distance = avg/10;
	avg=distance_s;
//	printf(" %d cm \n ", distance);
	if(distance <=20)
	{
	    //obstacle=1;
	}

    }

}

void *dc_motor(void *arg)
{
    /* Add code logic here */
    uint8_t max=0;
    static goal=0;
//reverse();
//forward(25);
//sleep(10);
    while(1)
    {  

	if(goal==1);
	{
	    print(); 
	    if(find_path(startx,starty,1)==1)
		printf("Goal not reachable\n");    
	    else
		printf("Goal reachable\n");    

	    goal=1;
	}
	{
	    stop();
	    break;
	}
    }
}


