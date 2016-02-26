#include <pigpio.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>
#define RIGHT_MOTOR_PQM 255
#define LEFT_MOTOR_PQM 240
#define DRIFT_PQM 95
#define REVERSE 105


#define DYNAMIC_DIS 40
#define DISTANCE 18
#define TURN_DISTANCE 10

#define TURN_TIME       250000
#define TURN_DRIFT_TIME 250000

#define SERVO_RIGHT 700
#define SERVO_CENTER 1200
#define SERVO_LEFT 1900

const int n=20;// horizontal size of the map
const int m=20; // vertical size size of the map
unsigned int robo_dir=2, new_dir=0;
int secs, mics;
    /* Value for Direction
           2
           |
           |
       3-------1
           |
           |  
           0  
       */
static int map[n][m] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1}, 
    {1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,0,1,1,0,0,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1}, 
    {1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1}, 
    {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1}, 
    {1,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,0,0,1,1}, 
    {1,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,1,1,1}, 
    {1,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,1,1,1}, 
    {1,1,1,1,1,1,0,0,0,0,1,0,0,0,0,1,1,1,1,1}, 
    {1,1,1,1,1,1,0,0,0,0,1,0,0,'B',0,1,1,1,1,1}, 
    {1,1,1,1,1,1,0,'C',0,0,1,0,0,0,0,1,1,1,1,1}, 
    {1,1,1,1,1,1,0,0,0,0,1,0,0,0,'A',1,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
};
#if 0
static int map[n][m] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,'A',0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1}, 
    {1,1,0,'C',0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,0,0,'B',0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1}, 
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, 
};
#endif
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


int starty=0,startx=0, endx=1,endy=1;
int xA, yA, xB, yB, xC,yC,xD,yD;
static int intial_dir = 1;
static uint32_t distance_dynamic=0,avg=0,start=0, distance_current;
pthread_t *p1,*p2;
uint8_t i;
uint8_t right_d=0;
uint8_t center_d=0;
uint8_t left_d=0;
static uint8_t obstacle=0;
void move_servo_and_get_distance();
static uint8_t eye_set=0;

uint32_t left_check=0;   
uint32_t right_check =0;
/* forward prototypes */
void forward(uint8_t);
void forward_slow(void);
void rerouting( uint8_t left);
int forward_flag=0;
void reverse(void);
void leftturn(void);
void rightturn(void);

void leftdrift(void);
void rightdrift(void);
void stop(void);
void accel(void);
void brake(void);

void clear(int a);
void sonarTrigger(void);
void sonarEcho(int gpio, int level, uint32_t tick);
using namespace std;

//static int map[n][m];
//8 -source , 9-Destination
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static int dir_map[n][m]; // map of directions
const int dir=4; // number of possible directions to go at any position
// if dir==4
static int dx[dir]={1, 0, -1, 0};
static int dy[dir]={0, 1, 0, -1};
// if dir==8
//static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
//static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
    node(int xp, int yp, int d, int p) 
    {xPos=xp; yPos=yp; level=d; priority=p;}

    int getxPos() const {return xPos;}
    int getyPos() const {return yPos;}
    int getLevel() const {return level;}
    int getPriority() const {return priority;}

    void updatePriority(const int & xDest, const int & yDest)
    {
        priority=level+estimate(xDest, yDest)*10; //A*
    }

    // give better priority to going strait instead of diagonally
    void nextLevel(const int & i) // i: direction
    {
        level+=(dir==8?(i%2==0?10:14):10);
    }

    // Estimation function for the remaining distance to the goal.
    const int & estimate(const int & xDest, const int & yDest) const
    {
        static int xd, yd, d;
        xd=xDest-xPos;
        yd=yDest-yPos;         

        // Euclidian Distance
        d=static_cast<int>(sqrt(xd*xd+yd*yd));

        // Manhattan distance
        //d=abs(xd)+abs(yd);

        // Chebyshev distance
        //d=max(abs(xd), abs(yd));

        return(d);
    }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
    return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart, 
        const int & xFinish, const int & yFinish )
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
                pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();           
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1 
                        || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(), 
                        n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx && 
                                pq[pqi].top().getyPos()==ydy))
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}


void sonarTrigger(void)
{
    /* trigger a sonar reading */

    gpioWrite(SONAR_TRIGGER, PI_ON);
    gpioDelay(10); /* 10us trigger pulse */
    gpioWrite(SONAR_TRIGGER, PI_OFF);
}

void sonarEcho(int gpio, int level, uint32_t tick)
{
    static uint32_t startTick, firstTick=0, distance_s;

    if (!firstTick) firstTick = tick;
    if (level == PI_ON)
    {
        startTick = tick;
        //gpioTime(PI_TIME_RELATIVE, &secs, &mics);
        //printf("S0-library started %d.%03d seconds ago\n", secs, mics/1000);
    }
    else if (level == PI_OFF)
    {
        distance_s = (tick - startTick)/100;
        //	printf("%u ms, %d cm \n ", (tick-firstTick)/1000000, distance_s);
        //  printf(" %d cm \n ", distance_s);
    }
#if 1
    if(eye_set==0)
    {
	if((tick-firstTick)/1000000 > 5)
	    eye_set=1;
	printf("%u sec, %d cm \n ", (tick-firstTick)/1000000, distance_dynamic);
    }
#endif

    if(start<10)
    {
        avg+=distance_s;
        start++;
    }
    else
    {
        start=0;
        distance_dynamic = avg/10;
        avg=distance_s;
        //gpioTime(PI_TIME_RELATIVE, &secs, &mics);
        //printf("10-library started %d.%03d seconds ago\n", secs, mics/1000);
        //	printf("%u sec, %d cm \n ", (tick-firstTick)/1000000, distance_dynamic);
        if(distance_dynamic <= 15)
        {
            obstacle=1;
            printf("Obstacle found!!!!! STOP LOOKOUT\n\n");
	    gpioWrite(MOTOR1E, PI_LOW);
	    gpioWrite(MOTOR2E, PI_LOW);
	    forward_flag=0;
	    gpioSetAlertFunc(SONAR_ECHO, NULL);
	    move_servo_and_get_distance();
	    gpioSetAlertFunc(SONAR_ECHO, sonarEcho);
        }
        else
        {
            obstacle=0;
        }

    }

}

void move_servo_and_get_distance()
{
    gpioWrite  (PWM_TRIGGER, PI_OFF);
    gpioSetMode(PWM_TRIGGER, PI_OUTPUT);

    gpioServo(18, SERVO_RIGHT);
    time_sleep(1.5);
    left_d = distance_dynamic;

    gpioServo(18, SERVO_CENTER);
    time_sleep(1.5);
    center_d = distance_dynamic;

    gpioServo(18,SERVO_LEFT);
    time_sleep(1.5);
    right_d = distance_dynamic;

    gpioServo(18,SERVO_CENTER);
    time_sleep(1);
    gpioSetMode(PWM_TRIGGER, PI_INPUT);
//    printf("Left %d cm, Ceter %d cm Right %d cm\n", left_d, center_d, right_d);
}

void rightdrift(void)
{

    gpioWrite(MOTOR1A, PI_LOW);
    gpioWrite(MOTOR1B, PI_HIGH);
    gpioPWM(MOTOR2E, LEFT_MOTOR_PQM - DRIFT_PQM);
    gpioPWM(MOTOR1E, RIGHT_MOTOR_PQM - DRIFT_PQM);
    forward_flag=0;
    gpioDelay(TURN_DRIFT_TIME); 
    gpioWrite(MOTOR1E, PI_LOW);
    gpioWrite(MOTOR2E, PI_LOW);
}
void leftdrift(void)
{
    gpioWrite(MOTOR2A, PI_LOW);
    gpioWrite(MOTOR2B, PI_HIGH);
    gpioPWM(MOTOR2E, LEFT_MOTOR_PQM - DRIFT_PQM);
    gpioPWM(MOTOR1E, RIGHT_MOTOR_PQM- DRIFT_PQM);
    forward_flag=0;
    gpioDelay(TURN_DRIFT_TIME); 
    gpioWrite(MOTOR1E, PI_LOW);
    gpioWrite(MOTOR2E, PI_LOW);
}
void forward(uint8_t pwm)
{
follow_normal_path:
    /* Check distance for right iturn is enough */
printf(" visibility L:%d C:%d R:%d\n", left_d, distance_dynamic, right_d);
    if(distance_dynamic <= DYNAMIC_DIS)
    {
        gpioWrite(MOTOR1E, PI_LOW);
        gpioWrite(MOTOR2E, PI_LOW);
	move_servo_and_get_distance();
        if( (right_d <DISTANCE) && (right_d < left_d) )
        {
	    /* Left path is open try rerouting via left */
	    rerouting(1);
	}
        else
        {
	    /* right path is open try rerouting via right */
	    rerouting(0);
	}

        stop();
	move_servo_and_get_distance();
        goto follow_normal_path;
    } 
    else
    {
        if(forward_flag==0)
        {
            gpioWrite(MOTOR1A, PI_HIGH);
            gpioWrite(MOTOR1B, PI_LOW);

            gpioWrite(MOTOR2A, PI_HIGH);
            gpioWrite(MOTOR2B, PI_LOW);

            gpioPWM(MOTOR2E, LEFT_MOTOR_PQM);
            gpioPWM(MOTOR1E, RIGHT_MOTOR_PQM);
            forward_flag=1;
        }
        time_sleep(0.8);
        obstacle = 0;
    }
}
void reverse(void)
{
    printf("Reversing\n");
    gpioPWM(MOTOR2E, LEFT_MOTOR_PQM-REVERSE);
    gpioPWM(MOTOR1E, RIGHT_MOTOR_PQM-REVERSE);

    gpioWrite(MOTOR1B, PI_HIGH);
    gpioWrite(MOTOR1A, PI_LOW);

    gpioWrite(MOTOR2B, PI_HIGH);
    gpioWrite(MOTOR2A, PI_LOW);
    obstacle = 0;
    gpioDelay(TURN_DRIFT_TIME); 

}
void rerouting( uint8_t left)
{
    if(left ==1)
    {
	printf("Rerouting via left\n");
	leftturn();
	forward_slow();
	rightturn();
	forward_slow();
	rightturn();
	forward_slow();
	leftturn();
    }
    else
    {
	printf("Rerouting via right\n");
	rightturn();
	forward_slow();
	leftturn();
	forward_slow();
	leftturn();
	forward_slow();
	rightturn();
    }
    printf("Rerouting Completed\n");
}
void forward_slow(void)
{
    gpioWrite(MOTOR1A, PI_HIGH);
    gpioWrite(MOTOR1B, PI_LOW);

    gpioWrite(MOTOR2A, PI_HIGH);
    gpioWrite(MOTOR2B, PI_LOW);

    gpioPWM(MOTOR2E, LEFT_MOTOR_PQM-REVERSE);
    gpioPWM(MOTOR1E, RIGHT_MOTOR_PQM-REVERSE);

    time_sleep(1.5); 
    gpioWrite(MOTOR1E, PI_LOW);
    gpioWrite(MOTOR2E, PI_LOW);

}

void rightturn(void)
{
#if 1
follow_normal_path:
    /* Check distance for right iturn is enough */
printf(" Visibility  L:%d C:%d R:%d\n", left_d,center_d, right_d);
    if( (right_d <=TURN_DISTANCE) || (center_d <=TURN_DISTANCE) || (left_d <=TURN_DISTANCE) )
    {
        reverse();
        stop();
	move_servo_and_get_distance();
        goto follow_normal_path;
    } 
    else
#endif
    {
        /* enough*/
        gpioWrite(MOTOR1A, PI_LOW);
        gpioWrite(MOTOR1B, PI_HIGH);
    
	gpioPWM(MOTOR2E, LEFT_MOTOR_PQM);
	gpioPWM(MOTOR1E, RIGHT_MOTOR_PQM);
        forward_flag=0;
        gpioDelay(TURN_TIME); 
        stop();
        obstacle = 0;
    }
}
void leftturn(void)
{
#if 1
follow_normal_path:
    /* Check distance for right iturn is enough */
printf(" visibility L:%d C:%d R:%d\n", left_d,center_d, right_d);
    if( (right_d <=TURN_DISTANCE) || (center_d <=TURN_DISTANCE) || (left_d <=TURN_DISTANCE) )
    {
printf("VALUESSS ISNIDE %d %d %d\n", left_d,center_d, right_d);
        reverse();
        stop();
	move_servo_and_get_distance();
        goto follow_normal_path;
    } 
    else
#endif
    {
        /* enough*/
        gpioWrite(MOTOR2A, PI_LOW);
        gpioWrite(MOTOR2B, PI_HIGH);
	gpioPWM(MOTOR2E, LEFT_MOTOR_PQM);
	gpioPWM(MOTOR1E, RIGHT_MOTOR_PQM);
        forward_flag=0;
        gpioDelay(TURN_TIME); 
        stop();
        obstacle = 0;
    }
}
void stop(void)
{
//    cout<<"STOP"<<endl;
    gpioWrite(MOTOR1E, PI_LOW);
    gpioWrite(MOTOR2E, PI_LOW);
    forward_flag=0;
    obstacle=1;
    time_sleep(1);
}

string route;

void get_route_move_robo(int xa,int ya, int xb, int yb)
{
        system("clear");
    cout<<endl<<endl;  
    cout<<"Going from "<<xa<<","<<ya<<"->"<<xb<<","<<yb<<endl;
    route=pathFind(xa, ya, xb, yb);
    if(route=="") cout<<"An empty route generated!"<<endl;
    // follow the route on the map and display it 
    if(route.length()>0)
    {
	int j; char c;
	int x=xa;
	int y=ya;
	map[x][y]=2;
	for(int i=0;i<route.length();i++)
	{
	    c =route.at(i);
	    j=atoi(&c); 
	    x=x+dx[j];
	    y=y+dy[j];
	    map[x][y]=3;
	}
	map[x][y]=4;

	// display the map with the route
	for(int x=0;x<n;x++)
	{
	    cout<<"               ";
	    for(int y=0;y<m;y++)
	    {    if(map[x][y]==0)
		cout<<". ";
		else if(map[x][y]==1)
		    cout<<"# "; //obstacle
		else if(map[x][y]==2)
		    cout<<"S "; //start
		else if(map[x][y]==3)
		    cout<<"+ "; //route
		else if(map[x][y]==4)
		    cout<<"G "; //finish
		else
		    cout<<". ";
	    }
	    cout<<endl;
	}
    }

    cout<<"Moving robot in path"<<endl;
    getchar();
    while(1) {
	for(int i=0;i<route.length();i++)
	{
	    while(obstacle==1)
	    {
		time_sleep(0.5); /* Wait to clear the obstacle */
	    }
	    char c[0];
	    c[0] =route.at(i);
	    new_dir = atoi(c) ;
	    //        cout<<"Robo Dir "<<robo_dir<<" New Dir "<<new_dir<<endl;
	    cout<<"--->";
	    if((new_dir - robo_dir) == 0)
	    { 
		cout<<"Forward";
		forward(25);
		/*            if(new_dir==0)
			      cout<<"Moving South";
			      else if(new_dir ==1)
			      cout<<"Moving East";
			      else if(new_dir ==2)
			      cout<<"Moving North";
			      else if(new_dir ==3)
			      cout<<"Moving West";
		 */
	    }
	    else if(( (new_dir - robo_dir) == -1 ) || ((new_dir - robo_dir) == -3))
	    {
		cout<<"Moving Right";
		stop();
		move_servo_and_get_distance();
		rightturn();
		forward(25);
	    }
	    else if(( (new_dir - robo_dir) == 1 ) || ((new_dir - robo_dir) == 3))
	    {
		cout<<"Moving Left";
		stop();
		move_servo_and_get_distance();
		leftturn();
		forward(25);
	    }
	    else if( (new_dir +robo_dir) % 2 == 0)
	    {
		cout<<"About Turn required  "<<robo_dir<<" to "<<new_dir<<" ";
		cout<<"Moving Right";
		rightturn();
		cout<<"Moving Right";
		rightturn();
	    }
	    else
	    {
		cout<<"************Wrong Direction *******"<<endl;
		cout<<"Robo Dir "<<robo_dir<<" New Dir "<<new_dir<<endl;
	    }
	    robo_dir = new_dir;
	    cout<<std::flush;
	    //  getchar(); // wait for a (Enter) keypress  
	    cout<<endl;

	}
	break;
    }

    cout<<"===> Goal Reached"<<endl;
    stop();
    cout<<"Want new route, Press enter"<<endl;
    getchar();
    system("clear");  
}

void get_goals() 
{
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++)
        {
            //	       cout<<map[x][y];
            if(map[x][y]=='B')
            {
                xB=x;yB=y;
            }
            if(map[x][y]=='A')
            {
                xA=x;yA=y;
            }
            if(map[x][y]=='C')
            {
                xC=x;yC=y;
            }
            if(map[x][y]=='D')
            {
                xD=x;yB=y;
            }
            //	      cout<<" ";
        }
        //	cout<<endl;
    }
}
int main()
{
    cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
    get_goals(); 
    cout<<endl;
    cout<<"*************Gpio Init*************"<<endl; 
    if (gpioInitialise()<0) return 1;
    gpioSetSignalFunc(2, clear);

    gpioSetMode(SONAR_TRIGGER, PI_OUTPUT);
    gpioWrite  (SONAR_TRIGGER, PI_OFF);
    gpioSetMode(SONAR_ECHO,    PI_INPUT);

    gpioSetMode(MOTOR1E,  PI_OUTPUT);
    gpioSetMode(MOTOR1A,  PI_OUTPUT);
    gpioSetMode(MOTOR1B,  PI_OUTPUT);

    gpioSetMode(MOTOR2E,  PI_OUTPUT);
    gpioSetMode(MOTOR2A,  PI_OUTPUT);
    gpioSetMode(MOTOR2B,  PI_OUTPUT);
    
    gpioServo(18, SERVO_CENTER);
    time_sleep(1);
    gpioSetMode(PWM_TRIGGER, PI_INPUT);

    /* update sonar 20 times a second, timer #0 */
    gpioSetTimerFunc(0, 50, sonarTrigger); /* every 50ms */
    /* monitor sonar echos */
    gpioSetAlertFunc(SONAR_ECHO, sonarEcho);
printf("Waiting for Sonar to settle \n");
getchar();	    
    get_route_move_robo(xA,yA,xB,yB);
    get_route_move_robo(xB,yB,xA,yA);
    get_route_move_robo(xC,yC,xA,yA);

    getchar();
    while (1)
    {
        time_sleep(5);
    }
    clear(2);
    gpioTerminate();
    return 0;

    return(0);
}

void clear(int a)
{
    printf("\n\nInside GPIO reset \n");
    gpioStopThread(p1); time_sleep(1);
    //    gpioStopThread(p2); time_sleep(1);
    gpioSetAlertFunc(SONAR_ECHO, NULL);
    gpioSetTimerFunc(0, 20, NULL); /* every 50ms */
    time_sleep(2);
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
    time_sleep(1);
    printf("Mode %d\n",gpioGetMode(SONAR_TRIGGER));
    printf("MOde %d\n",gpioGetMode(SONAR_ECHO));
    printf("MOde %d\n",gpioGetMode(PWM_TRIGGER));
    gpioTerminate();
    printf("Exiting GPIO Reset\n");
    exit(1);
}
