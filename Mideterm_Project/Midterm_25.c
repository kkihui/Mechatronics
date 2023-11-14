/* Mechatronics Midterm Project by Team25*/

/* Include Header File*/
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>

/* Define constant*/
#define LOOPTIME 1		// Sampling Time
#define ENCODERA 17		// Hall Sensor A (Blue line)
#define ENCODERB 27		// Hall Sensor B (Purple line)
#define ENC2REDGEAR 216 // 1 rotation

#define MOTOR1 19
#define MOTOR2 26
#define PULSE 5 // Pulse pin

#define PGAIN 184 // First Using Ziegler method (0.6Ku)
#define IGAIN 0.0012 // 1.2Ku/Tu
#define DGAIN 0 // 0.075 Ku*Tu

/* Define Global variable*/
int encA,encB;
int encoderPosition = 0;
int pulseRecived = 0; // When pulse is recived, pulseRecived will change to 1
int i = 0;

float redGearPosition = 0;
float referencePosition = 10;
float errorPosition = 0;
float targetrotation[10]={0,}; // using array to contain the target rotation (1~10 time)

unsigned int checkTime;
unsigned int checkTimeBefore;

float G1 = PGAIN + IGAIN*LOOPTIME + DGAIN/LOOPTIME ; // PID control's term 
float G2 = -(PGAIN + 2*DGAIN/LOOPTIME) ; // PID control's term
float G3 = DGAIN/LOOPTIME ; // PID control's term

/* Function of updating errorPosition */
void funcEncoderA()
{
    encA = digitalRead(ENCODERA);
    encB = digitalRead(ENCODERB);
    if (encA == HIGH)
    {
        if (encB == LOW) encoderPosition++;
        else encoderPosition--;
    }
    else
    {
        if (encB == LOW) encoderPosition--;
        else encoderPosition++;
    }
    redGearPosition = (float)encoderPosition / ENC2REDGEAR;
    errorPosition = referencePosition - redGearPosition;
    printf("[%d]refPos: %f gearPos: %f  err: %f\n",
        i+1,referencePosition, redGearPosition, errorPosition);
}

/* Function of updating errorPosition2 */
void funcEncoderB()
{
    encA = digitalRead(ENCODERA);
    encB = digitalRead(ENCODERB);
    if (encB == HIGH)
    {
        if (encA == LOW) encoderPosition--;
        else encoderPosition++;
    }
    else
    {
        if (encA == LOW) encoderPosition++;
        else encoderPosition--;
    }
    redGearPosition = (float)encoderPosition / ENC2REDGEAR;
    errorPosition = referencePosition - redGearPosition;
    printf("[%d]refPos: %f gearPos: %f  err: %f\n",
        i+1,referencePosition, redGearPosition, errorPosition);
}

/* Function of checking pulse */
void pulse_check()
{
    pulseRecived = 1;
}

/* Main Function */
int main(void)
{
    /* Initialize and set up Gpio & set pin as input */
    wiringPiSetupGpio();
    pinMode(ENCODERA, INPUT);
    pinMode(ENCODERB, INPUT);
    pinMode(PULSE, INPUT);
    
    /* Create soft Pwm (0~100) */
    softPwmCreate(MOTOR1, 0, 100);
    softPwmCreate(MOTOR2, 0, 100);
    
    /* Define Interrupt Service Routine */
    wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderB);
    wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderA);
    wiringPiISR(PULSE, INT_EDGE_RISING, pulse_check);

    /* Input iteration & tagetrotation */
    int iteration;
    printf("Enter the number of iteration:");
    scanf("%d",&iteration);
    for (i=0;i<iteration;i++)
    {
        printf("[%d] Enter the number of rotation:",i+1);
        scanf("%f",&targetrotation[i]);
    }

    /*Check wheter pulse is received*/
    printf("Waiting 1st Pulse\n");
    while (1) if (pulseRecived) break; // Waiting untill 1st pulse is recived
    printf("Pulse received\n");

    /* Use PID control for the target position as much as the input iteration */
    for (i=0;i<iteration;i++)
    {
        /* Initialize Global variance & Define Local variance to use PID control & Print status  */
        pulseRecived = 0; // set pulseRecive as 0. so when next pulse is recieved, can execute the next itreation 
        printf("[%d] time \n",i);

        referencePosition = targetrotation[i]; // Enter the ith target rotation number using an array.
        errorPosition = referencePosition - redGearPosition;
        printf("reference is %f \n",referencePosition);
        
        float e,e1=0,e2=0,m,m1=0; // set PID control's error and magnitude term to 0
        checkTimeBefore = millis();
        
        /* Do PID control untill pulse is recived*/
        while (1)
        {
            checkTime = millis();
            if (checkTime - checkTimeBefore > LOOPTIME) // Update based on LOOPTIME
            {
                e = errorPosition;
                m = m1 +G1*e + G2*e1+ G3*e2; // set magnitude by PID control
                e2 = e1; // set e2 to error of 2step before 
                e1 = e; // set e1 to error of previous step
                m1 = m; // set m1 to magnitude of previous step

                if (e > 0)
                {
                    softPwmWrite(MOTOR2, m);
                    softPwmWrite(MOTOR1, 0);
                }
                else
                {
                    softPwmWrite(MOTOR1, -m);
                    softPwmWrite(MOTOR2, 0);
                }
                
                checkTimeBefore = checkTime;
            }
            if (pulseRecived) break;
        }        
    }    
    
    return 0;
}