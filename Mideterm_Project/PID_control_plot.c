/* Mechatronics Midterm Project by Team25*/

// P control is Kp en
// I control is K
// D control nm = Kd (en-en-1/T)

#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <string.h>


#define LOOPTIME 5		// Sampling Time
#define ENCODERA 17		// Hall Sensor A (Blue line)
#define ENCODERB 27		// Hall Sensor B (Purple line)
#define ENC2REDGEAR 216 // 1 rotation

#define MOTOR1 19
#define MOTOR2 26
#define PULSE 5 // Pulse pin

#define PGAIN 184 // First Using Ziegler method (0.6Ku)
#define IGAIN 0.0012 // 1.2Ku/Tu
#define DGAIN 0 // 0.075 Ku*Tu

#define ARRAY_SIZE 10000
#define NUM_COLUMNS 2
#define DAQ_TIME 10000 //10s

int encA,encB;
int encoderPosition = 0;
int pulseRecived = 0;
int i = 0;

float redGearPosition = 0;
float referencePosition = 10;
float errorPosition = 0;
float targetrotation[10]={0,};

unsigned int checkTime;
unsigned int checkTimeBefore;
unsigned int startTime;
unsigned int iterationTime;

float G1 = PGAIN + IGAIN*LOOPTIME + DGAIN/LOOPTIME ;
float G2 = -(PGAIN + 2*DGAIN/LOOPTIME) ;
float G3 = DGAIN/LOOPTIME ;

int dataIndex = 0;
float dataArray[ARRAY_SIZE][NUM_COLUMNS];

void updateDataArray(float input)
{
    dataArray[dataIndex][0] = (float)(checkTime - startTime) / 1000.0;
    dataArray[dataIndex][1] = input;
    dataArray[dataIndex][2] = redGearPosition;
    dataIndex++;
}

void plotGraph()
{
    FILE *gnuplot = popen("gnuplot -persistent", "w");
    
    fprintf(gnuplot, "set yrange [-100:100]\n");
    fprintf(gnuplot, "set xlabel 'Time [s]'\n"); 
    fprintf(gnuplot, "set ylabel 'Magnitude(result of PID) [PWM]'\n"); 
    fprintf(gnuplot, "plot '-' with lines title 'Magnitude'");
    for (int i = 0; i < dataIndex; ++i) {
        fprintf(gnuplot, "%f %f\n", dataArray[i][0], dataArray[i][1]);
    }
    fprintf(gnuplot, "e\n");
}


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

int main(void)
{
    char filename[100];
    FILE* file;

    printf("Enter the file name: ");
    scanf("%s", filename);
    
    file = fopen(strcat(filename,".csv"), "w+");

    wiringPiSetupGpio();
    pinMode(ENCODERA, INPUT);		// Set ENCODERA as input
    pinMode(ENCODERB, INPUT);		// Set ENCODERB as input
    pinMode(PULSE, INPUT);

    softPwmCreate(MOTOR1, 0, 100);		// Create soft Pwm
    softPwmCreate(MOTOR2, 0, 100); 	// Create soft Pwm

    wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderB);
    wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderA);
    
    // input iteration & tagetrotation
    int iteration;
    printf("Enter the number of iteration:");
    scanf("%d",&iteration);
    for (i=0;i<iteration;i++)
    {
        printf("[%d] Enter the number of rotation:",i+1);
        scanf("%f",&targetrotation[i]);
    }

    startTime = millis();
    for (i=0;i<iteration;i++) // Repeat as many times as the number entered initially
    {
        iterationTime = millis();
        printf("[%d] time \n",i);
        referencePosition = targetrotation[i]; // Enter the ith target rotation number using an array.
        errorPosition = referencePosition - redGearPosition;
        checkTimeBefore = millis();
        float e,e1=0,e2=0,m,m1=0;
        printf("reference is %f \n",referencePosition);
        printf("%f\n",targetrotation[i]);

        while (millis() - startTime < DAQ_TIME || millis() - iterationTime < 2000)
        {
            checkTime = millis();
            if (checkTime - checkTimeBefore > LOOPTIME) // Update based on LOOPTIME
            {
                e = errorPosition;
                m = m1 +G1*e + G2*e1+ G3*e2; // set magnitude by PID control
                e2 = e1;
                e1 = e;
                m1 = m;

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
                // if (m > 100) updateDataArray(100);
                // else if (m < -100) updateDataArray(-100);
                // else updateDataArray(m);
                updateDataArray(m);
                checkTimeBefore = checkTime;
            }
        }        
    }
    softPwmWrite(MOTOR1, 0);
    softPwmWrite(MOTOR2, 0);
    for (int j = 0; j < dataIndex; j++)
    {
        fprintf(file, "%.3f,%.3f\n", dataArray[j][0], dataArray[j][1]);
    }
    fclose(file);
    
    plotGraph();
        
    return 0;
}
