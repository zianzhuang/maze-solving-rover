// LED pins
#define LED_R 53       //DO
#define LED_Y 52       //DO
#define LED_B 51       //DO

// servo pins
#include <Servo.h>
int SERVO_A = 45;       //PWM
int SERVO_G = 44;       //PWM
Servo SERVO_ARM;
Servo SERVO_GRIP;

// motor pins
#define RM_BRAKE 22      //DO
#define RM_DIREC 23      //DO
#define RM_PMW 11        //PWM
#define LM_BRAKE 24      //DO
#define LM_DIREC 25      //DO
#define LM_PMW 5         //PWM
#define BM_BRAKE 26      //DO
#define BM_DIREC 27      //DO
#define BM_PMW 4         //PWM

// motor encoder pins
#define RM_ENCODER_A 18  //INTERRUPT DI
#define RM_ENCODER_B 30  //DI
#define LM_ENCODER_A 19  //INTERRUPT DI
#define LM_ENCODER_B 32  //DI
#define BM_ENCODER_A 3   //INTERRUPT DI
#define BM_ENCODER_B 34  //DI

// ultrasonic sonar pins
#define trigPin_FT 29    //DO
#define trigPin_FB 28    //DO
#define trigPin_BK 31    //DO
#define trigPin_RB 33    //DO
#define trigPin_RF 35    //DO
#define trigPin_LF 37    //DO
#define trigPin_LB 39    //DO
#define echoPin_FT 10    //PWM
#define echoPin_FB 7     //PWM
#define echoPin_BK 6     //PWM
#define echoPin_RB 8     //PWM
#define echoPin_RF 9     //PWM
#define echoPin_LF 12    //PWM
#define echoPin_LB 13    //PWM

// motor encoder variables
volatile unsigned long RMCount = 210000000;
volatile unsigned long LMCount = 210000000;
volatile unsigned long BMCount = 210000000;

// ultrasonic sonar variables
long duration_FT, duration_FB, duration_BK, duration_LF, duration_RF, duration_LB, duration_RB;
float distance_FT, distance_FB, distance_BK, distance_LF, distance_RF, distance_LB, distance_RB;
float distance_FT_prev, distance_FB_prev, distance_BK_prev, distance_LF_prev, distance_RF_prev, distance_LB_prev, distance_RB_prev;

// motor speed variables
float rm_pulse_prev, lm_pulse_prev, bm_pulse_prev;
float rm_speed, lm_speed, bm_speed;
float rm_distance, lm_distance, bm_distance;
float prev_time;

// command
int state = 0;
bool matlab_ready = false;                                   // BT command - "r" -- matlab_ready
bool rule = false;     //false - LHL || true == RHL  // one-time rule switch
bool LZ = false;                                             // LZ confirmation
char movement = 'w'; 

int wall_state;  
float side_distance = 5;                                            //default 5
