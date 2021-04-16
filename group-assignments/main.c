#include "botlib.h"

/************************************************************************/
/*                                  WEEK 3                              */
/************************************************************************/

#define ASSIGNMENT_3_1 0
#define ASSIGNMENT_3_2 0
#define ASSIGNMENT_3_3 0

#if ASSIGNMENT_3_1
void zmain(void)
{
    motor_start();
    motor_forward(0, 0);
    motor_forward(100, 2950);
    motor_turn(100, 50, 1050);
    motor_forward(100, 2000);
    motor_turn(100, 50, 1050);
    motor_forward(100, 2300);
    motor_turn(100, 0, 600);
    motor_forward(100, 200);
    motor_turn(100, 90, 1200);
    motor_turn(100, 70, 1100);
    motor_forward(100, 900);
    
    motor_stop();
    
    while(FOREVER) { vTaskDelay(100); }
}
#endif
#if ASSIGNMENT_3_2

#define DISTANCE_TRESHOLD 10
const int ROBOT_SPEED = 100;
void zmain(void)
{
    init(NULL, INIT_MOTOR | INIT_ULTRA);
    while(FOREVER) 
    { 
        if (Ultra_GetDistance() < DISTANCE_TRESHOLD)
        {
            motor_backward(50, 500);
            motor_turn(0, 50, 1450);
        }
        else 
        {
            motor_forward(ROBOT_SPEED, 0);
        }
    }
}
#endif
#if ASSIGNMENT_3_3

#define DISTANCE_TRESHOLD 10
#define TURN_90_DEG 525
#define TURN_270_DEG 1575
const int ROBOT_SPEED = 100;
void zmain(void)
{
    srand(0xf4b25a3e);
    init(NULL, INIT_MOTOR | INIT_ULTRA);
    
    while(FOREVER) 
    { 
        if (Ultra_GetDistance() < DISTANCE_TRESHOLD)
        {
            motor_backward(ROBOT_SPEED, 500);
            tank_turn(rand_range(RIGHT, LEFT), ROBOT_SPEED,
                rand_range(TURN_90_DEG, TURN_270_DEG));
        }
        else 
        {
            motor_forward(ROBOT_SPEED, 0);
        }
    }
}

#endif

/************************************************************************/
/*                                  WEEK 4                              */
/************************************************************************/

#define ASSIGNMENT_4_1 0
#define ASSIGNMENT_4_2 0
#define ASSIGNMENT_4_3 0

#if ASSIGNMENT_4_1

const int ROBOT_SPEED = 50;
#define LINE_GOAL 5
void zmain(void)
{
    // Setup
    int line_count = 0;
    struct sensors_ sensors;
    refl_conf_t confs = { 9000, 9000, 11000, 11000, 9000, 9000 };
    init(&confs, INIT_IR | INIT_MOTOR);
    
    // Drive to first line.
    io_wait_SW1();
    reflectance_digital(&sensors);
    while (!on_line(&sensors)) 
    {
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Drive to end after remote signal.
    IR_wait();
    int current_line_state;
    // Calibrate goal line count based on start position.
    int prev_line_state = on_line(&sensors);
    int line_count_goal = prev_line_state ? LINE_GOAL-1 : LINE_GOAL;
    while(line_count < line_count_goal)
    {
        current_line_state = on_line(&sensors);
        // Increment counter if bot drives to line.
        if (current_line_state && !prev_line_state)
                line_count++;
        
        // Store current line state.
        prev_line_state = current_line_state;
        
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
    }
    
    // End
    motor_forward(0, 0);
    motor_stop();
    while (FOREVER) {};
}   

#endif
#if ASSIGNMENT_4_2

#define TURN_SPEED 100
const int ROBOT_SPEED = 50;
    
void zmain(void)
{
    struct sensors_ sensors;
    refl_conf_t confs = { 9000, 11000, 11000, 11000, 11000, 9000 };
    init(&confs, INIT_IR | INIT_MOTOR);
    
    // Drive to first line.
    io_wait_SW1();
    reflectance_digital(&sensors);
    while (!on_line(&sensors)) 
    {
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Cross the first line.
    IR_wait();
    while (on_line(&sensors))
    {
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Drive to the end of the track.
    while (!on_line(&sensors))
    {
        if (sensors.R2)
        {
            SetMotors(0, 1, ROBOT_SPEED, ROBOT_SPEED, 0); // TURN RIGHT
        }
        else if (sensors.L2)
        {
            SetMotors(1, 0, ROBOT_SPEED, ROBOT_SPEED, 0); // TURN LEFT
        }
        else
        {
            motor_forward(ROBOT_SPEED, 0);
        }
        reflectance_digital(&sensors);
    }
    
    // End
    motor_forward(0, 0);
    motor_stop();
    while (FOREVER) {}
}

#endif
#if ASSIGNMENT_4_3
    
#define PRE_TURN_OFFSET 250
const int ROBOT_SPEED;

void zmain(void)
{
    // Setup
    int route[] = { LEFT, RIGHT, RIGHT };
    const int ROUTE_LEN = 3;
    
    struct sensors_ sensors;
    refl_conf_t confs = { 9000, 9000, 11000, 11000, 9000, 9000 };
    init(&confs, INIT_IR | INIT_MOTOR);
    
    // Drive to first line.
    io_wait_SW1();
    reflectance_digital(&sensors);
    while (!on_line(&sensors)) 
    {
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Cross the first line.
    IR_wait();
    while (on_line(&sensors))
    {
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Drive to the end of the track
    int current_line_state;
    int prev_line_state = on_line(&sensors);
    int turn = 0;
    while (FOREVER)
    {
        current_line_state = on_line(&sensors);
        if (!current_line_state && prev_line_state)
        {
            motor_forward(ROBOT_SPEED, PRE_TURN_OFFSET);
            smart_tank_turn(route[turn++], ROBOT_SPEED, &sensors);
        }
        // Check if robot is at the finish line
        else if (current_line_state && turn == ROUTE_LEN)
        {
            break; 
        }
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
        prev_line_state = current_line_state;
    }
    
    // End
    motor_forward(0, 0);
    motor_stop();
    while (FOREVER) {};
}
    
#endif

/************************************************************************/
/*                                  WEEK 5                              */
/************************************************************************/

#define ASSIGNMENT_5_1 1
#define ASSIGNMENT_5_2 0
#define ASSIGNMENT_5_3 0

#if ASSIGNMENT_5_1
    
void zmain(void)
{

}

#endif
#if ASSIGNMENT_5_2
    
void zmain(void) 
{

}

#endif
#if ASSIGNMENT_5_3

void zmain(void)
{

}
    
#endif













































































#if 0
// Name and age
void zmain(void)
{
    char name[32];
    int age;
    
    
    printf("\n\n");
    
    printf("Enter your name: ");
    //fflush(stdout);
    scanf("%s", name);
    printf("Enter your age: ");
    //fflush(stdout);
    scanf("%d", &age);
    
    printf("You are [%s], age = %d\n", name, age);

    while(true)
    {
        BatteryLed_Write(!SW1_Read());
        vTaskDelay(100);
    }
 }   
#endif


#if 0
//battery level//
void zmain(void)
{
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed

    while(true)
    {
        char msg[80];
        ADC_Battery_StartConvert(); // start sampling
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        vTaskDelay(500);
    }
 }   
#endif

#if 0 

//Tick Timer Example
void zmain(void) 
{
	TickType_t Ttime = xTaskGetTickCount();
	TickType_t PreviousTtime = 0;

	while(true) 
	{
		while(SW1_Read()) vTaskDelay(1); // loop until user presses button
		Ttime = xTaskGetTickCount(); // take button press time
		/*Print out the time between button presses in seconds. int cast used to suppress warning messages*/
		printf("The amount of time between button presses is: %d.%d seconds\n", (int)(Ttime-PreviousTtime)/1000%60, (int)(Ttime-PreviousTtime)%1000);
		while(!SW1_Read())vTaskDelay(1); // loop while user is pressing the button
		PreviousTtime = Ttime; // remember previous press time
	}
	
}

#endif

#if 0
// button
void zmain(void)
{
    while(true) {
        printf("Press button within 5 seconds!\n");
	    TickType_t Ttime = xTaskGetTickCount(); // take start time
        bool timeout = false;
        while(SW1_Read() == 1) {
            if(xTaskGetTickCount() - Ttime > 5000U) { // too long time since start
                timeout = true;
                break;
            }
            vTaskDelay(10);
        }
        if(timeout) {
            printf("You didn't press the button\n");
        }
        else {
            printf("Good work\n");
            while(SW1_Read() == 0) vTaskDelay(10); // wait until button is released
        }
    }
}
#endif

#if 0
// button
void zmain(void)
{
    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    bool led = false;
    
    while(true)
    {
        // toggle led state when button is pressed
        if(SW1_Read() == 0) {
            led = !led;
            BatteryLed_Write(led);
            if(led) printf("Led is ON\n");
            else printf("Led is OFF\n");
            Beep(1000, 150);
            while(SW1_Read() == 0) vTaskDelay(10); // wait while button is being pressed
        }        
    }
 }   
#endif


#if 0
//ultrasonic sensor//
void zmain(void)
{
    Ultra_Start();                          // Ultra Sonic Start function
    
    while(true) {
        int d = Ultra_GetDistance();
        // Print the detected distance (centimeters)
        printf("distance = %d\r\n", d);
        vTaskDelay(200);
    }
}   
#endif

#if 0
//IR receiverm - how to wait for IR remote commands
void zmain(void)
{
    IR_Start();
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    bool led = false;
    // Toggle led when IR signal is received
    while(true)
    {
        IR_wait();  // wait for IR command
        led = !led;
        BatteryLed_Write(led);
        if(led) printf("Led is ON\n");
        else printf("Led is OFF\n");
    }    
 }   
#endif



#if 0
//IR receiver - read raw data
// RAW data is used when you know how your remote modulates data and you want to be able detect 
// which button was actually pressed. Typical remote control protocols requires a protocol specific
// state machine to decode button presses. Writing such a state machine is not trivial and requires
// that you have the specification of your remotes modulation and communication protocol    
void zmain(void)
{
    IR_Start();
    
    uint32_t IR_val; 
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    // print received IR pulses and their lengths
    while(true)
    {
        if(IR_get(&IR_val, portMAX_DELAY)) {
            int l = IR_val & IR_SIGNAL_MASK; // get pulse length
            int b = 0;
            if((IR_val & IR_SIGNAL_HIGH) != 0) b = 1; // get pulse state (0/1)
            printf("%d %d\r\n",b, l);
        }
    }    
 }   
#endif


#if 0
//reflectance
void zmain(void)
{
    struct sensors_ ref;
    struct sensors_ dig;

    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    

    while(true)
    {
        // read raw sensor values
        reflectance_read(&ref);
        // print out each period of reflectance sensors
        printf("%5d %5d %5d %5d %5d %5d\r\n", ref.L3, ref.L2, ref.L1, ref.R1, ref.R2, ref.R3);       
        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig); 
        //print out 0 or 1 according to results of reflectance period
        printf("%5d %5d %5d %5d %5d %5d \r\n", dig.L3, dig.L2, dig.L1, dig.R1, dig.R2, dig.R3);        
        
        vTaskDelay(200);
    }
}   
#endif


#if 0
//motor
void zmain(void)
{
    motor_start();              // enable motor controller
    motor_forward(0,0);         // set speed to zero to stop motors

    vTaskDelay(3000);
    
    motor_forward(100,2000);     // moving forward
    motor_turn(200,50,2000);     // turn
    motor_turn(50,200,2000);     // turn
    motor_backward(100,2000);    // moving backward
     
    motor_forward(0,0);         // stop motors

    motor_stop();               // disable motor controller
    
    while(true)
    {
        vTaskDelay(100);
    }
}
#endif

#if 0
/* Example of how to use te Accelerometer!!!*/
void zmain(void)
{
    struct accData_ data;
    
    printf("Accelerometer test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Device Ok...\n");
    }
    
    while(true)
    {
        LSM303D_Read_Acc(&data);
        printf("%8d %8d %8d\n",data.accX, data.accY, data.accZ);
        vTaskDelay(50);
    }
 }   
#endif    

#if 0
// MQTT test
void zmain(void)
{
    int ctr = 0;

    printf("\nBoot\n");
    send_mqtt("Zumo01/debug", "Boot");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 

    while(true)
    {
        printf("Ctr: %d, Button: %d\n", ctr, SW1_Read());
        print_mqtt("Zumo01/debug", "Ctr: %d, Button: %d", ctr, SW1_Read());

        vTaskDelay(1000);
        ctr++;
    }
 }   
#endif


#if 0
void zmain(void)
{    
    struct accData_ data;
    struct sensors_ ref;
    struct sensors_ dig;
    
    printf("MQTT and sensor test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Accelerometer Ok...\n");
    }
    
    int ctr = 0;
    reflectance_start();
    while(true)
    {
        LSM303D_Read_Acc(&data);
        // send data when we detect a hit and at 10 second intervals
        if(data.accX > 1500 || ++ctr > 1000) {
            printf("Acc: %8d %8d %8d\n",data.accX, data.accY, data.accZ);
            print_mqtt("Zumo01/acc", "%d,%d,%d", data.accX, data.accY, data.accZ);
            reflectance_read(&ref);
            printf("Ref: %8d %8d %8d %8d %8d %8d\n", ref.L3, ref.L2, ref.L1, ref.R1, ref.R2, ref.R3);       
            print_mqtt("Zumo01/ref", "%d,%d,%d,%d,%d,%d", ref.L3, ref.L2, ref.L1, ref.R1, ref.R2, ref.R3);
            reflectance_digital(&dig);
            printf("Dig: %8d %8d %8d %8d %8d %8d\n", dig.L3, dig.L2, dig.L1, dig.R1, dig.R2, dig.R3);
            print_mqtt("Zumo01/dig", "%d,%d,%d,%d,%d,%d", dig.L3, dig.L2, dig.L1, dig.R1, dig.R2, dig.R3);
            ctr = 0;
        }
        vTaskDelay(10);
    }
 }   

#endif

#if 0
void zmain(void)
{    
    RTC_Start(); // start real time clock
    
    RTC_TIME_DATE now;

    // set current time
    now.Hour = 12;
    now.Min = 34;
    now.Sec = 56;
    now.DayOfMonth = 25;
    now.Month = 9;
    now.Year = 2018;
    RTC_WriteTime(&now); // write the time to real time clock

    while(true)
    {
        if(SW1_Read() == 0) {
            // read the current time
            RTC_DisableInt(); /* Disable Interrupt of RTC Component */
            now = *RTC_ReadTime(); /* copy the current time to a local variable */
            RTC_EnableInt(); /* Enable Interrupt of RTC Component */

            // print the current time
            printf("%2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);
            
            // wait until button is released
            while(SW1_Read() == 0) vTaskDelay(50);
        }
        vTaskDelay(50);
    }
 }   
#endif

/* [] END OF FILE */