//Justin Harris
//RTOS -- Assignment 9 -- HDC 1080
//3-1-22
//This program reads inputs from the HDC1080 Temperature
//Humidity sensor, prints them out to the console, and displays
//them on a 7 segment LED.
//Additionally a driver for the 28BYJ-48 Stepper Motor is added which is controlled by the 
//on-board buttons on the Vandaluino3 PCB. 

//FreeRTOS headers
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

//C Headers
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>


//Pico Headers
#include "pico/stdlib.h"
#include "tusb.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
#define HDC1080TEMPREG 0x00
#define HDC1080HUMREG 0x01
#define HDC1080CONFIGREG 0x02
#define HDC1080ADDRESS 0x40
#define HDC1080SN1 0xFB
#define HDC1080SN2 0xFC
#define HDC1080SN3 0xFD
#define HDC1080DEVICEIDREG 0xFE
#define HDC1080DEVICEID 0xFF
#define I2C_PORT i2c1

//Step Motor Pins and steps
//IN1=12,  IN2=9, IN3=8,IN4=19
#define StepMotorIN1 12
#define StepMotorIN2 1
#define StepMotorIN3 0
#define StepMotorIN4 6

//Button pines
#define ButtonS1 19
#define ButtonS2 9
#define ButtonS3 8

//define 7-segment led pins
#define SevenSegCC1 11  //right number
#define SevenSegCC2 10  //left number

#define SevenSegA 26    //Top bar
#define SevenSegB 27    //Top right
#define SevenSegC 29    //bottom right
#define SevenSegD 18    //bottom bar
#define SevenSegE 25    //bottom left
#define SevenSegF 7     //Top Left
#define SevenSegG 28    //Middle
#define SevenSegDP 24   //decimal points

//Function prototypes for HDC1080 API
int readConfigReg();
int readMFID();
int readSN1();
int readSN2();
int readSN3();
int readTemperature();
int readHumidity();

//Function prototypes for Step Motor API
void rotateCW();
void rotateCCW();
void fullRotateFB();
void rotateOnTemp();
void rotateOnHum();
void emergencyStop();
void smStatus(int status);

//task list
void listTasks();

//Function prototypes for buttons API
void getButtons();

//Task Prototypes
void readHDC1080Task();
void stepMotorTask();
void buttonsTask();
void segLEDLeft();
void segLEDRight();

//Define Queue variable to hold humidity and temp values
QueueHandle_t mainControlQueue;
QueueHandle_t smButtonQueue;
QueueHandle_t sevSegDisQueue;
QueueHandle_t tempMotQueue;

//Define semaphore for 7segLEDs
SemaphoreHandle_t ledSem;
SemaphoreHandle_t i2cSem;
SemaphoreHandle_t buttonSem;

//Define Task handle, used to stop temp/hum sensor task in EE
TaskHandle_t hdc1080;

//buffer to vTaskList
char TaskListPtr[250];

//timer
//static const TickType_t button_wait = 2000 / portTICK_PERIOD_MS;

int main() {
    // Enable UART so we can print status output
  stdio_init_all();
  while (!tud_cdc_connected()) { sleep_ms(100);  }    
    
    // This example will use I2C1 on the default SDA and SCL pins
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    //set up Button Pins
    gpio_init(ButtonS1);
    gpio_init(ButtonS2);
    gpio_init(ButtonS3);

    gpio_set_dir(ButtonS1, GPIO_IN);
    gpio_set_dir(ButtonS2, GPIO_IN);
    gpio_set_dir(ButtonS3, GPIO_IN);

    //set up and init motor pins
    gpio_init(StepMotorIN1);
    gpio_init(StepMotorIN2);
    gpio_init(StepMotorIN3);
    gpio_init(StepMotorIN4);

    gpio_set_dir(StepMotorIN1, GPIO_OUT);
    gpio_set_dir(StepMotorIN2, GPIO_OUT);
    gpio_set_dir(StepMotorIN3, GPIO_OUT);
    gpio_set_dir(StepMotorIN4, GPIO_OUT);

    //set up and initialize 7SegLed pins
    gpio_init(SevenSegA);   //top bar
    gpio_init(SevenSegB);   //top right
    gpio_init(SevenSegC);   //bottom right?
    gpio_init(SevenSegD);   //bottom bar
    gpio_init(SevenSegE);   //bottom left
    gpio_init(SevenSegF);   //Top Left
    gpio_init(SevenSegG);   //Middle
    gpio_init(SevenSegDP);  //decimal points

    gpio_init(SevenSegCC1); //right digit
    gpio_init(SevenSegCC2); //left digit

    gpio_set_dir(SevenSegA, GPIO_OUT);  //top bar
    gpio_set_dir(SevenSegB, GPIO_OUT);  //top right
    gpio_set_dir(SevenSegC, GPIO_OUT);  //bottom right?
    gpio_set_dir(SevenSegD, GPIO_OUT);  //bottom bar
    gpio_set_dir(SevenSegE, GPIO_OUT);  //bottom left
    gpio_set_dir(SevenSegF, GPIO_OUT);  //Top Left
    gpio_set_dir(SevenSegG, GPIO_OUT);  //Middle
    gpio_set_dir(SevenSegDP, GPIO_OUT); //Decimal points

    gpio_set_dir(SevenSegCC1, GPIO_OUT);    //right digit
    gpio_set_dir(SevenSegCC2, GPIO_OUT);    //left digit

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    //initialize Semaphores
    vSemaphoreCreateBinary(ledSem);
    vSemaphoreCreateBinary(i2cSem);
    vSemaphoreCreateBinary(buttonSem);
    //buttonSem = xSemaphoreCreateMutex();

    //initialize Queues
    mainControlQueue = xQueueCreate(10, sizeof(int));
    smButtonQueue = xQueueCreate(2, sizeof(int));
    sevSegDisQueue = xQueueCreate(2, sizeof(int));
    tempMotQueue = xQueueCreate(2, sizeof(int));
    
    //initialize task to read from HDC1080
    xTaskCreate(readHDC1080Task, "readHDC1080Task", 256, NULL, 2, &hdc1080);
    
    //initialize task to control the step motor
    xTaskCreate(stepMotorTask, "stepMotorTask", 256, NULL, 2, NULL);

    //initialize task for button inputs
    xTaskCreate(buttonsTask, "buttonsTask", 256, NULL, 3, NULL);
 
    //initialize tasks to display on 7 seg leds
    xTaskCreate(segLEDLeft, "segLEDLeft", 128, NULL, 2, NULL);
    xTaskCreate(segLEDRight, "segLEDRight", 128, NULL, 2, NULL);

    //start scheduler
    vTaskStartScheduler();
  
  while(1){};
}

//This is the main task that reads the data from the HDC1080
//and sends the data to a queue to display on 7-segment LEDs
void readHDC1080Task() {

    //Initialize variables
    int configStat;
    int mfID;
    int serialNum1;
    int serialNum2;
    int serialNum3;
    int temperatureInC;
    int temperatureInF;
    int humidity;
    int clearQueue;
    int buttonSig;
    int sendTemp = 31;
    int sendHum = 32;
    int sendSM = 33;
    int motStat;
    int queueStat;
    int overFlow = 993;

    //Get Device ID values and print out on intial execution
    configStat = readConfigReg();
    mfID = readMFID();
    serialNum1 = readSN1();
    serialNum2 = readSN2();
    serialNum3 = readSN3();


    printf("Configuration Register = 0x%X\n", configStat);
    printf("Manufacturer ID = 0x%X\n", mfID);
    printf("Serial Number = %X-%X-%X\n", serialNum1, serialNum2, serialNum3);

    while(true){

        xSemaphoreTake(buttonSem, 1);
        //printf("Sensor took Semaphore\n");

        //Get current Temperature in C
        temperatureInC = readTemperature();

        //Convert Temperature in C to F
        temperatureInF = (temperatureInC * 1.8) + 32;

        //Get current Humidity
        humidity = readHumidity();

        //print statements for temperature and humidity
        //printf("Temperature in C: %d\n", temperatureInC);
        //printf("Temperature in F: %d\n", temperatureInF);
        //printf("Humidity %d\n", humidity);

        xQueueReceive(sevSegDisQueue, &buttonSig, 0);

        //if main control queue goes over 10 entries, display overflow
        queueStat = uxQueueMessagesWaiting(mainControlQueue);

        if(queueStat > 10){
            xQueueSend(mainControlQueue, &overFlow, 0);
            vTaskDelay(5000/portTICK_PERIOD_MS);
            xQueueReset(mainControlQueue);
        }

        if(buttonSig == sendHum){
            //Send humidity data to queue and delay for 5 seconds
            
            xQueueSend(mainControlQueue, &humidity, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            
            //check temp/hem queue for emergency stop
            if(xQueuePeek(mainControlQueue, &clearQueue, 0)){
                if(clearQueue == 999){
                    buttonSig = 0;
                    xSemaphoreGive(buttonSem);
                    vTaskSuspend(NULL);
                }
                else{
                    //clear Queue
                    xQueueReceive(mainControlQueue, &clearQueue, 0);
                }
            }
        }

        else if(buttonSig == sendTemp){
            //send temperature data in F and delay for 5 seconds
            xQueueSend(mainControlQueue, &temperatureInF, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);

            //check temp/hem queue for emergency stop
            if(xQueuePeek(mainControlQueue, &clearQueue, 0)){
                if(clearQueue == 999){
                    buttonSig = 0;
                    xSemaphoreGive(buttonSem);
                    vTaskSuspend(NULL);
                }
                else{
                    //clear Queue
                    xQueueReceive(mainControlQueue, &clearQueue, 0);
                }
            }
        }

        //suspend task for motor task to post status to led segments
        else if(buttonSig == sendSM){

            xQueueReceive(tempMotQueue, &motStat, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);

            xQueueSend(mainControlQueue, &motStat, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);

            //check temp/hem queue for emergency stop
            if(xQueuePeek(mainControlQueue, &clearQueue, 0)){
                if(clearQueue == 999){
                    buttonSig = 0;
                    xSemaphoreGive(buttonSem);
                    vTaskSuspend(NULL);
                }
                else{
                    //clear Queue
                    xQueueReceive(mainControlQueue, &clearQueue, 0);
                }
            }
            
        }

        xSemaphoreGive(buttonSem);



    }
}

//Task to run button function
void buttonsTask(){
    while(true){

        //take semaphore, run get buttons, release semaphore
       // xSemaphoreTake(buttonSem, 1);
        getButtons();
        vTaskDelay(1/portTICK_PERIOD_MS);
       // xSemaphoreGive(buttonSem);
    }
}

//////////////////////////////BUTTONS API START//////////////////////////////////////////////////////
void getButtons(){

    //set up variables to track which buttons are pressed
    bool button1 = false;
    bool button2 = false;
    bool button3 = false;
    bool b1WasPressed = false;
    bool b2WasPressed = false;
    bool b3WasPressed = false;

    //set up counters for button press totals and timer
    int button1Total = 0;
    int button2Total = 0;
    int button3Total = 0;
    int buttonReturn = 0;
    int buttonTimer = 0;
    int buttonTimerMax = 200;
    int bSend = 0;

    while(buttonTimer < buttonTimerMax){
        
        button1 = gpio_get(ButtonS1);
        button2 = gpio_get(ButtonS2);
        button3 = gpio_get(ButtonS3);

        //if button 1 is pressed, and not logged increment button 1 total and set
        //b1WasPressed to true to log press
        if(button1 == true && b1WasPressed == false){
            button1Total += 1;
            b1WasPressed = true;
        }
        //if button is not pressed, set log to false
        else if(button1 == false){
            b1WasPressed = false;
        }

        //if button 1 is pressed, and not logged increment button 1 total and set
        //b1WasPressed to true to log press
        if(button2 == true && b2WasPressed == false){
            button2Total += 1;
            b2WasPressed = true;
        }
        //if button is not pressed, set log to false
        else if(button2 == false){
            b2WasPressed = false;
        }

        //if button 1 is pressed, and not logged increment button 1 total and set
        //b1WasPressed to true to log press
        if(button3 == true && b3WasPressed == false){
            button3Total += 1;
            b3WasPressed = true;
        }
        //if button is not pressed, set log to false
        else if(button3 == false){
            b3WasPressed = false;
        }
        

            //delay and increment timing counter
            vTaskDelay(10/portTICK_PERIOD_MS);
            buttonTimer++;
    }

    //Error check, only accept 1 button being pressed at at time
    //button 1 baseline
    if(button1Total > 0 && ((button2Total > 0 && button3Total > 0) || ((button2Total > 0) || (button3Total > 0)))){
        printf("Error: only press 1 button at a time\n");
        button1Total = 0;
    }
    //button 2 baseline
    else if(button2Total > 0 && ((button1Total > 0 && button3Total > 0) || ((button1Total > 0) || (button3Total > 0)))){
        printf("Error: only press 1 button at a time\n");
        button2Total = 0;
    }
    //button 3 baseline
    else if(button3Total > 0 && ((button2Total > 0 && button1Total > 0) || ((button2Total > 0) || (button1Total > 0)))){
        printf("Error: only press 1 button at a time\n");
        button3Total = 0;
    }
    //don't accept any values over 3
    else if(button1Total > 3 || button2Total > 3 || button3Total > 3){
        printf("Error: button presses must be < 3 in 2 seconds\n");
        buttonReturn = 0;
    }
    //valid button 1 pressed
    else if(button1Total > 0){
        printf("button1 pressed %d times\n", button1Total);
        //move on temp
        if(button1Total == 1){
            bSend = 11;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //move on humidity
        else if(button1Total == 2){
            bSend = 12;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //emergency stop
        else if(button1Total == 3){
            bSend = 13;
            xQueueSend(smButtonQueue, &bSend, 0);
        }    
    }
    //valid button 2 pressed
    else if(button2Total > 0){
        printf("button2 pressed %d times\n", button2Total);
        
        //move motor CW
        if(button2Total == 1){
            bSend = 21;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //move motor CCW
        else if(button2Total == 2){
            bSend = 22;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //full CC CCW repeat
        else if(button2Total == 3){
            bSend = 23;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
    }
    //valid button 3 pressed
    else if(button3Total > 0){
        printf("button3 pressed %d times\n", button3Total);
        
        //display temperature
        if(button3Total == 1){
            bSend = 31;
            xQueueSend(sevSegDisQueue, &bSend, 0);
        }
        //display humidity
        else if(button3Total == 2){
            bSend = 32;
            xQueueSend(sevSegDisQueue, &bSend, 0);
        }
        //display step motor status
        else if(button3Total == 3){
            bSend = 33;
            xQueueSend(sevSegDisQueue, &bSend, 0);
        }
    }

}
//////////////////////////////BUTTONS API END//////////////////////////////////////////////////////


//Step Motor Task that controls the how the step motor moves
void stepMotorTask(){
    //set up variables for how to move
    int buttonSig;
    int moveTemp = 11;
    int moveHum = 12;
    int stopEE = 13;
    int moveCW = 21;
    int moveCCW = 22;
    int fullMove = 23;
    
    //motor status
    int status;

    while(true){

        xSemaphoreTake(buttonSem, 1);
        xQueueReceive(smButtonQueue, &buttonSig, 0);

        //signals received from button 1
        if(buttonSig == moveTemp){
            status = 994;
            xQueueSend(tempMotQueue, &status, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateOnTemp();
            xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == moveHum){
            status = 995;
            xQueueSend(tempMotQueue, &status, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateOnHum();
            xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == stopEE){
            emergencyStop();

            //resume test rotation
            buttonSig = 23;
            xSemaphoreGive(buttonSem);
        }

        //signals received from Button 2
        else if(buttonSig == moveCW){
            status = 996;
            xQueueSend(tempMotQueue, &status, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateCW();
            xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == moveCCW){
            status = 997;
            xQueueSend(tempMotQueue, &status, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateCCW();
            xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == fullMove){
            fullRotateFB();
            xSemaphoreGive(buttonSem);
        }

        //signal received from Button 3
        /*else if(buttonSig == 33){
            smStatus(status);
            xSemaphoreGive(buttonSem);

        }*/
        //if no button signal received, give up semaphore
        else{
            xSemaphoreGive(buttonSem);
        }

        
    }
}
//////////////////////////////STEP MOTOR API START//////////////////////////////////////////////////////

//vTaskList function
void listTasks(){
    vTaskList(TaskListPtr);
    printf("task_n   task_s  priority        ss     tn\n");
    printf("%s\n", TaskListPtr);
}
//Function in the Step Motor API to rotate clockwise
void rotateCW(){
    //////////////////////////////////////////////
    //steps for clockwise direction, full step
    // #step    1   2   3   4
    //          --------------
    //
    //pin13     1   1   0   0
    //pin12     0   1   1   0
    //pin9      0   0   1   1
    //pin6      1   0   0   1
    //////////////////////////////////////////////


    //step 1
    gpio_put(StepMotorIN1, 1);
    gpio_put(StepMotorIN2, 0);
    gpio_put(StepMotorIN3, 0);
    gpio_put(StepMotorIN4, 1);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 2
    gpio_put(StepMotorIN1, 1);
    gpio_put(StepMotorIN2, 1);
    gpio_put(StepMotorIN3, 0);
    gpio_put(StepMotorIN4, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 3
    gpio_put(StepMotorIN1, 0);
    gpio_put(StepMotorIN2, 1);
    gpio_put(StepMotorIN3, 1);
    gpio_put(StepMotorIN4, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 4
    gpio_put(StepMotorIN1, 0);
    gpio_put(StepMotorIN2, 0);
    gpio_put(StepMotorIN3, 1);
    gpio_put(StepMotorIN4, 1);
    vTaskDelay(10/portTICK_PERIOD_MS);

}

//Function to move the step motor in counter clockwise direction
void rotateCCW(){
        //////////////////////////////////////////////
    //steps for counter-clockwise direction, full step
    // #step    1   2   3   4
    //          --------------
    //
    //pin13     0   0   1   1
    //pin12     0   1   1   0
    //pin9      1   1   0   0
    //pin6      1   0   0   1
    //////////////////////////////////////////////

    //step 1
    gpio_put(StepMotorIN1, 0);
    gpio_put(StepMotorIN2, 0);
    gpio_put(StepMotorIN3, 1);
    gpio_put(StepMotorIN4, 1);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 2
    gpio_put(StepMotorIN1, 0);
    gpio_put(StepMotorIN2, 1);
    gpio_put(StepMotorIN3, 1);
    gpio_put(StepMotorIN4, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 3
    gpio_put(StepMotorIN1, 1);
    gpio_put(StepMotorIN2, 1);
    gpio_put(StepMotorIN3, 0);
    gpio_put(StepMotorIN4, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 4
    gpio_put(StepMotorIN1, 1);
    gpio_put(StepMotorIN2, 0);
    gpio_put(StepMotorIN3, 0);
    gpio_put(StepMotorIN4, 1);
    vTaskDelay(10/portTICK_PERIOD_MS);
}

//Function to rotate 1 the step motor one full revolution clockwise
//and then 1 full revolution counter clockwise
void fullRotateFB(){

    int max_steps = 500;
    int status;
    int clearQueue;

    //clear previous status
    xQueueReceive(tempMotQueue, &clearQueue, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);

    status = 998;
    xQueueSend(tempMotQueue, &status, 0);
    vTaskDelay(200/portTICK_PERIOD_MS);

    //one full rotation clockwise
    for(int i = 0; i <= max_steps; i++){
        //check for emergency stop
        rotateCW();
    }

    //one full rotation counter-clockwise
    for(int i = 0; i <= max_steps; i++){
        rotateCCW();
    }

    //delay for 10 seconds
    //vTaskDelay(1000/portTICK_PERIOD_MS);

}

//Function that rotates on changes in temperature
void rotateOnTemp(){
    int currentTemp;
    int tempCheck;
    static int prevTemp;
    int numSteps = 0;

    //look at mainControlQueue and get a copy of the current value
    if(xQueuePeek(mainControlQueue, &tempCheck, 0)){

        //ignore motor status numbers
        if(tempCheck < 992){
            currentTemp = tempCheck;
        }
        else{
            currentTemp = prevTemp;
        }

        //check if previous temp or current temp is larger
        //current Temp is larger, move clockswise for x steps
        if(currentTemp > prevTemp){
            numSteps = currentTemp - prevTemp;
            //printf("number of steps: %d\n", numSteps);

            prevTemp = currentTemp;

            for(int i = 0; i < numSteps; i++){
                rotateCW();
            }
        }

        //if temp decreasing, rotate ccw for x steps
        else if(currentTemp < prevTemp){
            numSteps = prevTemp - currentTemp;

            prevTemp = currentTemp;

            for(int i = 0; i < numSteps; i++){
                rotateCCW();
            }
        }
        //if no change, delay
        else{
            //xSemaphoreGive(buttonSem);
            vTaskDelay(2000/portTICK_PERIOD_MS);
            //xSemaphoreTake(buttonSem, 0);
        }
    }

}

//Function that rotates motor on changes in humidity
void rotateOnHum(){
    int currentHum;
    int humCheck;
    static int prevHum;
    int numSteps = 0;

    //look at mainControlQueue and get a copy of the current value
    if(xQueuePeek(mainControlQueue, &humCheck, 0)){

        //ignore motor status numbers
        if(humCheck < 992){
            currentHum = humCheck;
        }
        else{
            currentHum = prevHum;
        }

        //check if previous temp or current temp is larger
        //current Temp is larger, move clockswise for x steps
        if(currentHum > prevHum){
            numSteps = currentHum - prevHum;
            printf("number of steps: %d\n", numSteps);

            prevHum = currentHum;

            for(int i = 0; i < numSteps; i++){
                rotateCW();
            }
        }

        //if temp decreasing, rotate ccw for x steps
        else if(currentHum < prevHum){
            numSteps = prevHum - currentHum;

            prevHum = currentHum;

            for(int i = 0; i < numSteps; i++){
                rotateCCW();
            }
        }
        //if no change, delay
        else{
            gpio_put(StepMotorIN1, 0);
            gpio_put(StepMotorIN2, 0);
            gpio_put(StepMotorIN3, 0);
            gpio_put(StepMotorIN4, 0);
            //xSemaphoreGive(buttonSem);
            vTaskDelay(2000/portTICK_PERIOD_MS);
            //xSemaphoreTake(buttonSem, 0);
        }
    }
}

//Function to stop motor, display EE to 7 seg
void emergencyStop(){

    //intitialize variables
    int stop = 999;
    int clearQueue;
    int status;

    //send signal to 7 seg display
    xQueueSend(mainControlQueue, &stop, 0);

    //stop motor for 5 seconds
    gpio_put(StepMotorIN1, 0);
    gpio_put(StepMotorIN2, 0);
    gpio_put(StepMotorIN3, 0);
    gpio_put(StepMotorIN4, 0);

    vTaskDelay(5000/portTICK_PERIOD_MS);

    //resume readHDC1080 task
    vTaskResume(hdc1080);

    //clear message from queue
    xQueueReceive(mainControlQueue, &clearQueue, 0);

    //send test status
    status = 998;
    xQueueSend(tempMotQueue, &status, 0);
    vTaskDelay(200/portTICK_PERIOD_MS);
    
}

//////////////////////////////STEP MOTOR API END//////////////////////////////////////////////////////

//////////////////////////////HDC1080 API START//////////////////////////////////////////////////////

//Function to read a return the Configuration Register Status
//Value will be printed once at the beginning of the
//readHDC1080Task  task
int readConfigReg(){

    uint8_t cfReg[2];    
    uint8_t cfRegVal = HDC1080CONFIGREG;

    int ret;

      //write blocking for Configuration Register
      ret = i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &cfRegVal, 1, false);
      vTaskDelay(100/portTICK_PERIOD_MS);

      //read blocking. Read Configuration Register
      ret = i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, cfReg, 2, false);
      int fullcfReg = cfReg[0]<<8|cfReg[1];

      return fullcfReg;

}

//Function to read and return the Manufacturing ID
//Value will be printed once at the beginning of the
//readHDC1080Task  task
int readMFID(){

      uint8_t manufactID[2];
      uint8_t mfVal = HDC1080DEVICEIDREG;
      int ret;

      //write blocking for MF ID
      ret = i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &mfVal, 1, false);
      vTaskDelay(100/portTICK_PERIOD_MS);

      //read blocking. Return Full Manufacturing ID
      ret = i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, manufactID, 2, false);
      int fullMfID = manufactID[0]<<8|manufactID[1];

      return fullMfID;

}

//Function to read and return the 1st block of the Serial Number
//Value will be printed once at the beginning of the
//readHDC1080Task  task
int readSN1(){

      uint8_t sn1[2];
      uint8_t sn1RegVal = HDC1080SN1;
      int ret;

      //write blocking for sn1 register
      ret = i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &sn1RegVal, 1, false);
      vTaskDelay(100/portTICK_PERIOD_MS);

      //read blocking for sn1
      ret = i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, sn1, 2, false);
      int fullSN1 = sn1[0]<<8|sn1[1];
      
      return fullSN1;
}

//Function to read and return the 2nd block of the Serial Number
//Value will be printed once at the beginning of the
//readHDC1080Task  task
int readSN2(){

      uint8_t sn2[2];
      uint8_t sn2RegVal = HDC1080SN2;
      int ret;

      //write blocking for sn1 register
      ret = i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &sn2RegVal, 1, false);
      vTaskDelay(100/portTICK_PERIOD_MS);

      //read blocking for sn1
      ret = i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, sn2, 2, false);
      int fullSN2 = sn2[0]<<8|sn2[1];
      
      return fullSN2;
}

//Function to read and return the 3rd block of the Serial Number
//Value will be printed once at the beginning of the
//readHDC1080Task  task
int readSN3(){

      uint8_t sn3[2];
      uint8_t sn3RegVal = HDC1080SN3;
      int ret;

      //write blocking for sn1 register
      ret = i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &sn3RegVal, 1, false);
      vTaskDelay(100/portTICK_PERIOD_MS);

      //read blocking for sn1
      ret = i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, sn3, 2, false);
      int fullSN3 = sn3[0]<<8|sn3[1];
      
      return fullSN3;
}

//This function reads the current temperature from the HDC1080.
//This function is called once every 10 seconds
int readTemperature(){

  uint8_t temperatue[2];
  uint8_t tempRegVal = HDC1080TEMPREG;
  int ret;
  int fullTemperatureC;
  int fullTemperatureF;

    //write block for temperature
      ret = i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &tempRegVal, 1, false);
      vTaskDelay(100/portTICK_PERIOD_MS);

      //read block for temperature
      ret = i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, temperatue, 2, false);
      int fullTemperature = temperatue[0]<<8|temperatue[1];

      float actualTempC = (fullTemperature / 65536.0) * 165 - 40;

      fullTemperatureC = round(actualTempC);

      return fullTemperatureC;

}

//This function reads the current humidity from the HDC1080
//This function is called once every 10 seconds
int readHumidity(){

      uint8_t humidty[2];
      uint8_t humRegVal = HDC1080HUMREG;
 
      int ret;
      int rndHumidity;

      //write block for humidity
      ret = i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &humRegVal, 1, false);
      vTaskDelay(100/portTICK_PERIOD_MS);

      //read block for humidity
      ret = i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, humidty, 2, false);
      int fullHumidity = humidty[0]<<8|humidty[1];

      float actualHumidity = (fullHumidity / 65536.0)*100.0;

      rndHumidity = round(actualHumidity);      

      return rndHumidity;

}
//////////////////////////////HDC1080 API END//////////////////////////////////////////////////////

//////////////////////////////7SegLED API START//////////////////////////////////////////////////////
//This function controls numbers displayed on the left number
//of the 7 segment LED. segLEDLeft and segLED right share
//the ledSem semaphore to alternate blinking sides
//quickly so there is no flashing.
void segLEDLeft()
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    // initialize digital pin LED_BUILTIN as an output.

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    int leftNum;

    while(true){
        
        //initialize counter variable
        int i;

        //peek at current value in the queue and get the left
        //side by dividing by 10
        if(xQueuePeek(mainControlQueue, &leftNum, 0)){
            
            //update value if left queue is not empty and not equal to 999
            if(leftNum > 992){
                leftNum = leftNum;
           }
           else{
               leftNum = leftNum / 10;
           }
        }

        //switch case for which segments to light up on the left
        //side of the 7 segment led to display leftNum.
        //In each case, this
        //function will take and release the shared semaphore.
        switch(leftNum){

        //Case 0
        case 0 :
            for (i = 0; i < 15; i++){
                xSemaphoreTake(ledSem, 1);
                
                gpio_put(SevenSegCC1, 0);   //right multiplex
                gpio_put(SevenSegCC2, 1);   //left multiplex
                gpio_put(SevenSegA, 1);    //top bar   
                gpio_put(SevenSegB, 1);    //top right  
                gpio_put(SevenSegC, 1);    //bottom right
                gpio_put(SevenSegD, 1);    //bottom bar
                gpio_put(SevenSegE, 1);    //bottom left
                gpio_put(SevenSegF, 1);    //Top Left
                gpio_put(SevenSegG, 0);    //Middle              
                
                xSemaphoreGive(ledSem);
                
                vTaskDelay(1/portTICK_PERIOD_MS);
            }
            break;

        //case 1
        case 1 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle             
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 2
        case 2 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                            
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 3
        case 3 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle               
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 4
        case 4 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle              
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 5
        case 5 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                 
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 6
        case 6 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 7
        case 7 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle               
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 8
        case 8 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 9
        case 9 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //overflow OF
        case 993:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //move on Temp -- PP
        case 994:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //move on humidity -- HH
        case 995:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //full cw -- FF
        case 996:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //constant ccw - bb
        case 997:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //full rotation back and forth
        case 998:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //display E
        case 999:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 0);   //right multiplex
            gpio_put(SevenSegCC2, 1);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;
        }
    }
}

//This function controls numbers displayed on the right number
//of the 7 segment LED. segLEDLeft and segLED right share
//the ledSem semaphore to alternate blinking sides
//quickly so there is no flashing.
void segLEDRight()
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    // initialize digital pin LED_BUILTIN as an output.

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    int rightNum;

    while(true){

        //initialize variables for counter
        int i;
        
        //peek at number in the queue and get the right digit by
        //perfoming a mod on the number in the queue
        if(xQueuePeek(mainControlQueue, &rightNum, 0)){
            
            //if queue number isn't 999, continue
            //else it's the Emergency stop code.
            if(rightNum > 992){
                rightNum = rightNum;
            }
            else{
                //update rightNum if queue is not empty
                rightNum = rightNum % 10;
            }
        }
        
        //Switch case to determine which segments of the right digit
        //on the 7 segment led to light up based on rightNum. 
        //In each case, this
        //function will take and release the shared semaphore.
        switch(rightNum){
        //Case 0
        case 0 :
        for (i = 0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle              
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 1
        case 1 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle              
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 2
        case 2 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle             
            
            xSemaphoreGive(ledSem);
           
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 3
        case 3 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle  
                               
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 4
        case 4 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle  
                          
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 5
        case 5 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);
            
            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                 
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 6
        case 6 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);
            
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 7
        case 7 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 0);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle               
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 8
        case 8 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //case 9
        case 9 :
        for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 0);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //overflow OF
        case 993:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //mvoe on Temp -- PP
        case 994:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //move on humidity -- HH
        case 995:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 1);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //full cw -- FF
        case 996:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 0);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //constant ccw - bb
        case 997:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 0);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 1);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //full rotation back and forth - CC
        case 998:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 0);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;

        //Emergncy Stop Display
        case 999:
            for(i=0; i < 15; i++){
            xSemaphoreTake(ledSem, 1);

            gpio_put(SevenSegCC1, 1);   //right multiplex
            gpio_put(SevenSegCC2, 0);   //left multiplex
            gpio_put(SevenSegA, 1);    //top bar   
            gpio_put(SevenSegB, 0);    //top right  
            gpio_put(SevenSegC, 0);    //bottom right
            gpio_put(SevenSegD, 1);    //bottom bar
            gpio_put(SevenSegE, 1);    //bottom left
            gpio_put(SevenSegF, 1);    //Top Left
            gpio_put(SevenSegG, 1);    //Middle                
            
            xSemaphoreGive(ledSem);

            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        break;
        }
    }
}
//////////////////////////////7SegLED API END//////////////////////////////////////////////////////



