#include "mbed.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"

Serial pc(PB_6, PB_7); //D5, D4 //PB_6, PB_7
I2C i2c(PA_10, PA_9); //sda, scl PA_10, PA_9 D0, D1
//(PinName mosi, PinName miso, PinName sck, PinName _cepin, PinName _csnpin)
RF24 NRF24L01(PB_5, PB_4, PB_3, PA_12, PA_11);
//RF24 NRF24L01(D11, D12, D13, D2, D10);

AnalogIn JOYPIN1(PA_0); //A0
AnalogIn JOYPIN2(PA_1); //A1
AnalogIn JOYPIN3(PA_3); //A2
AnalogIn JOYPIN4(PA_4); //A3

AnalogIn POT1(PA_7); //A6
AnalogIn POT2(PA_6); //A5
AnalogIn POT3(PA_5); //A4

DigitalIn BUTTON1(PA_2); //A7
DigitalIn BUTTON2(PA_8); //D9

DigitalOut MCU_LED(PB_0); //D3
DigitalOut MCU_LED2(PB_1); //D6

//MPU
float mpu_temp_read;
int32_t t_fine;
int16_t gyro_raw[3], acc_raw[3], mag_raw[3];
float gyro_offset[3], acc_offset[3];
float mag_factory[3];
int16_t angle[2];

//magnetometer
float x_offset = -1320.5, y_offset = 1301.8, z_offset = -633.25;
float x_scale = 0.9819, y_scale = 0.9957, z_scale = 1.0233;

//RF
const uint64_t pipe = 0x1212121212LL;
float PITCH2 = 0, ROLL2 = 0, YAW2 = 0, THROTTLE2 = 0;
int16_t PITCH = 0, ROLL = 0, YAW = 0, THROTTLE = 0;
int8_t recv[30];
int8_t trans[30];
int8_t ackMessage[30];
int16_t ackMessage2[30];
uint32_t ackMessage3;
int32_t ackMessage4[4];
int8_t ackMessage5[3];

//Timer Loop
Timer timer1;

int8_t TEMP_BUTTON1, TEMP_BUTTON2;
int8_t temp_led = 1, temp_led2 = -1;
int loop_count = 0, loop_count2 = 0, loop_count3 = 0, loop_count4 = 0, loop_count5 = 0; 
int loop_count4_max = 50;
float OFFSET1, OFFSET2, OFFSET3;

int8_t current_mode = 0;

int8_t toggle_mode = 1;

//rf ack
int8_t ack_count = 0;
uint8_t cnt = 0;

int32_t temp_time;
uint8_t temp_time2;

//-----------------------------------------------------------------------------------------

int16_t constrain_int16(int16_t x, int16_t min, int16_t max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}

int32_t constrain_int32(int32_t x, int32_t min, int32_t max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}

float constrain_float(float x, float min, float max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}

//------------------------------------------------------------------------------------------------
int main() {
    
    //disableDebugPorts();
    //pc.baud(230400); //115200
    //i2c.frequency(400000);

    //RF
    NRF24L01.begin();
    NRF24L01.setDataRate(RF24_2MBPS); //RF24_2MBPS
    NRF24L01.setChannel(99);
    NRF24L01.setPayloadSize(28); //28
    NRF24L01.setAddressWidth(5);
    NRF24L01.setRetries(2,4); //1,3 2,8
    
    NRF24L01.enableAckPayload();
    NRF24L01.openWritingPipe(pipe);
    
    timer1.start();
    
//------------------------------------------------------------------------------------------------
    while(1) {
  
    loop_count = loop_count + 1;
    loop_count2 = loop_count2 + 1;
    loop_count4 = loop_count4 + 1;

    TEMP_BUTTON1 = (int8_t)BUTTON1;
    TEMP_BUTTON2 = (int8_t)BUTTON2;

        THROTTLE2 = JOYPIN1.read(); //left side vertical
        PITCH2 = JOYPIN3.read() - 0.5f; //65536 //up positive
        ROLL2 = JOYPIN4.read() - 0.5f; //right positive
        YAW2 = JOYPIN2.read() - 0.5f; // right positive
        
        OFFSET1 = POT1.read() - 0.5f; //left //reduce value counter-clockwise
        OFFSET2 = POT2.read() - 0.5f; //middle
        OFFSET3 = POT3.read() - 0.5f; //right
        //pc.printf("%f, %f, %f\n\r", OFFSET1, OFFSET2, OFFSET3);
        
        THROTTLE = (int16_t)(THROTTLE2 * 1024 * 1.15f);
        THROTTLE = constrain_int16(THROTTLE, 0, 1500);
        
        PITCH = -(int16_t)((PITCH2 * 1024  / 2 - OFFSET2 * 1024 / 4)); //-
        ROLL = -(int16_t)((ROLL2 * 1024 / 2 - OFFSET1 * 1024 / 4)); //-
        YAW = (int16_t)((- YAW2 * 1024 / 6 + OFFSET3 * 1024 / 2));

    if (loop_count >= 4){
        
        trans[0] = ((char *)&ROLL)[0];
        trans[1] = ((char *)&ROLL)[1];
        trans[2] = ((char *)&PITCH)[0]; //0;//
        trans[3] = ((char *)&PITCH)[1];
        trans[4] = ((char *)&YAW)[0];
        trans[5] = ((char *)&YAW)[1];
        trans[6] = ((char *)&THROTTLE)[0];
        trans[7] = ((char *)&THROTTLE)[1];
        trans[8] = (char)BUTTON1;
        trans[9] = (char)BUTTON2;
        
        NRF24L01.powerUp();
        NRF24L01.write(trans, 10);
        
/*       
        if (NRF24L01.isAckPayloadAvailable()){
            NRF24L01.read(ackMessage, 14); //28 23
            
            ackMessage2[0] = *(int16_t*)(&ackMessage[0]); 
            ackMessage2[1] = *(int16_t*)(&ackMessage[2]); 
            ackMessage2[2] = *(int16_t*)(&ackMessage[4]); 
            
            ackMessage2[3] = *(int16_t*)(&ackMessage[6]);
            ackMessage2[4] = *(int16_t*)(&ackMessage[8]);
            
            ackMessage2[5] = *(int16_t*)(&ackMessage[10]);
            ackMessage2[6] = *(int16_t*)(&ackMessage[12]);
            
            temp_time2 = (uint8_t)(temp_time / 100); //1000
            
            temp_time = 0;
  
        }    
*/

    loop_count = 0;
    }

    if (loop_count4 >= loop_count4_max){
        
        if (temp_led == 1) {
            MCU_LED = 1;
            //MCU_LED2 = 0;
            }
        if (temp_led == -1) {
            MCU_LED = 0;
            //MCU_LED2 = 1;
            }
        temp_led = -1 * temp_led;  
        loop_count4 = 0;
    }

        while (timer1.read_us() < 2500); //400 Hz
        
        temp_time = temp_time + timer1.read_us();
        
        timer1.reset();
        
    }
    
}
