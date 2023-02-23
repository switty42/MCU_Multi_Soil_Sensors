/*  Mulit Soil Sensor Project using MUX
    Start date 11-17-22
    Authors - Stephen Witty / SAM @WW
    Source - started with blank sketch (setup and loop) with includes copied from RAK Wisblock example

    Project should be considered demo/POC level ready only

    V1 11-17-22 Started dev
    V2 11-18-22 Gathering sensor data from sensors and displaying, releasing to Sam
    V3 11-18-22 Minor mods, releasing to Sam
    V4 11-28-22 Added LoRa reporting and pump control.  Example LoRa code was used from RAK IDE Arduino example
    V5 12-2-22  Added averaging raw data, mapped raw data to 1-80 scale, switched to single byte reporting per sensor
    V6 12-14-22 Accounted for millis rollover in time delay keeping, watch dog installed
    V7 dev
    V8 02-22-22 Cleanup for github release
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <Adafruit_SleepyDog.h>
#include "keys.h" //Contain Helium / LoRa keys

//Start Sensor calibration values
#define HIGH0     676
#define LOW0      264
#define HIGH1     689
#define LOW1      258
#define HIGH2     694
#define LOW2      270
#define HIGH3     689
#define LOW3      308
#define HIGH4     912
#define LOW4      288
//End Sensor calibration values

#define VERSION "8"                         //Software release number
#define NUM_SOIL_SENSORS 5                  //Number of soil sensors attached to the system
#define MAX_SOIL_SENSORS 8                  //Max number of soil sensors the code currently supports
#define SOIL_POLL_TIME 1000                 //Number of milliseconds between soil sensor samples
#define LORA_SEND_TIME 120000               //Number of milliseconds between LoRa transmits
#define PUMP_ON_SETTING 20                  //Soil Sensor threshold for pump ON - after calibration
#define PUMP_OFF_SETTING 60                 //Soil Sensor threshold for pump OFF - after calibration
#define PUMP_SENSOR_NUMBER 0                //Soil sensor number that controls pump, numbered from zero
#define PUMP_ENABLED true                   //Is pump functionality turned on?
#define SOIL_SAMPLES 5                      //Number of sensor samples to take for average value
#define SENSOR_HIGH_VAL 699                 //Calibrated in the air
#define SENSOR_LOW_VAL 263                  //Calibrated in water
#define ENABLE_WATCH_DOG true               //Controls watchdog and verifies we get a lora sends
#define LORA_SEND_TIME_FAILURE_LIMIT 3600000//If watchdog is enabled and lora send failes for this many microsecs, reset mcu

void set_soil_sensor(int);            //Function definition for set the soil sensor to sample from
void send_lora_frame(uint8_t[],int);  //Send LoRa data

/**************** LoRa Parms - from RAK Wisblock RAK4631 example in IDE *********************************/
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60									            	        /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_3									                    /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5							                  /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										                    /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;					                  /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;           /* Region:*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;				        /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							                /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };

//Moved Helium / LoRa keys to keys.h 

// Private definition
#define LORAWAN_APP_DATA_BUFF_SIZE 64                                         //< buffer size of the data to be transmitted.
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

static uint32_t count = 0;
static uint32_t count_fail = 0;
/******END LoRa Parms / Setup defintions ***********************************************************************************/

struct SENSOR 
{
  int high_val;  //High value for sensor
  int low_val;   //Low value for sensor
  uint16_t raw_data[SOIL_SAMPLES];  //Raw data storage
  uint16_t avg; //Average of raw data
  uint8_t calibrated; //Average value mapped to commerical sensor range
  uint8_t percentage; //Aaverage value mapped to plain percentage
} sensor[NUM_SOIL_SENSORS];

bool lora_ready=false;  //Turn true when MCU joins LoRa network, used to send first loRa data frame right away

void setup() 
{
 digitalWrite(LED_BUILTIN,LOW);  //Using built in LED (Green) to indicate connection to Helium / LoRa network
 Serial.begin(115200);

 unsigned long t=millis()+5000;  //Wait for serial port to become available
 while(!Serial) { if (t< millis()) break; }
 delay(1000); 

 Serial.print("Multi Soil Sensor project Version: "); Serial.println(VERSION);
 Serial.print("Number of soil sensors configured: "); Serial.println(NUM_SOIL_SENSORS);
 Serial.print("Soil Sensor sample interval in milli-seconds: "); Serial.println(SOIL_POLL_TIME);
 Serial.print("LoRa transmit interval in milli-seconds: "); Serial.println(LORA_SEND_TIME);
 Serial.print("Pump on threshold: "); Serial.println(PUMP_ON_SETTING);
 Serial.print("Pump off threshold: "); Serial.println(PUMP_OFF_SETTING);
 Serial.print("Pump control soil sensor number: "); Serial.println(PUMP_SENSOR_NUMBER);
 Serial.print("Soil raw low value: "); Serial.println(SENSOR_LOW_VAL);
 Serial.print("Soil raw high value: "); Serial.println(SENSOR_HIGH_VAL);
 Serial.print("Soil samples for average: "); Serial.println(SOIL_SAMPLES);
 Serial.print("LoRa send time fail limit: "); Serial.println(LORA_SEND_TIME_FAILURE_LIMIT);
 Serial.print("Pump enabled: "); 
 if (PUMP_ENABLED) Serial.print("YES");
  else Serial.print("NO");
 Serial.println("");
 Serial.println("");

  if (ENABLE_WATCH_DOG)
  {
    Serial.println("Turning on watchdog");
    Watchdog.enable(8000);  //Set watch dog to 8 seconds, the maximum, must feed the dog more often than 8 seconds
  }

  pinMode(WB_IO1,OUTPUT);  //Connected to MUX S0 - These output lines control which MUX sensor port is selected
  pinMode(WB_IO3,OUTPUT);  //Connected to MUX S1
  pinMode(WB_IO4,OUTPUT);  //Connected to MUX S2
  pinMode(WB_IO5,OUTPUT);  //Connected to Pump relay
  pinMode(WB_IO6,OUTPUT);  //Connected to Pump running LED

  digitalWrite(WB_IO1,LOW); //Set MUX control pins to known state
  digitalWrite(WB_IO3,LOW);
  digitalWrite(WB_IO4,LOW);
  digitalWrite(WB_IO5,LOW); //Set Pump relay off
  digitalWrite(WB_IO6,LOW); //Set Pump running LED off

  //Setup calibration values
  sensor[0].high_val=HIGH0;
  sensor[0].low_val=LOW0;
  sensor[1].high_val=HIGH1;
  sensor[1].low_val=LOW1;
  sensor[2].high_val=HIGH2;
  sensor[2].low_val=LOW2;
  sensor[3].high_val=HIGH3;
  sensor[3].low_val=LOW3;
  sensor[4].high_val=HIGH4;
  sensor[4].low_val=LOW4;

/** LoRa setup from RAK4631 IDE example **************/
  // Initialize LoRa chip.
  lora_rak4630_init();

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  // Initialize LoRaWan
  uint32_t err_code;
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, true, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }
  // Start Join procedure
  lmh_join();
/*** End LorA setup *********************************************************************/

  Serial.println("Entering run mode........"); Serial.flush();
}

//Variables for main routine  
unsigned long time_lora_send_start=0,time_soil_sample_start=0;
bool pump_running=false;
int sample_index=0;              //Which sample are we filling in sample array
bool sample_bucket_full=false;   //Is there enough soil samples to take average
bool first_run_sample_loop=true; //Used to force soil sample withot initial dealy
bool first_run_lora_send=true;   //Used to force first lora send withotu initial delay
unsigned long last_successful_lora_send_time=0;  //Used to time how long it has been since successful lora send for possible reset

void loop() 
{ 
  delay(500); //save power and avoid loop spin
  if (ENABLE_WATCH_DOG)
  {
    Watchdog.reset(); //Feed the dog - must do this every 8 seconds or a MCU reset occurs
    if (millis()-last_successful_lora_send_time > (unsigned long) LORA_SEND_TIME_FAILURE_LIMIT)  //If LorA sends are failing for some time limit
    {
      Serial.println("ERROR - LoRa send time failure limit reached resetting MCU!!!!!");
      while(1);  //Let the watchdog dog catch and reset
    }
  }

  if (millis()-time_soil_sample_start > (unsigned long) SOIL_POLL_TIME || first_run_sample_loop==true)
  {
    first_run_sample_loop=false;
    for (int a=0;a<NUM_SOIL_SENSORS;a++)
    {
      set_soil_sensor(a);  //Set the soil sensor number to sample from (numbered from zero)
      sensor[a].raw_data[sample_index]=analogRead(A1);  //A1 pin is ADC on RAK, connected to MUX sig line which is connected to soil sensors
      Serial.print(sensor[a].raw_data[sample_index]);
      if (sample_bucket_full==true) 
        { 
          Serial.print("["); Serial.print(sensor[a].avg); Serial.print("]");
          Serial.print("("); Serial.print(sensor[a].percentage); Serial.print("%)");
          Serial.print("{"); Serial.print(sensor[a].calibrated); Serial.print("}");          
        }
      Serial.print(" ");
    }
    sample_index++;
    if (sample_index==SOIL_SAMPLES) { sample_index=0; sample_bucket_full=true; }

    if (sample_bucket_full==true) //Do sensor value average
    {
      for (int c=0;c<NUM_SOIL_SENSORS;c++)
      {
       int total=0;
       for (int b=0;b<SOIL_SAMPLES;b++) total=total+sensor[c].raw_data[b];
       sensor[c].avg=total/SOIL_SAMPLES;
      
       //Calculating percentage of water in sample and putting in sensor_data_percentage
       //The sensor data are higher is drier and lower is wetter, so reversing
       //Higher percentage is higher water content
       //Also mapping data onto commerical sensor scale of 1 to 80 (higher also wetter)
       //FYI - map function uses integer math so some accuracy is lost
       if (sensor[c].avg>=sensor[c].high_val) //Map most dry values
       {
        sensor[c].percentage=0;
        sensor[c].calibrated=1;
       }
        else if (sensor[c].avg<=sensor[c].low_val) //Map most wet values
        { 
          sensor[c].percentage=100;
          sensor[c].calibrated=80;
        }
          else
          {
            //Map to a percentage 
            sensor[c].percentage=map(sensor[c].avg,sensor[c].low_val,sensor[c].high_val,100,0);
            //Map from 1 to 80 which matches commerical sensor
            sensor[c].calibrated=map(sensor[c].avg,sensor[c].low_val,sensor[c].high_val,80,1);             
          }
      }
    }

    if (pump_running==true) Serial.print("   Pump running");
    Serial.println("");

    //Turn pump relay on and off depending on soil sensor data
    if (PUMP_ENABLED && sample_bucket_full==true)
    {
      if (sensor[PUMP_SENSOR_NUMBER].percentage>PUMP_OFF_SETTING) { digitalWrite(WB_IO6,LOW); digitalWrite(WB_IO5,LOW); pump_running=false; }
      if (sensor[PUMP_SENSOR_NUMBER].percentage<PUMP_ON_SETTING) { digitalWrite(WB_IO6,HIGH); digitalWrite(WB_IO5,HIGH); pump_running=true; }
    }
    time_soil_sample_start=millis();
  } 
  if ((millis()-time_lora_send_start > (unsigned long) LORA_SEND_TIME || first_run_lora_send==true) && lora_ready==true && sample_bucket_full==true)
  {
    first_run_lora_send=false;
    if (lmh_join_status_get()==LMH_SET)
    {
      Serial.println("Sending data to LoRa");  //Copy in data to send into transmist buffer and call function
      uint8_t transmit_data[NUM_SOIL_SENSORS];
      for(int z=0;z<NUM_SOIL_SENSORS;z++) transmit_data[z]=sensor[z].percentage;       
      send_lora_frame(transmit_data,sizeof(transmit_data));
    }
    else Serial.println("No LoRa network access");
    
    time_lora_send_start=millis();
  }
}

void set_soil_sensor(int num)
{
 if (num <0 || num>(NUM_SOIL_SENSORS-1) || num >(MAX_SOIL_SENSORS-1))  //Verify soil sensor number is valid
 {
   Serial.println("ERROR set soil sensor out of range");  Serial.flush();
   num=0;
 }

  if (B001 & num) digitalWrite(WB_IO1,HIGH);
    else digitalWrite(WB_IO1,LOW);
  if (B010 & num) digitalWrite(WB_IO3,HIGH);
    else digitalWrite(WB_IO3,LOW);
  if (B100 & num) digitalWrite(WB_IO4,HIGH);
    else digitalWrite(WB_IO4,LOW);

  delay(2);  //Make sure MUX has switched before returning, probably not needed
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  Serial.println("LoRa Network joined");
  lora_ready=true;
  digitalWrite(LED_BUILTIN,HIGH); //Turn on built in green LED
}

/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("ERROR OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)  //Part of RAK 4631 LoRa example, should not happen with Helium/LoRa
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  
  //Informs the server that switch has occurred ASAP    //Took this out since it did not apply to Helium
  //m_lora_app_data.buffsize = 0;
  //m_lora_app_data.port = gAppPort;
  //lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(uint8_t data[], int size)
{
  if (lmh_join_status_get() != LMH_SET)  //Should not make it this far if no LoRa
  {
    Serial.println("ERROR no LoRa access");
    return;
  }
  
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);    //Zero out buffer for safety, probably not needed
  memcpy(m_lora_app_data.buffer,data,size);                         //Copy data into lora message buffer
  m_lora_app_data.port = gAppPort;                                  //Setting LoRa port
  m_lora_app_data.buffsize=size;                                    //Set data length of buffer

  Serial.print("Size of payload "); Serial.println(m_lora_app_data.buffsize);

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS) { count++; last_successful_lora_send_time=millis(); }
    else count_fail++;
  Serial.printf("LoRa send successfull count %d failed count %d",count,count_fail);
  Serial.println("");
}