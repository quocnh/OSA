





//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//   Heartrate and respiration computation based on original code from Texas Instruments
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install

// If you have bought the breakout the connection with the Arduino board is as follows:
//
//|ads1292r pin label| Arduino Connection   |Pin Function      |
//|----------------- |:--------------------:|-----------------:|
//| VDD              | +5V                  |  Supply voltage  |
//| PWDN/RESET       | D4                   |  Reset           |
//| START            | D5                   |  Start Input     |
//| DRDY             | D6                   |  Data Ready Outpt|
//| CS               | D7                   |  Chip Select     |
//| MOSI             | D11                  |  Slave In        |
//| MISO             | D12                  |  Slave Out       |
//| SCK              | D13                  |  Serial Clock    |
//| GND              | Gnd                  |  Gnd             |
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <ads1292r.h>
#include <string.h>
#include <SPI.h>
#include <math.h>
#include <stdio.h>

#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <AutoConnect.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ThingsBoard.h>

ads1292r ADS1292;   // define class

//Autoconnect
static const char AUX_TIMEZONE[] PROGMEM = R"(
{
  "title": "TimeZone",
  "uri": "/timezone",
  "menu": true,
  "element": [
    {
      "name": "caption",
      "type": "ACText",
      "value": "Sets the time zone to get the current local time.",
      "style": "font-family:Arial;font-weight:bold;text-align:center;margin-bottom:10px;color:DarkSlateBlue"
    },
    {
      "name": "timezone",
      "type": "ACSelect",
      "label": "Select TZ name",
      "option": [],
      "selected": 10
    },
    {
      "name": "newline",
      "type": "ACElement",
      "value": "<br>"
    },
    {
      "name": "start",
      "type": "ACSubmit",
      "value": "OK",
      "uri": "/start"
    }
  ]
}
)";

typedef struct {
  const char* zone;
  const char* ntpServer;
  int8_t      tzoff;
} Timezone_t;

static const Timezone_t TZ[] = {
  { "Europe/London", "europe.pool.ntp.org", 0 },
  { "Europe/Berlin", "europe.pool.ntp.org", 1 },
  { "Europe/Helsinki", "europe.pool.ntp.org", 2 },
  { "Europe/Moscow", "europe.pool.ntp.org", 3 },
  { "Asia/Dubai", "asia.pool.ntp.org", 4 },
  { "Asia/Karachi", "asia.pool.ntp.org", 5 },
  { "Asia/Dhaka", "asia.pool.ntp.org", 6 },
  { "Asia/Jakarta", "asia.pool.ntp.org", 7 },
  { "Asia/Manila", "asia.pool.ntp.org", 8 },
  { "Asia/Tokyo", "asia.pool.ntp.org", 9 },
  { "Australia/Brisbane", "oceania.pool.ntp.org", 10 },
  { "Pacific/Noumea", "oceania.pool.ntp.org", 11 },
  { "Pacific/Auckland", "oceania.pool.ntp.org", 12 },
  { "Atlantic/Azores", "europe.pool.ntp.org", -1 },
  { "America/Noronha", "south-america.pool.ntp.org", -2 },
  { "America/Araguaina", "south-america.pool.ntp.org", -3 },
  { "America/Blanc-Sablon", "north-america.pool.ntp.org", -4},
  { "America/New_York", "north-america.pool.ntp.org", -5 },
  { "America/Chicago", "north-america.pool.ntp.org", -6 },
  { "America/Denver", "north-america.pool.ntp.org", -7 },
  { "America/Los_Angeles", "north-america.pool.ntp.org", -8 },
  { "America/Anchorage", "north-america.pool.ntp.org", -9 },
  { "Pacific/Honolulu", "north-america.pool.ntp.org", -10 },
  { "Pacific/Samoa", "oceania.pool.ntp.org", -11 }
};

WebServer Server;
AutoConnect       Portal(Server);
AutoConnectConfig Config;       // Enable autoReconnect supported on v0.9.4
AutoConnectAux    Timezone;

//wifi and device on Thingsboard
//#define WIFI_AP "tollieollie"
//#define WIFI_PASSWORD "Jaketom5"
//#define TOKEN "A1_TEST_TOKEN"    //Dev board
#define TOKEN "ecg_spaches"      //Prototype
char thingsboardServer[] = "134.129.122.213";
WiFiClient wifiClient;
ThingsBoard tb(wifiClient);
int status = WL_IDLE_STATUS;
unsigned long lastSend;

//NTP library to get real time from server
#define NTP_OFFSET   0 * 60      // In seconds
#define NTP_INTERVAL 5 * 1000    // In miliseconds
#define NTP_ADDRESS  "pool.ntp.org"
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);


//timestamp variables
unsigned long long timestamp;
unsigned long long real_time;
unsigned long long previous_ts;
int ii = 0;

//LED Time Variables
unsigned long current_time_LED;
unsigned long previous_time_LED;
unsigned long elapsed_time_LED = 0;
float voltage = 0.0f;
float percentage = 0.0f;
int battStatus = 5; //Start in default mode (not transmitting)
int ledState = LOW;   // ledState used to set the LED

//Packet format
#define  CES_CMDIF_PKT_START_1      0x0A
#define  CES_CMDIF_PKT_START_2      0xFA
#define  CES_CMDIF_TYPE_DATA        0x02
#define  CES_CMDIF_PKT_STOP_1       0x00
#define  CES_CMDIF_PKT_STOP_2       0x0B

//pins
#define ADS1292_DRDY_PIN 4
#define ADS1292_CS_PIN 15
#define ADS1292_START_PIN 0
#define ADS1292_PWDN_PIN 2
#define GRN_LED 27          //TBD after soldering
#define RED_LED 26          //TBD after soldering
#define BATTERY_IN 39
#define CHARGER 18
#define TEMPERATURE 0
#define FILTERORDER         161
/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)
#define WAVE_SIZE  1

//******* ecg filter *********
#define MAX_PEAK_TO_SEARCH         5
#define MAXIMA_SEARCH_WINDOW      25
#define MINIMUM_SKIP_WINDOW       30
#define SAMPLING_RATE             250
#define TWO_SEC_SAMPLES           2 * SAMPLING_RATE
#define QRS_THRESHOLD_FRACTION    0.4
#define TRUE 1
#define FALSE 0

volatile uint8_t  SPI_Dummy_Buff[30];
uint8_t DataPacketHeader[16];
volatile signed long s32DaqVals[8];
uint8_t data_len = 7;
volatile byte SPI_RX_Buff[15] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile bool ads1292dataReceived = false;
unsigned long uecgtemp = 0;
signed long secgtemp = 0;
int i, j;

//************** ecg *******************
int16_t CoeffBuf_40Hz_LowPass[FILTERORDER] =
{
  -72,    122,    -31,    -99,    117,      0,   -121,    105,     34,
  -137,     84,     70,   -146,     55,    104,   -147,     20,    135,
  -137,    -21,    160,   -117,    -64,    177,    -87,   -108,    185,
  -48,   -151,    181,      0,   -188,    164,     54,   -218,    134,
  112,   -238,     90,    171,   -244,     33,    229,   -235,    -36,
  280,   -208,   -115,    322,   -161,   -203,    350,    -92,   -296,
  361,      0,   -391,    348,    117,   -486,    305,    264,   -577,
  225,    445,   -660,     93,    676,   -733,   -119,    991,   -793,
  -480,   1486,   -837,  -1226,   2561,   -865,  -4018,   9438,  20972,
  9438,  -4018,   -865,   2561,  -1226,   -837,   1486,   -480,   -793,
  991,   -119,   -733,    676,     93,   -660,    445,    225,   -577,
  264,    305,   -486,    117,    348,   -391,      0,    361,   -296,
  -92,    350,   -203,   -161,    322,   -115,   -208,    280,    -36,
  -235,    229,     33,   -244,    171,     90,   -238,    112,    134,
  -218,     54,    164,   -188,      0,    181,   -151,    -48,    185,
  -108,    -87,    177,    -64,   -117,    160,    -21,   -137,    135,
  20,   -147,    104,     55,   -146,     70,     84,   -137,     34,
  105,   -121,      0,    117,    -99,    -31,    122,    -72
};
int16_t ECG_WorkingBuff[2 * FILTERORDER];
unsigned char Start_Sample_Count_Flag = 0;
unsigned char first_peak_detect = FALSE ;
unsigned int sample_index[MAX_PEAK_TO_SEARCH + 2] ;
uint16_t scaled_result_display[150];
uint8_t indx = 0;

int cnt = 0;
volatile uint8_t flag = 0;

int QRS_Second_Prev_Sample = 0 ;
int QRS_Prev_Sample = 0 ;
int QRS_Current_Sample = 0 ;
int QRS_Next_Sample = 0 ;
int QRS_Second_Next_Sample = 0 ;

static uint16_t QRS_B4_Buffer_ptr = 0 ;
/*   Variable which holds the threshold value to calculate the maxima */
int16_t QRS_Threshold_Old = 0;
int16_t QRS_Threshold_New = 0;
unsigned int sample_count = 0 ;

/* Variable which will hold the calculated heart rate */
volatile uint16_t QRS_Heart_Rate = 0 ;
int16_t ecg_wave_buff[1], ecg_filterout[1];

volatile uint8_t global_HeartRate = 0;
volatile uint8_t global_RespirationRate = 0;
long status_byte=0;
uint8_t LeadStatus=0;
boolean leadoff_deteted = true;
long time_elapsed=0;

void rootPage() {
  String  content =
    "<html>"
    "<head>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<script type=\"text/javascript\">"
    "setTimeout(\"location.reload()\", 1000);"
    "</script>"
    "</head>"
    "<body>"
    "<h2 align=\"center\" style=\"color:blue;margin:20px;\">Hello, world</h2>"
    "<h3 align=\"center\" style=\"color:gray;margin:10px;\">{{DateTime}}</h3>"
    "<p style=\"text-align:center;\">Reload the page to update the time.</p>"
    "<p></p><p style=\"padding-top:15px;text-align:center\">" AUTOCONNECT_LINK(COG_24) "</p>"
    "</body>"
    "</html>";
  static const char *wd[7] = { "Sun","Mon","Tue","Wed","Thr","Fri","Sat" };
  struct tm *tm;
  time_t  t;
  char    dateTime[26];

  t = time(NULL);
  tm = localtime(&t);
  sprintf(dateTime, "%04d/%02d/%02d(%s) %02d:%02d:%02d.",
    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
    wd[tm->tm_wday],
    tm->tm_hour, tm->tm_min, tm->tm_sec);
  content.replace("{{DateTime}}", String(dateTime));
  Server.send(200, "text/html", content);
}

void startPage() {
  // Retrieve the value of AutoConnectElement with arg function of WebServer class.
  // Values are accessible with the element name.
  String  tz = Server.arg("timezone");

  for (uint8_t n = 0; n < sizeof(TZ) / sizeof(Timezone_t); n++) {
    String  tzName = String(TZ[n].zone);
    if (tz.equalsIgnoreCase(tzName)) {
      configTime(TZ[n].tzoff * 3600, 0, TZ[n].ntpServer);
      Serial.println("Time zone: " + tz);
      Serial.println("ntp server: " + String(TZ[n].ntpServer));
      break;
    }
  }

  // The /start page just constitutes timezone,
  // it redirects to the root page without the content response.
  Server.sendHeader("Location", String("http://") + Server.client().localIP().toString() + String("/"));
  Server.send(302, "text/plain", "");
  Server.client().flush();
  Server.client().stop();
}

void setup()
{
  delay(2000);
  // initalize the  data ready and chip select pins:
  pinMode(ADS1292_DRDY_PIN, INPUT);  //6
  pinMode(ADS1292_CS_PIN, OUTPUT);    //7
  pinMode(ADS1292_START_PIN, OUTPUT);  //5
  pinMode(ADS1292_PWDN_PIN, OUTPUT);  //4
  
  //LED and battery read pins
  pinMode(GRN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(CHARGER,OUTPUT);
  pinMode(BATTERY_IN, INPUT);
  digitalWrite(RED_LED, HIGH);

  analogReadResolution(12);
  Serial.begin(115200);  // Baudrate for serial communica
  delay(50);
  
   //set up wifi
  //InitWiFi();
  
  // Enable saved past credential by autoReconnect option,
  // even once it is disconnected.
  Config.apid = "ECGap";
  Config.apip =  IPAddress(192,168,10,102);
  Config.autoReconnect = false;
  Config.retainPortal = true;
  Config.autoRise = true;
  //Config.preserveAPMode = true;
  Config.immediateStart = true;
  Config.hostName = "esp32-02";
  Portal.config(Config);
//   Portal.config(Config);

  // Load aux. page
  Timezone.load(AUX_TIMEZONE);
  // Retrieve the select element that holds the time zone code and
  // register the zone mnemonic in advance.
  AutoConnectSelect&  tz = Timezone["timezone"].as<AutoConnectSelect>();
  for (uint8_t n = 0; n < sizeof(TZ) / sizeof(Timezone_t); n++) {
    tz.add(String(TZ[n].zone));
  }

  Portal.join({ Timezone });        // Register aux. page

  // Behavior a root path of ESP8266WebServer.
  Server.on("/", rootPage);
  Server.on("/start", startPage);   // Set NTP server trigger handler

  Serial.println("Creating portal and trying to connect...");
  // Establish a connection with an autoReconnect option.
  if (Portal.begin()) {
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
    Serial.println(WiFi.getHostname());
  }

  //set up timestamp
  timeClient.begin();
  timeClient.update();
  timestamp = timeClient.getEpochTime();

  //set up for data saving
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Date,Time,Timestamp,ECG, ECG_Filtered,Respiration");
  Serial.println("RESETTIMER");

  ADS1292.ads1292_Init();  //initalize ADS1292 slave
  ADS1292.ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x01); //change the sampling rate to 250 SPS
  Serial.println("Initiliziation is done");
}

void loop()
{ 
  Portal.handleClient();
  
  timeClient.update();

  //voltage read
  voltage = ReadVoltage(BATTERY_IN);//ADC to voltage conversion
  percentage = 2808.3808*pow(voltage,4)-43560.9157*pow(voltage,3)+252848.5888*pow(voltage,2)-650767.4615*voltage+626532.5703; //curve fit of LiPo
  if(voltage > 4.19) percentage = 100; //upper limit
  if(voltage < 3.5) percentage = 0; //Lower limit

  //Charge logic
  if(voltage >= 4.1) digitalWrite(CHARGER, LOW);
  if(voltage < 3.9) digitalWrite(CHARGER,HIGH);
  
  if(percentage < 33){
    battStatus = 0;
  }else if(percentage < 66){
    battStatus = 1;
  }else{
    battStatus = 2;
  }

  current_time_LED = millis();                              //get current time for LED function
  elapsed_time_LED = current_time_LED - previous_time_LED;  //calculate elapsed time for LED function
  LEDFunction(battStatus);
  
  if ( !tb.connected() ) {
    reconnect();
  }
  if ((digitalRead(ADS1292_DRDY_PIN)) == LOW)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  {
    SPI_RX_Buff_Ptr = ADS1292.ads1292_Read_Data(); // Read the data,point the data to a pointer

    for (i = 0; i < 9; i++)
    {
      SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
    }
    ads1292dataReceived = true;
  }


  if (ads1292dataReceived == true)      // process the data
  {
    j = 0;
    for (i = 3; i < 9; i += 3)         // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
    {

      uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff[i + 0] << 16) | ( (unsigned long) SPI_RX_Buff[i + 1] << 8) |  (unsigned long) SPI_RX_Buff[i + 2]);
      uecgtemp = (unsigned long) (uecgtemp << 8);
      secgtemp = (signed long) (uecgtemp);
      secgtemp = (signed long) (secgtemp >> 8);

      s32DaqVals[j++] = secgtemp;  //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
//      Serial.print("Resp:");
//      Serial.println(s32DaqVals[0]);
//    Serial.print("ECG:");
//      Serial.println(s32DaqVals[1]);
    }

    status_byte = (long)((long)SPI_RX_Buff[2] | ((long) SPI_RX_Buff[1]) <<8 | ((long) SPI_RX_Buff[0])<<16); // First 3 bytes represents the status
    status_byte  = (status_byte & 0x0f8000) >> 15;  // bit15 gives the lead status
    LeadStatus = (unsigned char ) status_byte ;  

    if(!((LeadStatus & 0x1f) == 0 ))
      leadoff_deteted  = true; 
    else
      leadoff_deteted  = false;
    
    ecg_wave_buff[0] = (int16_t)(s32DaqVals[1] >> 8) ;  // ignore the lower 8 bits out of 24bits

    if(leadoff_deteted == false) 
       {
          ECG_ProcessCurrSample(&ecg_wave_buff[0], &ecg_filterout[0]);   // filter out the line noise @40Hz cutoff 161 order
          //Serial.print("ECG filtered: ");
          //Serial.println(ecg_filterout[0]);
  
          //send data to Thingsboard
          real_time = timeClient.getEpochTime();
          getAndSendECG(real_time);
          
          QRS_Algorithm_Interface(ecg_filterout[0]);             // calculate 
       }
    else
       ecg_filterout[0] = 0;

    if(millis() > time_elapsed)  // update every one second
    {
//      if(leadoff_deteted == true) // lead in not connected
//      Serial.println("ECG lead error!!! ensure the leads are properly connected");
//      else
//      {
//        Serial.print("Heart rate: ");
//        Serial.print(global_HeartRate);
//        Serial.println("BPM");
//      }
      tb.sendTelemetryFloat("Heart rate", global_HeartRate);
      time_elapsed += 1000;
    }
    
  }

  ads1292dataReceived = false;
  SPI_RX_Buff_Count = 0;
  
  tb.loop();

}


void getAndSendECG(unsigned long long real_time)
{
  // Prepare a JSON payload string
  char ts[13];
  if (real_time * 1000 > previous_ts)
    timestamp = real_time * 1000;
  else timestamp = previous_ts + 4; //add 4 ms for each data point because sampling rate is 250 SPS
  previous_ts = timestamp;
  sprintf(ts, "%llu", timestamp);
  String payload = "{";
//  payload += "\"ts\":"; payload += ts; payload += ",";
//  payload += "\"values\":"; payload += "{";

  payload += "\"ECG\":"; payload += s32DaqVals[1]; payload += ",";
  payload += "\"ECG_Filtered\":"; payload += ecg_filterout[0]; payload += ",";
  payload += "\"Respiration\":"; payload += s32DaqVals[0]; 
  payload += "}";
//  payload += "}";

  char PPG_data[1000];
  payload.toCharArray(PPG_data, 1000 );
  tb.sendTelemetryJson(PPG_data);

  //save data for PLX-DAQ serial monitor
     // Serial.print("DATA,DATE,TIME,");
      //Serial.print(ts);
      //Serial.print(",");
      //Serial.print(s32DaqVals[1]);
      //Serial.print(",");
      //Serial.print(ecg_filterout[0]);
      //Serial.println(",");
      //Serial.print(s32DaqVals[0]);
      //Serial.println(",");
}

/*
void InitWiFi()
{
//  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }
//  Serial.println("Connected to AP");
}
*/

void reconnect() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    /*
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
//        Serial.print(".");
      }
//      Serial.println("Connected to AP");
    }*/
//    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
//      Serial.println( "[DONE]" );
    } else {
     Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}



void ECG_FilterProcess(int16_t * WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut)
{

  int32_t acc = 0;   // accumulator for MACs
  int  k;

  // perform the multiply-accumulate
  for ( k = 0; k < 161; k++ )
  {
    acc += (int32_t)(*CoeffBuf++) * (int32_t)(*WorkingBuff--);
  }
  // saturate the result
  if ( acc > 0x3fffffff )
  {
    acc = 0x3fffffff;
  } else if ( acc < -0x40000000 )
  {
    acc = -0x40000000;
  }
  // convert from Q30 to Q15
  *FilterOut = (int16_t)(acc >> 15);
  //*FilterOut = *WorkingBuff;
}




void ECG_ProcessCurrSample(int16_t *CurrAqsSample, int16_t *FilteredOut)
{
  static uint16_t ECG_bufStart = 0, ECG_bufCur = FILTERORDER - 1, ECGFirstFlag = 1;
  static int16_t ECG_Pvev_DC_Sample, ECG_Pvev_Sample;/* Working Buffer Used for Filtering*/
  //  static short ECG_WorkingBuff[2 * FILTERORDER];
  int16_t *CoeffBuf;
  int16_t temp1, temp2, ECGData;

  /* Count variable*/
  uint16_t Cur_Chan;
  int16_t FiltOut = 0;
  //  short FilterOut[2];
  CoeffBuf = CoeffBuf_40Hz_LowPass;         // Default filter option is 40Hz LowPass

  if  ( ECGFirstFlag )                // First Time initialize static variables.
  {
    for ( Cur_Chan = 0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
    {
      ECG_WorkingBuff[Cur_Chan] = 0;
    }
    ECG_Pvev_DC_Sample = 0;
    ECG_Pvev_Sample = 0;
    ECGFirstFlag = 0;
  }

  temp1 = NRCOEFF * ECG_Pvev_DC_Sample;       //First order IIR
  ECG_Pvev_DC_Sample = (CurrAqsSample[0]  - ECG_Pvev_Sample) + temp1;
  ECG_Pvev_Sample = CurrAqsSample[0];
  temp2 = ECG_Pvev_DC_Sample >> 2;
  ECGData = (int16_t) temp2;

  /* Store the DC removed value in Working buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufCur] = ECGData;
  ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur], CoeffBuf, (int16_t*)&FiltOut);
  /* Store the DC removed value in ECG_WorkingBuff buffer in millivolts range*/
  ECG_WorkingBuff[ECG_bufStart] = ECGData;

  /* Store the filtered out sample to the LeadInfo buffer*/
  FilteredOut[0] = FiltOut ;//(CurrOut);

  ECG_bufCur++;
  ECG_bufStart++;

  if ( ECG_bufStart  == (FILTERORDER - 1))
  {
    ECG_bufStart = 0;
    ECG_bufCur = FILTERORDER - 1;
  }
  return ;
}


void QRS_Algorithm_Interface(int16_t CurrSample)
{
  //  static FILE *fp = fopen("ecgData.txt", "w");
  static int16_t prev_data[32] = {0};
  int16_t i;
  long Mac = 0;
  prev_data[0] = CurrSample;

  for ( i = 31; i > 0; i--)
  {
    Mac += prev_data[i];
    prev_data[i] = prev_data[i - 1];

  }
  Mac += CurrSample;
  Mac = Mac >> 2;
  CurrSample = (int16_t) Mac;
  QRS_Second_Prev_Sample = QRS_Prev_Sample ;
  QRS_Prev_Sample = QRS_Current_Sample ;
  QRS_Current_Sample = QRS_Next_Sample ;
  QRS_Next_Sample = QRS_Second_Next_Sample ;
  QRS_Second_Next_Sample = CurrSample ;


  QRS_process_buffer();
}

static void QRS_process_buffer( void )
{

  int16_t first_derivative = 0 ;
  int16_t scaled_result = 0 ;

  static int16_t Max = 0 ;

  /* calculating first derivative*/
  first_derivative = QRS_Next_Sample - QRS_Prev_Sample  ;



  /*taking the absolute value*/

  if (first_derivative < 0)
  {
    first_derivative = -(first_derivative);
  }


  scaled_result = first_derivative;

  if ( scaled_result > Max )
  {
    Max = scaled_result ;
  }


  QRS_B4_Buffer_ptr++;
  if (QRS_B4_Buffer_ptr ==  TWO_SEC_SAMPLES)
  {
    QRS_Threshold_Old = ((Max * 7) / 10 ) ;
    QRS_Threshold_New = QRS_Threshold_Old ;
   // if (Max > 70)
      first_peak_detect = TRUE ;
    Max = 0;
 
    //  ecg_wave_buff[0] = first_derivative;
    QRS_B4_Buffer_ptr = 0;


  }


  if ( TRUE == first_peak_detect )
  {
    QRS_check_sample_crossing_threshold( scaled_result ) ;
  }


}


static void QRS_check_sample_crossing_threshold( uint16_t scaled_result )
{
  /* array to hold the sample indexes S1,S2,S3 etc */

  static uint16_t s_array_index = 0 ;
  static uint16_t m_array_index = 0 ;

  static unsigned char threshold_crossed = FALSE ;
  static uint16_t maxima_search = 0 ;
  static unsigned char peak_detected = FALSE ;
  static uint16_t skip_window = 0 ;
  static long maxima_sum = 0 ;
  static unsigned int peak = 0;
  static unsigned int sample_sum = 0;
  static unsigned int nopeak = 0;
  uint16_t Max = 0 ;
  uint16_t HRAvg;
  uint16_t  RRinterval = 0;

  if ( TRUE == threshold_crossed  )
  {
    /*
    Once the sample value crosses the threshold check for the
    maxima value till MAXIMA_SEARCH_WINDOW samples are received
    */
    sample_count ++ ;
    maxima_search ++ ;

    if ( scaled_result > peak )
    {
      peak = scaled_result ;
    }

    if ( maxima_search >= MAXIMA_SEARCH_WINDOW )
    {
      // Store the maxima values for each peak
      maxima_sum += peak ;
      maxima_search = 0 ;


      threshold_crossed = FALSE ;
      peak_detected = TRUE ;
    }

  }
  else if ( TRUE == peak_detected )
  {
    /*
    Once the sample value goes below the threshold
    skip the samples untill the SKIP WINDOW criteria is meet
    */
    sample_count ++ ;
    skip_window ++ ;

    if ( skip_window >= MINIMUM_SKIP_WINDOW )
    {
      skip_window = 0 ;
      peak_detected = FALSE ;
    }

    if ( m_array_index == MAX_PEAK_TO_SEARCH )
    {
      sample_sum = sample_sum / (MAX_PEAK_TO_SEARCH - 1);
      HRAvg =  (uint16_t) sample_sum  ;

      // Compute HR without checking LeadOffStatus
      QRS_Heart_Rate = (uint16_t) 60 *  SAMPLING_RATE;
      QRS_Heart_Rate =  QRS_Heart_Rate / HRAvg ;
      if (QRS_Heart_Rate > 250)
        QRS_Heart_Rate = 250 ;
 //     Serial.println(QRS_Heart_Rate);

      /* Setting the Current HR value in the ECG_Info structure*/
      maxima_sum =  maxima_sum / MAX_PEAK_TO_SEARCH;
      Max = (int16_t) maxima_sum ;
      /*  calculating the new QRS_Threshold based on the maxima obtained in 4 peaks */
      maxima_sum = Max * 7;
      maxima_sum = maxima_sum / 10;
      QRS_Threshold_New = (int16_t)maxima_sum;

      /* Limiting the QRS Threshold to be in the permissible range*/
      if (QRS_Threshold_New > (4 * QRS_Threshold_Old))
      {
        QRS_Threshold_New = QRS_Threshold_Old;
      }

      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;

      sample_sum = 0;
    }
  }
  else if ( scaled_result > QRS_Threshold_New )
  {
    /*
      If the sample value crosses the threshold then store the sample index
    */
    Start_Sample_Count_Flag = 1;
    sample_count ++ ;
    m_array_index++;
    threshold_crossed = TRUE ;
    peak = scaled_result ;
    nopeak = 0;

    /*  storing sample index*/
    sample_index[ s_array_index ] = sample_count ;
    if ( s_array_index >= 1 )
    {
      sample_sum += sample_index[ s_array_index ] - sample_index[ s_array_index - 1 ] ;
    }
    s_array_index ++ ;
  }

  else if (( scaled_result < QRS_Threshold_New ) && (Start_Sample_Count_Flag == 1))
  {
    sample_count ++ ;
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
    {
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = FALSE ;
      sample_sum = 0;

      first_peak_detect = FALSE;
      nopeak = 0;

      QRS_Heart_Rate = 0;

    }
  }
  else
  {
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
    {
      /* Reset heart rate computation sate variable in case of no peak found in 3 seconds */
      sample_count = 0 ;
      s_array_index = 0 ;
      m_array_index = 0 ;
      maxima_sum = 0 ;
      sample_index[0] = 0 ;
      sample_index[1] = 0 ;
      sample_index[2] = 0 ;
      sample_index[3] = 0 ;
      Start_Sample_Count_Flag = 0;
      peak_detected = FALSE ;
      sample_sum = 0;
      first_peak_detect = FALSE;
      nopeak = 0;
      QRS_Heart_Rate = 0;
    }
  }

  global_HeartRate = (uint8_t)QRS_Heart_Rate;
  //Serial.println(global_HeartRate);

}

double ReadVoltage(uint8_t pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  //return (-0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089)*2; //original fir from creator
  return (-.0000000000000096795072211912461* pow(reading,4) + .000000000064564581092594387 * pow(reading,3) - .00000014328287130333392 * pow(reading,2)+ .00090565621090209041 * reading + .11253959753849530)*2;
} 

void LEDFunction (int battStatus){
  switch(battStatus){
    case 0: //Battery criticially low (less than 33%)
    {
      if(elapsed_time_LED > 1000){
        // if the LED is off turn it on and vice-versa:
        ledState = (ledState == LOW) ? HIGH : LOW;

        // set the LED with the ledState of the variable:
        digitalWrite(GRN_LED, LOW);
        digitalWrite(RED_LED, ledState);
        previous_time_LED = millis();
      }
      break;
    }
    case 1: //Battery < 66%
    {
      if(elapsed_time_LED > 1000){
        // if the LED is off turn it on and vice-versa:
        ledState = (ledState == LOW) ? HIGH : LOW;

        // set the LED with the ledState of the variable:
        digitalWrite(GRN_LED, ledState);
        digitalWrite(RED_LED, ledState);
        previous_time_LED = millis();
      }
      break;
    }
    case 2: //Battery > 66%
    {
      if(elapsed_time_LED > 2000){
        // if the LED is off turn it on and vice-versa:
        ledState = (ledState == LOW) ? HIGH : LOW;

        // set the LED with the ledState of the variable:
        digitalWrite(GRN_LED, ledState);
        digitalWrite(RED_LED, LOW);
        previous_time_LED = millis();
      }
      break;
    }
    default:
      break;
    }
  }
