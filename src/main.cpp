/* A program to receive data from a RYLR998 LoRa receiver and publish it to MQTT.
 * 
 * Configuration is done via serial connection.  Enter:
 *  broker=<broker name or address>
 *  port=<port number>   (defaults to 1883)
 *  topicroot=<topic root> (something like buteomont/gate/package/ - must end with / and 
 *  "present", "distance", "analog", or "voltage" will be added)
 *  user=<mqtt user>
 *  pass=<mqtt password>
 *  ssid=<wifi ssid>
 *  wifipass=<wifi password>
 *  loRaAddress=<LoRa address>
 *  loRaNetworkID=<must be the same for transmitter and receiver>
 *  loRaBand=<Frequency>
 *  loRaSpreadingFactor=<LoRa spreading factor
 *  loRaBandwidth=<bandwidth code>
 *  loRaCodingRate=<LoRa coding rate>
 *  loRaPreamble=<LoRa preamble
 *  loRaBaudRate=<LoRa baud rate for both RF and serial comms
 * 
 * Once connected to an MQTT broker, configuration can be done similarly via the 
 * <topicroot>/command topic. 
 *
 *
  */

#include <Arduino.h>
#include <math.h>    
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "pressureReporter.h"

#define VERSION "25.05.17.0"  //remember to update this after every change! YY.MM.DD.REV

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed

// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char mqttBrokerAddress[ADDRESS_SIZE]=""; //default
  int mqttBrokerPort=1883;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_TOPIC_SIZE]="";
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  bool debug=false;
  char address[ADDRESS_SIZE]=""; //static address for this device
  char netmask[ADDRESS_SIZE]=""; //size of network
  bool invertdisplay=false;   //rotate display 180 degrees
  int measureInterval=DEFAULT_MEASURE_INTERVAL; //How long to wait between measurements
  } conf;
conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

IPAddress ip;
IPAddress mask;

typedef struct
  {
  uint16_t distance;
  bool isPresent;
  float_t battery;
  int8_t rssi;
  int8_t snr;
  uint8_t address;
  } box;
box boxStatus; 


boolean rssiShowing=false; //used to redraw the RSSI indicator after clearing display
String lastMessage=""; //contains the last message sent to display. Sometimes need to reshow it
ulong activityLedTimeoff=millis();

void drawWifiStrength(int32_t rssi)
  {
  int strength = map(rssi, -100, -50, 0, 4);
  static int xLoc=SCREEN_WIDTH-RSSI_DOT_RADIUS;
  static int yLoc=SCREEN_HEIGHT-RSSI_DOT_RADIUS;
  
  // Draw the dot
  display.fillCircle(xLoc, yLoc, RSSI_DOT_RADIUS, SSD1306_WHITE);
  
  // Draw the arcs
  for (int i = 0; i < 4; i++) 
    {
    if (i < strength) 
      {
      display.drawCircle(xLoc, yLoc, RSSI_DOT_RADIUS + (i * 5), SSD1306_WHITE);
      display.drawCircle(xLoc, yLoc, RSSI_DOT_RADIUS+1 + (i * 5), SSD1306_WHITE);
      display.drawCircle(xLoc, yLoc, RSSI_DOT_RADIUS+2 + (i * 5), SSD1306_WHITE);
     }
    else 
      {
      display.drawCircle(xLoc, yLoc, RSSI_DOT_RADIUS + (i * 5), SSD1306_WHITE);
      display.drawCircle(xLoc, yLoc, RSSI_DOT_RADIUS+1 + (i * 5), SSD1306_BLACK);
      display.drawCircle(xLoc, yLoc, RSSI_DOT_RADIUS+2 + (i * 5), SSD1306_BLACK);
      } 
    }
  rssiShowing=true; //keep it up
//  display.display();
  }


//display something on the screen
void show(String msg)
  {
  if (msg==lastMessage)
    return;
    
  lastMessage=msg; //in case we need to redraw it

  if (settings.debug)
    {
    Serial.print("Length of display message:");
    Serial.println(msg.length());
    }
  display.clearDisplay(); // clear the screen
  display.setCursor(0, 0);  // Top-left corner

  if (msg.length()>20)
    {
    display.setTextSize(1);      // tiny text
    }
  else if (msg.length()>7 || rssiShowing) //make room for rssi indicator
    {
    display.setTextSize(2);      // small text
    }
  else
    {
    display.setTextSize(3);      // Normal 1:1 pixel scale
    }
  display.println(msg);
  if (rssiShowing && msg.length()>0)
    {
    drawWifiStrength(WiFi.RSSI());
    }
display.display(); // move the buffer contents to the OLED
  }


void show(uint16_t val, String suffix)
  {
  String msg=String(val)+suffix;
  show(msg);
  }




void showSettings()
  {
  Serial.print("broker=<MQTT broker host name or address> (");
  Serial.print(settings.mqttBrokerAddress);
  Serial.println(")");
  Serial.print("port=<port number>   (");
  Serial.print(settings.mqttBrokerPort);
  Serial.println(")");
  Serial.print("topicroot=<topic root> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")  Note: must end with \"/\"");  
  Serial.print("user=<mqtt user> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("pass=<mqtt password> (");
  Serial.print(settings.mqttPassword);
  Serial.println(")");
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("address=<Static IP address if so desired> (");
  Serial.print(settings.address);
  Serial.println(")");
  Serial.print("netmask=<Network mask to be used with static IP> (");
  Serial.print(settings.netmask);
  Serial.println(")");
  Serial.print("debug=1|0 (");
  Serial.print(settings.debug);
  Serial.println(")");
  Serial.print("invertdisplay=1|0 (");
  Serial.print(settings.invertdisplay);
  Serial.println(")");
  Serial.print("measureinterval=<seconds>   (");
  Serial.print(settings.measureInterval);
  Serial.println(")");
 
  Serial.print("MQTT Client ID is ");
  Serial.println(settings.mqttClientId);
  Serial.print("Address is ");
  Serial.println(wifiClient.localIP());
  Serial.println("\n*** Use NULL to reset a setting to its default value ***");
  Serial.println("*** Use \"factorydefaults=yes\" to reset all settings  ***\n");
  
  Serial.print("\nSettings are ");
  Serial.println(settingsAreValid?"valid.":"incomplete.");
  }

  
/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;
    if (newCommand.length()==0)
      newCommand='\n'; //to show available commands

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

bool processCommand(String cmd)
  {
  bool commandFound=true; //saves a lot of code
  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme=strtok((char *)str,"=");
  if (nme!=NULL)
    val=strtok(NULL,"=");

  if (nme[0]=='\n' || nme[0]=='\r' || nme[0]=='\0') //a single cr means show current settings
    {
    showSettings();
    commandFound=false; //command not found
    }
  else
    {
    //Get rid of the carriage return
    if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
      val[strlen(val)-1]=0; 

    if (val!=NULL)
      {
      if (strcmp(val,"NULL")==0) //to nullify a value, you have to really mean it
        {
        strcpy(val,"");
        }
      
      if (strcmp(nme,"broker")==0)
        {
        strcpy(settings.mqttBrokerAddress,val);
        saveSettings();
        }
      else if (strcmp(nme,"port")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.mqttBrokerPort=atoi(val);
        saveSettings();
        }
      else if (strcmp(nme,"topicroot")==0)
        {
        strcpy(settings.mqttTopicRoot,val);
        saveSettings();
        }
      else if (strcmp(nme,"user")==0)
        {
        strcpy(settings.mqttUsername,val);
        saveSettings();
        }
      else if (strcmp(nme,"pass")==0)
        {
        strcpy(settings.mqttPassword,val);
        saveSettings();
        }
      else if (strcmp(nme,"ssid")==0)
        {
        strcpy(settings.ssid,val);
        saveSettings();
        }
      else if (strcmp(nme,"wifipass")==0)
        {
        strcpy(settings.wifiPassword,val);
        saveSettings();
        }
      else if (strcmp(nme,"address")==0)
        {
        strcpy(settings.address,val);
        saveSettings();
        }
      else if (strcmp(nme,"netmask")==0)
        {
        strcpy(settings.netmask,val);
        saveSettings();
        }
      else if (strcmp(nme,"debug")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.debug=atoi(val)==1?true:false;
        saveSettings();
        }
      else if (strcmp(nme,"invertdisplay")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.invertdisplay=atoi(val)==1?true:false;
        display.setRotation(settings.invertdisplay?2:0); //go ahead and do it
        saveSettings();
        }
      else if (strcmp(nme,"measureinterval")==0)
        {
        if (!val)
          strcpy(val,"0");
        settings.measureInterval=atoi(val);
        saveSettings();
        }
      else if ((strcmp(nme,"resetmqttid")==0)&& (strcmp(val,"yes")==0))
        {
        generateMqttClientId(settings.mqttClientId);
        saveSettings();
        }
      else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
        {
        Serial.println("\n*********************** Resetting EEPROM Values ************************");
        initializeSettings();
        saveSettings();
        delay(2000);
        ESP.restart();
        }
      else
        {
        showSettings();
        commandFound=false; //command not found
        }
      }
    }
  return commandFound;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  strcpy(settings.mqttBrokerAddress,""); //default
  settings.mqttBrokerPort=1883;
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttPassword,"");
  strcpy(settings.mqttTopicRoot,"");
  strcpy(settings.address,"");
  strcpy(settings.netmask,"255.255.255.0");
  settings.invertdisplay=false;
  settings.measureInterval=DEFAULT_MEASURE_INTERVAL;
  generateMqttClientId(settings.mqttClientId);
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    incomingSerialData();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      processCommand(cmd);
      }
    }
  }


/*
Spec sheet says readings are linear from 0.5v @ 0psi to 4.5v @ 150psi.
It also says the voltage @ 100psi is 2.5v.  This is not a linear progression.
I have my doubts but that's what I'm using here.  The input voltage
argument has already been stepped down by the voltage divider to a maximum
of 3.3v.
*/


/*
Spec sheet says readings are linear from 0.5v @ 0psi to 4.5v @ 150psi.
It also says the voltage @ 100psi is 2.5v.  This is not a linear progression.
I am dubious so I made my own measurements at the sensor:

Pressure	Measured Volts	Raw ADC Value
0	        0.51	          120
10	      0.67	          153
20	      0.91	          217
30	      1.16	          270
40	      1.42	          327
50	      1.70	          379
60	      1.98	          442
70	      2.24	          495
80	      2.51	          555
90	      2.76	          610
100	      3.03	          664
108	      3.27	          714


The two functions below were written by Gemini.
*/

///////////////////////////////////////////////////////////////////////////

// Define the new linear coefficients for ADC reading to Pressure
// These coefficients directly map raw ADC values to PSI for the range >= 20 PSI.
// Derived from measured ADC values.
#define LINEAR_M_ADC 0.19967664f     // Slope (m)
#define LINEAR_C_ADC -22.95159025f   // Y-intercept (c)

// Define the approximate raw ADC reading for 0 PSI from your calibration data (120)
#define ADC_READING_0PSI_CALIBRATED 120

// Define the approximate raw ADC reading for 20 PSI from your calibration data (217)
// This marks the lower bound of the linear fit's accurate range.
#define ADC_READING_10PSI_CALIBRATED 153


/**
 * @brief Converts a raw ADC reading to pressure (PSI) using a linear fit.
 * The polynomial equation is: Pressure = m * ADC_Reading + c
 * This function directly calculates pressure using coefficients derived from
 * raw ADC readings vs. measured pressures for the >= 20 PSI range.
 * @param adc_reading The measured raw ADC reading (0-1023).
 * @return The calculated pressure in PSI.
 */
float convertADCToPressureLinear(int adc_reading)
  {
  // Ensure adc_reading is cast to float for floating-point arithmetic
  float calculatedPressure = LINEAR_M_ADC * (float)adc_reading + LINEAR_C_ADC;

  // Add robust checks to ensure the output pressure is within the expected physical range
  if (calculatedPressure < MIN_PRESSURE_PSI) // Assuming MIN_PRESSURE_PSI is 0.0f
    {
    if (settings.debug)
      {
      Serial.print("Warning: Calculated pressure (");
      Serial.print(calculatedPressure, 2); // Print with 2 decimal places
      Serial.println(") is below MIN_PRESSURE_PSI. Clamping.");
      }
    calculatedPressure = MIN_PRESSURE_PSI; // Clamp to minimum
    }
  else if (calculatedPressure > MAX_PRESSURE_PSI) // Assuming MAX_PRESSURE_PSI is 150.0f
    {
    if (settings.debug)
      {
      Serial.print("Warning: Calculated pressure (");
      Serial.print(calculatedPressure, 2);
      Serial.println(") is above MAX_PRESSURE_PSI. Clamping.");
      }
    calculatedPressure = MAX_PRESSURE_PSI; // Clamp to maximum
    }

  return calculatedPressure; // Single exit point
  }


float read_pressure()
  {
  float finalPressure = 0.0f; // Variable to hold the calculated pressure for a single exit point

  digitalWrite(ACTIVITY_LED_PIN,HIGH);
  activityLedTimeoff = millis() + 125; // LED will light for one-eighth second

  int reading = analogRead(PRESSURE_SENSOR_PORT);  // reading is unitless 0 to 1023

  if (settings.debug)
    {
    Serial.print("Raw ADC reading: ");
    Serial.println(reading);
    }

  // Handle ADC readings that are below the effective range of our linear fit (below 20 PSI).
  // Given gauge inaccuracy below 20 PSI and that this range is rare.
  if (reading < ADC_READING_10PSI_CALIBRATED)
    {
    if (settings.debug)
      {
      Serial.print("ADC reading (");
      Serial.print(reading);
      Serial.print(") is below the 10 PSI calibrated range (ADC ");
      Serial.print(ADC_READING_10PSI_CALIBRATED);
      Serial.println(").");
      }

    // Specific clamping for readings very close to 0 PSI to manage noise.
    // If the reading is within a small window around the 0 PSI calibrated point.
    if (reading >= ADC_READING_0PSI_CALIBRATED - 10 && reading <= ADC_READING_0PSI_CALIBRATED + 10)
      {
      if (settings.debug)
        {
        Serial.print("Clamping pressure to 0 PSI due to proximity to ADC ");
        Serial.println(ADC_READING_0PSI_CALIBRATED);
        }
      finalPressure = MIN_PRESSURE_PSI; // Explicitly set to 0 PSI
      }
    else if (reading < ADC_READING_0PSI_CALIBRATED - 10) // Significantly below 0 PSI equivalent (e.g., sensor fault)
      {
      if (settings.debug)
        {
        Serial.print("Warning: ADC reading (");
        Serial.print(reading);
        Serial.println(") is extremely low, indicating possible sensor fault or out of range.");
        }
      finalPressure = -1; // Sensor error
      }
    else // For readings between ~0 PSI and 20 PSI (ADC 130-216), where gauge is less reliable
      {
      if (settings.debug)
        {
        Serial.println("Applying linear model to low-pressure region; note potential gauge inaccuracy here.");
        }
      // Apply the linear model, understanding its reduced accuracy/validity in this range based on your gauge insight.
      finalPressure = convertADCToPressureLinear(reading);
      }
    }
  // For all other readings (>= ADC_READING_10PSI_CALIBRATED), apply the main linear conversion
  else
    {
    finalPressure = convertADCToPressureLinear(reading);
    }

  return finalPressure; // Single exit point
  }


/////////////////////////////////////////////////////////////////////////




/************************
 * Do the MQTT thing
 ************************/
bool report(int pressure)
  {
  char topic[MQTT_TOPIC_SIZE+9];
  char reading[18];
  bool ok=true;
  sprintf(topic,"%spressure",settings.mqttTopicRoot);

  if (pressure >=0)
    sprintf(reading,"%d",pressure);
  else
    sprintf(reading,"error");
    
  ok=publish(topic,reading,true);

  //publish the radio strength reading while we're at it
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_RSSI);
  sprintf(reading,"%d",WiFi.RSSI()); 
  ok=ok|publish(topic,reading,true); //retain

  //publish the status
  sprintf(topic,"%sstatus",settings.mqttTopicRoot);
  if (pressure < 0)  //then there was an error reading the sensor
    {
    ok=ok|publish(topic,"error",true);
    }
  else
    {
    ok=ok|publish(topic,"ok",true);
    }
  
  if (settings.debug)
    {
    Serial.print("Publish ");
    Serial.println(ok?"OK":"Failed");
    }
  if (!ok)
    show("Pub Fail.");
  return ok;
  }


boolean publish(char* topic, const char* reading, boolean retain)
  {
  if (settings.debug)
    {
    Serial.print(topic);
    Serial.print(" ");
    Serial.println(reading);
    }
  boolean ok=false;
  connectToWiFi(); //just in case we're disconnected from WiFi
  reconnect(); //also just in case we're disconnected from the broker

  if (mqttClient.connected() && 
      settings.mqttTopicRoot &&
      WiFi.status()==WL_CONNECTED)
    {
    ok=mqttClient.publish(topic,reading,retain); 
    }
  else
    {
    Serial.print("Can't publish due to ");
    if (WiFi.status()!=WL_CONNECTED)
      Serial.println("no WiFi connection.");
    else if (!mqttClient.connected())
      Serial.println("not connected to broker.");
    }
  return ok;
  }



/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. 
 * The MQTT message topic sent is the topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  if (settings.debug)
    {
    Serial.println("====================================> Callback works.");
    }
  payload[length]='\0'; //this should have been done in the calling code, shouldn't have to do it here
  boolean rebootScheduled=false; //so we can reboot after sending the reboot response
  char charbuf[100];
  sprintf(charbuf,"%s",payload);
  const char* response;
  
  
  //if the command is MQTT_PAYLOAD_SETTINGS_COMMAND, send all of the settings
  if (strcmp(charbuf,MQTT_PAYLOAD_SETTINGS_COMMAND)==0)
    {
    char tempbuf[35]; //for converting numbers to strings
    char jsonStatus[JSON_STATUS_SIZE];
    
    strcpy(jsonStatus,"{");
    strcat(jsonStatus,"\"broker\":\"");
    strcat(jsonStatus,settings.mqttBrokerAddress);
    strcat(jsonStatus,"\", \"port\":");
    sprintf(tempbuf,"%d",settings.mqttBrokerPort);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,", \"topicroot\":\"");
    strcat(jsonStatus,settings.mqttTopicRoot);
    strcat(jsonStatus,"\", \"user\":\"");
    strcat(jsonStatus,settings.mqttUsername);
    strcat(jsonStatus,"\", \"pass\":\"");
    strcat(jsonStatus,settings.mqttPassword);
    strcat(jsonStatus,"\", \"ssid\":\"");
    strcat(jsonStatus,settings.ssid);
    strcat(jsonStatus,"\", \"wifipass\":\"");
    strcat(jsonStatus,settings.wifiPassword);
    strcat(jsonStatus,"\", \"mqttClientId\":\"");
    strcat(jsonStatus,settings.mqttClientId);
    strcat(jsonStatus,"\", \"address\":\"");
    strcat(jsonStatus,settings.address);
    strcat(jsonStatus,"\", \"netmask\":\"");
    strcat(jsonStatus,settings.netmask);
    strcat(jsonStatus,"\", \"debug\":\"");
    strcat(jsonStatus,settings.debug?"true":"false");
    strcat(jsonStatus,"\", \"invertdisplay\":\"");
    strcat(jsonStatus,settings.invertdisplay?"true":"false");
    strcat(jsonStatus,"\", \"measureinterval\":");
    sprintf(tempbuf,"%d",settings.measureInterval);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"IPAddress\":\"");
    strcat(jsonStatus,wifiClient.localIP().toString().c_str());
    
    strcat(jsonStatus,"\"}");
    response=jsonStatus;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_VERSION_COMMAND)==0) //show the version number
    {
    char tmp[15];
    strcpy(tmp,VERSION);
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_STATUS_COMMAND)==0) //show the latest value
    {
    report(read_pressure());
    
    char tmp[25];
    strcpy(tmp,"Status report complete");
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_REBOOT_COMMAND)==0) //reboot the controller
    {
    char tmp[10];
    strcpy(tmp,"REBOOTING");
    response=tmp;
    rebootScheduled=true;
    }
  else if (processCommand(charbuf))
    {
    response="OK";
    }
  else
    {
    char badCmd[18];
    strcpy(badCmd,"(empty)");
    response=badCmd;
    }
    
  char topic[MQTT_TOPIC_SIZE];
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,charbuf); //the incoming command becomes the topic suffix

  if (!publish(topic,response,false)) //do not retain
    Serial.println("************ Failure when publishing status response!");
    
  delay(2000); //give publish time to complete
  
  if (rebootScheduled)
    {
    ESP.restart();
    }
  }


//Generate an MQTT client ID.  This should not be necessary very often
char* generateMqttClientId(char* mqttId)
  {
  strcpy(mqttId,MQTT_CLIENT_ID_ROOT);
  strcat(mqttId, String(random(0xffff), HEX).c_str());
  if (settings.debug)
    {
    Serial.print("New MQTT userid is ");
    Serial.println(mqttId);
    }
  return mqttId;
  }


void setup_wifi()
  {
  // WiFi connection setup code here
  if (WiFi.status() != WL_CONNECTED)
    {
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");

    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world

    if (ip.isSet()) //Go with a dynamic address if no valid IP has been entered
      {
      if (!WiFi.config(ip,ip,mask))
        {
        Serial.println("STA Failed to configure");
        }
      }

    unsigned long connectTimeout = millis() + WIFI_TIMEOUT_SECONDS*1000; // 10 second timeout
    WiFi.begin(settings.ssid, settings.wifiPassword);
    while (WiFi.status() != WL_CONNECTED && millis() < connectTimeout) 
      {
      // not yet connected
      Serial.print(".");
      checkForCommand(); // Check for input in case something needs to be changed to work
      delay(500);
      }
    
    checkForCommand(); // Check for input in case something needs to be changed to work

    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("Connection to network failed. ");
      Serial.println();
//      show("Wifi failed to \nconnect");
      delay(3000);
      }
    else 
      {
      Serial.print("Connected to network with address ");
      Serial.println(WiFi.localIP());
      Serial.println();
      show(WiFi.localIP().toString());
      }
    }
  } 

/*
 * Reconnect to the MQTT broker
 */
void reconnect() 
  {
  if (strlen(settings.mqttBrokerAddress)>0)
    {
    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("WiFi not ready, skipping MQTT connection");
      }
    else
      {
      // Loop until we're reconnected
      while (!mqttClient.connected()) 
        {
        show("Connecting\nto MQTT");    
        Serial.print("Attempting MQTT connection...");

        mqttClient.setBufferSize(JSON_STATUS_SIZE); //default (256) isn't big enough
        mqttClient.setKeepAlive(120); //seconds
        mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
        mqttClient.setCallback(incomingMqttHandler);
        
        // Attempt to connect
        if (mqttClient.connect(settings.mqttClientId,settings.mqttUsername,settings.mqttPassword))
          {
          Serial.println("connected to MQTT broker.");
          show("Connected\nto MQTT");

          //resubscribe to the incoming message topic
          char topic[MQTT_TOPIC_SIZE];
          strcpy(topic,settings.mqttTopicRoot);
          strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
          bool subgood=mqttClient.subscribe(topic);
          showSub(topic,subgood);
          }
        else 
          {
          Serial.print("failed, rc=");
          Serial.println(mqttClient.state());
          Serial.println("Will try again in a second");
          
          // Wait a second before retrying
          // In the meantime check for input in case something needs to be changed to make it work
        //  checkForCommand(); 
          
          delay(1000);
          }
        checkForCommand();
        }
      mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason
      }
    }
  else if (settings.debug)
    {
    Serial.println("Broker address not set, ignoring MQTT");
    }
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
    }
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
      strlen(settings.wifiPassword)>0 &&
      // strlen(settings.mqttBrokerAddress)>0 &&
      // settings.mqttBrokerPort!=0 &&
      strlen(settings.mqttTopicRoot)>0 &&
      strlen(settings.mqttClientId)>0)
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
    
  //The mqttClientId is not set by the user, but we need to make sure it's set  
  if (strlen(settings.mqttClientId)==0)
    {
    generateMqttClientId(settings.mqttClientId);
    }
    
  EEPROM.put(0,settings);
  if (settings.debug)
    Serial.println("Committing settings to eeprom");
  return EEPROM.commit();
  }

// populate the box struct from the received json
void deserialize(StaticJsonDocument<250> &doc)
  {
  boxStatus.address=doc["address"];
  boxStatus.battery=doc["battery"];
  boxStatus.distance=doc["distance"];
  }


void initSerial()
  {
  Serial.begin(115200);
  Serial.setTimeout(10000);
  
  while (!Serial); // wait here for serial port to connect.
  Serial.println();
  Serial.println("Serial communications established.");
  }

/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    if (settings.debug)
      {
      Serial.println("\nLoaded configuration values from EEPROM");
      }
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
    showSettings();
  }


void initSettings()
  {
  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string

  loadSettings(); //set the values from eeprom 

  //show the MAC address
  Serial.print("ESP8266 MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (settings.mqttBrokerPort < 0) //then this must be the first powerup
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  }

/*
 * If not connected to wifi, connect.
 */
void connectToWiFi()
  {
  if (settingsAreValid && WiFi.status() != WL_CONNECTED)
    {
    if (settings.debug)
      show("Connecting\nto WiFi");
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");

//    WiFi.forceSleepWake(); //turn on the radio
//    delay(1);              //return control to let it come on
    
    WiFi.disconnect(true); // Completely reset Wi-Fi stack
    delay(100); // Small delay to ensure reset is applied
    WiFi.persistent(false); // Prevent saving to flash
    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world

    if (ip.isSet()) //Go with a dynamic address if no valid IP has been entered
      {
      if (!WiFi.config(ip,ip,mask))
        {
        Serial.println("STA Failed to configure");
        }
      }

    unsigned long connectTimeout = millis() + WIFI_TIMEOUT_SECONDS*1000; // 10 second timeout
    WiFi.begin(settings.ssid, settings.wifiPassword);
    delay(1000);
    while (WiFi.status() != WL_CONNECTED && millis() < connectTimeout) 
      {
      // not yet connected
      // Serial.print(".");
      // checkForCommand(); // Check for input in case something needs to be changed to work
      Serial.print(".");
      checkForCommand();
      delay(500);
      }
    
    checkForCommand(); // Check for input in case something needs to be changed to work

    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("\nConnection to network failed. ");
      delay(3000);
      }
    else 
      {
      Serial.print("\nConnected to network with address ");
      Serial.println(WiFi.localIP());
      Serial.println();
          // if this is just turning on, reshow the last message except smaller
      if (!rssiShowing)
        {
        rssiShowing=true;
        show(lastMessage);
        }
      if (settings.debug)
        show("Connected\nTo Wifi");
      }
    }
  }

void initDisplay()
  {
  if (settings.debug)
    {
    Serial.println("Initializing display");
    }
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
    {
    Serial.println(F("SSD1306 allocation failed"));
    delay(5000);
    ESP.reset();  //try again
    }
  display.setRotation(settings.invertdisplay?2:0); //make it look right
  display.clearDisplay();       //no initial logo
  display.setTextSize(3);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  if (settings.debug)
    show("Init");
  }

// A map() funtction that works with floats instead of ints

float fmap(float value, float fromLow, float fromHigh, float toLow, float toHigh) 
  {
  return toLow+(value-fromLow)*(toHigh-toLow)/(fromHigh-fromLow);
  }

// float read_pressure()
//   {
//   digitalWrite(ACTIVITY_LED_PIN,HIGH);
//   activityLedTimeoff=millis()+125; //LED will light for one-eighth second

//   // The ESP processor can only handle a maximum of 1 volt, so the 
//   // D1 mini has a voltage divider on it to allow up to 3.3 volts
//   // on the external port pin. We need to convert this reading to 
//   // a voltage from 0 to 3.3 volts.
//   int reading=analogRead(PRESSURE_SENSOR_PORT);  //reading is unitless 0 to 1023

//   // When the pressure is at zero (open air pressure), the input reading will dance
//   // around the actual reading and one that is just low enough to make the code think
//   // that the sensor has failed. Check for this condition and clamp it to the value
//   // for zero if it's just below that. If it's too low then it probably is an actual
//   // sensor failure.
//   if (reading > 100 && reading < 113)
//     {
//     if (settings.debug)
//       {
//       Serial.print("Clamping input reading of ");
//       Serial.print(reading);
//       Serial.println(" to 113");
//       }
//     reading=113;
//     }

//   float fReading=fmap((float)reading,114.0f,1023.0f,0.336f,3.3f); //convert to voltage 0v - 3.3v

//   // Since the ESP board can only accept voltages on the analog port
//   // up to 3.3 volts, I had to add another voltage divider to bring the 
//   // maximum 5v from the sensor down to the maximum 3.3v that the 
//   // board can handle.  This line reverses the effects of that.

//   // Measured voltage divider values in schematic
//   #define R1 4932.0f
//   #define R2 9681.0f

//   float sensorVolts=fReading/(R2/(R1+R2)); //this gives the voltage from the sensor 

//   if (settings.debug)
//     {
//     Serial.print("Raw reading: ");
//     Serial.println(reading);
//     Serial.print("Mapped reading: ");
//     Serial.println(fReading);
//     Serial.print("Measured voltage: ");
//     Serial.println(sensorVolts);
//     }

//   return convertVoltageToPressure(sensorVolts);
//   }

void setup()
  {
  initSerial();

  initSettings();

  pinMode(SHOW_PRESSURE_PIN,INPUT_PULLUP); //the button to light up the display
  pinMode(ACTIVITY_LED_PIN, OUTPUT);
  digitalWrite(ACTIVITY_LED_PIN, LOW);

  if (settingsAreValid)
    {      
    //initialize everything
    initDisplay();

    if (settings.debug)
      {
      if (!ip.fromString(settings.address))
        {
        Serial.println("IP Address "+String(settings.address)+" is not valid. Using dynamic addressing.");
        // settingsAreValid=false;
        // settings.validConfig=false;
        }
      else if (!mask.fromString(settings.netmask))
        {
        Serial.println("Network mask "+String(settings.netmask)+" is not valid.");
        // settingsAreValid=false;
        // settings.validConfig=false;
        }
      }
    }
  //showSettings();
  }

void loop()
  {
  static unsigned long takeReadingTime=millis();
  static ulong screensaver=millis()+DISPLAY_TIME; //how long to show a message before blanking the display
  static bool cleared=false;  //display has been cleared
  static int pressure;
  static int lastPressure=0;
  
  if (millis() > activityLedTimeoff)
    digitalWrite(ACTIVITY_LED_PIN,LOW); //turn off the activity LED

  if (millis()>screensaver && WiFi.status() == WL_CONNECTED && mqttClient.connected())
    {
    if (settings.debug && !cleared)
      {
      Serial.println("Clearing display");
      }
    show(""); //don't wear out the display
    cleared=true;
    }

  // If the show button is pressed, read and display the pressure
  // but don't send it to MQTT until the right time
  bool viewPressure=!digitalRead(SHOW_PRESSURE_PIN); //active low
  if (viewPressure)
    {
    screensaver=millis()+DISPLAY_TIME; 
    cleared=false;
    
    if (pressure>=0)
      show(String(pressure)+" PSI");
    else
      show("Sensor\nFailure");
    }

  if (millis() >= takeReadingTime)  //time to read the sensor
    {
    pressure=read_pressure();
    if (mqttClient.connected() && WiFi.status() == WL_CONNECTED)
      {
      report(pressure);
      }

    if (pressure>=0)
      {
      if (lastPressure != pressure)
        {
        if (settings.debug)
          {
          Serial.println("Showing display");
          }
        show(String(pressure)+" PSI");
        lastPressure=pressure;
        screensaver=millis()+DISPLAY_TIME; //when to blank the display
        cleared=false;
        }
      Serial.print(pressure);
      Serial.println(" PSI");
      }
    else
      {
      show("Sensor\nFailure");
      lastPressure=0;
      screensaver=millis()+DISPLAY_TIME; //when to blank the display
      cleared=false;
      Serial.println("Pressure sensor failure.");
      }

    takeReadingTime=millis()+settings.measureInterval*1000;
    }

  if (settingsAreValid)
    {      
    if (WiFi.status() != WL_CONNECTED)
      {
      connectToWiFi();
      }
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED)
      {
      reconnect();
      }  
    else 
      mqttClient.loop();

    }
  checkForCommand();
  }

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void incomingSerialData() 
  {
  while (Serial.available()) 
    {
    char inChar = (char)Serial.read(); // get the new byte
    Serial.print(inChar); //echo it back to the terminal

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n' || inChar == '\r') 
      {
      commandComplete = true;
      }
    else
      {
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }
