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
  if (rssiShowing)
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


/**
 * @brief Converts a voltage reading to pressure (PSI) using a 2nd order polynomial fit.
 * The polynomial equation is: Volts = a * Pressure^2 + b * Pressure + c
 * This function solves for pressure using the quadratic formula.
 * Function was developed with the assistance of Google Gemini and LibreOffice Calc
 * 
 * @param voltage_v The measured voltage in volts.
 * @return The calculated pressure in PSI. Returns a special value (e.g., -1.0)
 * if the voltage is outside the valid range or results in no real solution.
 */
float convertVoltageToPressure(float voltage_v)
  {
  // Rearrange the polynomial to the quadratic form: a*P^2 + b*P + (c - V) = 0
  // Here, (c - V) is the 'C' term in the standard quadratic equation Ax^2 + Bx + C = 0
  // Our A = POLY_A, B = POLY_B, and C_quadratic = POLY_C - voltage_v

  float C_quadratic = POLY_C - voltage_v;

  // Calculate the discriminant (the part under the square root in the quadratic formula)
  float discriminant = POLY_B * POLY_B - 4 * POLY_A * C_quadratic;

  // Check if there's a real solution (discriminant must be non-negative)
  if (discriminant < 0) 
    {
    // No real solution for pressure with this voltage.
    // This might happen if the voltage is outside the range that the polynomial covers,
    // or if there's sensor noise pushing it beyond the valid mathematical domain.
    Serial.println("Error: Discriminant < 0. Voltage out of calculated range.");
    return -1.0; // Return an error indicator
    }

  // Calculate the two possible solutions for Pressure
  // The quadratic formula gives two roots, but only one will be physically meaningful
  float pressure1 = (-POLY_B + sqrt(discriminant)) / (2 * POLY_A);
  float pressure2 = (-POLY_B - sqrt(discriminant)) / (2 * POLY_A);

  // Determine the physically meaningful pressure value within the expected range
  // Since pressure typically increases with voltage for these sensors, we expect one positive root.
  // We should check which root falls within our sensor's operating range (0-150 PSI).

  if (pressure1 >= MIN_PRESSURE_PSI && pressure1 <= MAX_PRESSURE_PSI) 
    {
    return pressure1;
    } 
  else if (pressure2 >= MIN_PRESSURE_PSI && pressure2 <= MAX_PRESSURE_PSI) 
    {
    return pressure2;
    }
  else 
    {
    // Neither calculated pressure is within the expected range.
    // This can happen if the measured voltage is outside the sensor's
    // calibrated range (e.g., negative pressure, or pressure beyond max).
    Serial.print("Error: Calculated pressure (");
    Serial.print(pressure1, 2);
    Serial.print(", ");
    Serial.print(pressure2, 2);
    Serial.println(") is outside expected range.");
    return -1.0; // Return an error indicator
  }
}


/************************
 * Do the MQTT thing
 ************************/
bool report(int pressure)
  {
  char topic[MQTT_TOPIC_SIZE+9];
  char reading[18];
  bool ok=true;
  sprintf(topic,"%spressure",settings.mqttTopicRoot);
  sprintf(reading,"%d",pressure);
  ok=publish(topic,reading,true);
  
  Serial.print("Publish ");
  Serial.println(ok?"OK":"Failed");
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

float read_pressure()
  {
  // The ESP processor can only handle a maximum of 1 volt, so the 
  // D1 mini has a voltage divider on it to allow up to 3.3 volts
  // on the external port pin. We need to convert this reading to 
  // a voltage from 0 to 3.3 volts.
  int reading=analogRead(PRESSURE_SENSOR_PORT);  //reading is unitless 0 to 1023
  float fReading=fmap((float)reading,0.0f,1023.0f,0.0f,3.32f); //convert to voltage 0v - 3.3v

  // Since the ESP board can only accept voltages on the analog port
  // up to 3.3 volts, I had to add another voltage divider to bring the 
  // maximum 5v from the sensor down to the maximum 3.3v that the 
  // board can handle.  This line reverses the effects of that.

  //voltage divider values in schematic
  #define R1b 4932.0f
  #define R2b 9681.0f

  float sensorVolts=fReading/(R2b/(R1b+R2b)); //this gives the voltage from the sensor 

  if (settings.debug)
    {
    Serial.print("Measured voltage: ");
    Serial.println(sensorVolts);
    }

  return convertVoltageToPressure(sensorVolts);
  }

void setup()
  {
  initSerial();

  initSettings();

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
  static int lastPressure=0;

  if (millis()>screensaver && WiFi.status() == WL_CONNECTED && mqttClient.connected())
    {
    if (settings.debug && !cleared)
      {
      Serial.println("Clearing display");
      }
    show(""); //don't wear out the display
    cleared=true;
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
    if (millis() >= takeReadingTime)  //time to read the sensor
      {
      int pressure=read_pressure();
      report(pressure);

      
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

      takeReadingTime=millis()+MEASURE_INTERVAL;
    
      //handle the case that millis is about to roll over to zero
      if (takeReadingTime<millis()) //overflow!
        {
        delay(MEASURE_INTERVAL); //let millis() catch up
        }
      }
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
