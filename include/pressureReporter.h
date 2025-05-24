#define MAX_HARDWARE_FAILURES 20
#define VALID_SETTINGS_FLAG 0xDAB0
#define LED_ON LOW
#define LED_OFF HIGH
#define SSID_SIZE 100
#define PASSWORD_SIZE 50
#define ADDRESS_SIZE 30
#define USERNAME_SIZE 50
#define MQTT_CLIENTID_SIZE 25
#define MQTT_TOPIC_SIZE 150
#define MQTT_TOPIC_DISTANCE "distance"
#define MQTT_TOPIC_STATE "present"
#define MQTT_TOPIC_BATTERY "battery"
#define MQTT_TOPIC_ANALOG "analog"
#define MQTT_TOPIC_RSSI "rssi"
#define MQTT_TOPIC_SNR "snr"
#define MQTT_CLIENT_ID_ROOT "DeliveryReporter"
#define MQTT_TOPIC_COMMAND_REQUEST "command"
#define MQTT_PAYLOAD_SETTINGS_COMMAND "settings" //show all user accessable settings
#define MQTT_PAYLOAD_RESET_PULSE_COMMAND "resetPulseCounter" //reset the pulse counter to zero
#define MQTT_PAYLOAD_REBOOT_COMMAND "reboot" //reboot the controller
#define MQTT_PAYLOAD_VERSION_COMMAND "version" //show the version number
#define MQTT_PAYLOAD_STATUS_COMMAND "status" //show the most recent flow values
#define JSON_STATUS_SIZE SSID_SIZE+PASSWORD_SIZE+USERNAME_SIZE+MQTT_TOPIC_SIZE+150 //+150 for associated field names, etc
#define PUBLISH_DELAY 400 //milliseconds to wait after publishing to MQTT to allow transaction to finish
#define WIFI_TIMEOUT_SECONDS 20 // give up on wifi after this long
#define FULL_BATTERY_COUNT 3686 //raw A0 count with a freshly charged 18650 lithium battery 
#define FULL_BATTERY_VOLTS 412 //4.12 volts for a fully charged 18650 lithium battery 
#define ONE_HOUR 3600000 //milliseconds
#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 32      // OLED display height, in pixels
#define OLED_RESET    -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define DOT_RADIUS    2       // radius of activity dot
#define DOT_SPACING   4       // spacing between dots

#define RSSI_DOT_RADIUS 2       // Radius of the little dot at the bottom of the wifi indicator
#define SHOWBUF_LENGTH 20     // Number of entries in the show buffer
#define SHOWBUF_WIDTH 20     // Max size of entries in the show buffer

#define PRESSURE_SENSOR_PORT A0 //Analog port to which the pressure sensor is attached
#define MEASURE_INTERVAL 5000 // 5 seconds between readings
#define DISPLAY_TIME 5000  // Show the display only for this long when value changes

// --- POLYNOMIAL COEFFICIENTS USED TO CONVERT VOLTS TO PSI ---
// Obtained from LibreOffice Calc's LINEST function
// Must use sufficient precision (e.g., 8-10 decimal places)
const float POLY_A = 0.0001333333; // Coefficient 'a' for Pressure^2
const float POLY_B = 0.0066666667; // Coefficient 'b' for Pressure
const float POLY_C = 0.5;          // Coefficient 'c' (y-intercept)

// The pressure sensor's pressure range
const float MIN_PRESSURE_PSI = 0.0;
const float MAX_PRESSURE_PSI = 150.0; 



void showSettings();
String getConfigCommand();
bool processCommand(String cmd);
void checkForCommand();
float read_pressure();
bool report(int pressure);
boolean publish(char* topic, const char* reading, boolean retain);
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) ;
void setup_wifi();
void connectToWiFi();
void reconnect();
void showSub(char* topic, bool subgood);
void initializeSettings();
boolean saveSettings();
void deserialize(StaticJsonDocument<250> &doc);
void setup();
void loop();
void incomingSerialData();
char* generateMqttClientId(char* mqttId);

//MQTT status for reference only
// MQTT_CONNECTION_TIMEOUT     -4
// MQTT_CONNECTION_LOST        -3
// MQTT_CONNECT_FAILED         -2
// MQTT_DISCONNECTED           -1
// MQTT_CONNECTED               0
// MQTT_CONNECT_BAD_PROTOCOL    1
// MQTT_CONNECT_BAD_CLIENT_ID   2
// MQTT_CONNECT_UNAVAILABLE     3
// MQTT_CONNECT_BAD_CREDENTIALS 4
// MQTT_CONNECT_UNAUTHORIZED    5