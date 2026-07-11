//==============================================================
// ESP32_6TOTI_WiFi based on;-
// ESP32_2Servo_2Frog_2TOTI_WiFi based on;-
// ESP32_2Frog_WiFi based on;
// ESP32_2TOTI_WiFi based on;-
// Pico_2TOTI_WiFi based on;-
// https://github.com/openlcb/OpenLCB_Single_Thread/tree/master/examples/Pico_8ServoWifiGC
//
// Modified John Callingham 2025, removed servo code and added;-
// - code to read input pins and send events when a change is detected.
// - code to respond to a JMRI query and return the current state of a TOTI.
// MOdified DPH 2024
// Copyright 2019 Alex Shepherd and David Harris
//
// 18 June 2026 - changed version of OpenLCB_Single_Thread from 0.1.14 to 0.1.19
//==============================================================

// Debugging -- uncomment to activate debugging statements:
    // dP(x) prints x, 
    // dPH(x) prints x in hex, 
    // dPS(string,x) prints string and x
#include <Arduino.h>
#include "credentials.h"
#include "ESP32WiFiGC_V2.h"
#include "TOTI.h"
#include "configurationOTA.h"
#include "configurationPreferences.h"

#define DEBUG Serial
#define NOCAN

// Board definitions
#define MANU "J Callingham"  // The manufacturer of node
#define MODEL "ESP32_6TOTI_Wifi" // The model of the board
#define HWVERSION "0.1"   // Hardware version
#define SWVERSION "1.0.0"   // Software version

// To Reset the Node Number, Uncomment and edit the next line
// Need to do this at least once.  
// #define NODE_ADDRESS  5,1,1,1,0x91,0x07  // First servo node.
// #define NODE_ADDRESS  5,1,1,1,0x91,0x08  // Second servo node.
// #define NODE_ADDRESS  5,1,1,1,0x91,0x09  // Third servo node.
#define NODE_ADDRESS  5,1,1,1,0x91,0x0A  // Six TOTI node.

// Set to 1 to Force Reset EEPROM to Factory Defaults 
// Need to do this at least once.  
#define RESET_TO_FACTORY_DEFAULTS 0

#define NUM_RGB_LED 3 // Red, green and blue.
#define RGB_LED_RED 0 // The index to the RGB_LEDs array.
#define RGB_LED_GREEN 1 // The index to the RGB_LEDs array.
#define RGB_LED_BLUE 2 // The index to the RGB_LEDs array.

#define TOTI_EVENT_BASE 0 // This is the first TOTI event.
#define NUM_TOTI 6
#define NUM_TOTI_EVENT (NUM_TOTI * 2) // Occupied event and Not Occupied event for each TOTI

#define NUM_EVENT NUM_TOTI_EVENT

#define DESCRIPTION_LENGTH 16

/**
 * Definitions of the LED configuration property values.
 */
#define LED_CONFIG_NOT_CONFIGURED 0
#define LED_CONFIG_HUB_STATE 1
#define LED_CONFIG_TOTI_1_STATE 2
#define LED_CONFIG_TOTI_2_STATE 3
#define LED_CONFIG_TOTI_3_STATE 4
#define LED_CONFIG_TOTI_4_STATE 5
#define LED_CONFIG_TOTI_5_STATE 6
#define LED_CONFIG_TOTI_6_STATE 7

// These have all been changed from 8 bit to 16 bit variables so that the following debounce delay variables, which are 16 bit, are aligned to a 16 bit boundary.
uint16_t ledRedConfiguration;
uint16_t ledGreenConfiguration;
uint16_t ledBlueConfiguration;
bool ledConfigHubConnected = false;

// Forward declarations
uint8_t getLEDState(int switchInput);

/**
 * Configure TOTIs.
 */
#define TOTI_1_INPUT_PIN A6 // J1
#define TOTI_2_INPUT_PIN A7 // J2
#define TOTI_3_INPUT_PIN A1 // J3
#define TOTI_4_INPUT_PIN D1 // K1
#define TOTI_5_INPUT_PIN D2 // K2
#define TOTI_6_INPUT_PIN D0 // K3
#define TOTI_TEST_PIN D12 // Use the same test pin for all TOTIs.

// Create six TOTI objects and set their pins.
TOTI toti0(0, TOTI_1_INPUT_PIN, TOTI_TEST_PIN);
TOTI toti1(1, TOTI_2_INPUT_PIN, TOTI_TEST_PIN);
TOTI toti2(2, TOTI_3_INPUT_PIN, TOTI_TEST_PIN);
TOTI toti3(3, TOTI_4_INPUT_PIN, TOTI_TEST_PIN);
TOTI toti4(4, TOTI_5_INPUT_PIN, TOTI_TEST_PIN);
TOTI toti5(5, TOTI_6_INPUT_PIN, TOTI_TEST_PIN);

// Declare an array of pointers to the TOTI objects.
TOTI *toti[NUM_TOTI];

/**
 * Various #includes from the OpenLCB_Single_Thread library.
 */
#include "mdebugging.h"           // debugging
#include "processor.h"
#include "processCAN.h"
// #include "OpenLCBHeader.h"
#include "OpenLCBHeaderJC.h"

// CDI (Configuration Description Information) in xml, must match MemStruct
// See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.1-ConfigurationDescriptionInformation-2016-02-06.pdf
extern "C" {
    #define N(x) xN(x)     // allow the insertion of the value (x) ..
    #define xN(x) #x       // .. into the CDI string. 

const char configDefInfo[] PROGMEM =
// ===== Enter User definitions below =====
  CDIheader R"(

    <group replication=')" N(NUM_RGB_LED) R"('>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <name>LED Control</name>
      <repname>Red</repname><repname>Green</repname><repname>Blue</repname>

      <int size='2'>
        <name>LED function</name>
        <default>0</default>
        <map>
          <relation><property>0</property><value>Not configured</value></relation>
          <relation><property>1</property><value>Hub status</value></relation>
          <relation><property>2</property><value>TOTI J1 status</value></relation>
          <relation><property>3</property><value>TOTI J2 status</value></relation>
          <relation><property>4</property><value>TOTI J3 status</value></relation>
          <relation><property>5</property><value>TOTI K1 status</value></relation>
          <relation><property>6</property><value>TOTI K2 status</value></relation>
          <relation><property>7</property><value>TOTI K3 status</value></relation>
        </map>
      </int>
    </group>

    <group replication=')" N(NUM_TOTI) R"('>
        <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
        <name>TOTIs</name>
        <repname>J1</repname><repname>J2</repname><repname>J3</repname><repname>K1</repname><repname>K2</repname><repname>K3</repname>
        <string size=')" N(DESCRIPTION_LENGTH) R"('><name>Description</name></string>
        <int size='2'>
          <name>Occupied Delay</name>
          <description>The number of mS that this TOTI should permanently indicate occupied before the occupied event is sent.</description>
          <min>0</min><max>5000</max>
          <hints><slider tickSpacing='1000' immediate='yes' showValue='true'></slider></hints>
        </int>
        <int size='2'>
          <name>Not Occupied Delay</name>
          <description>The number of mS that this TOTI should permanently indicate not occupied before the not occupied event is sent.</description>
          <min>0</min><max>5000</max>
          <hints><slider tickSpacing='1000' immediate='yes' showValue='true'></slider></hints>
        </int>
        <eventid><name>Occupied Event</name></eventid>
        <eventid><name>Not Occupied Event</name></eventid>
    </group>
  )" CDIfooter;
// ===== Enter User definitions above =====
} // end extern

// ===== MemStruct =====
//   Memory structure of EEPROM, must match CDI above
typedef struct { 
  EVENT_SPACE_HEADER eventSpaceHeader; // MUST BE AT THE TOP OF STRUCT - DO NOT REMOVE!!!
  
  char nodeName[20];  // optional node-name, used by ACDI
  char nodeDesc[24];  // optional node-description, used by ACDI
  // ===== Enter User definitions below =====

  struct {
    uint16_t ledConfiguration; // Changed from 8 bit to 16 bit so that all variables align to a 16 bit boundary.
  } RGB_LEDs[NUM_RGB_LED];

  struct {
    char totidesc[DESCRIPTION_LENGTH];        // description of this TOTI
    uint16_t occupiedDelay;
    uint16_t notOccupiedDelay;
    EventID eidON;
    EventID eidOFF;
  } TOTIs[NUM_TOTI];

  // ===== Enter User definitions above =====
} MemStruct;       // type definition

// This is called to initialize the EEPROM during Factory Reset.
void userInitAll()
{
  NODECONFIG.put(EEADDR(nodeName), ESTRING("ESP32"));
  NODECONFIG.put(EEADDR(nodeDesc), ESTRING("6TOTI_WiFi"));

  NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration), LED_CONFIG_NOT_CONFIGURED); // The red LED default is not configured.
  NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration), LED_CONFIG_NOT_CONFIGURED); // The green LED default is not configured.
  NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration), LED_CONFIG_HUB_STATE); // The blue LED default is to show the hub connection status.

  // Set the default debounce delay for all TOTIs to 100 mS.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    NODECONFIG.update16(EEADDR(TOTIs[i].occupiedDelay), 100);
    NODECONFIG.update16(EEADDR(TOTIs[i].notOccupiedDelay), 100);
  }
}

extern "C" {
    // ===== eventid Table =====
    // useful macro to help fill the table

    #define REG_TOTI(s) PEID(TOTIs[s].eidON), PEID(TOTIs[s].eidOFF)
    
    //  Array of the offsets to every eventID in MemStruct/EEPROM/mem, and P/C flags
    const EIDTab eidtab[NUM_EVENT] PROGMEM = {
      REG_TOTI(0), REG_TOTI(1), REG_TOTI(2), REG_TOTI(3), REG_TOTI(4), REG_TOTI(5)
    };
    
    // SNIP Short node description for use by the Simple Node Information Protocol
    // See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.3-SimpleNodeInformation-2016-02-06.pdf
    // extern const char SNII_const_data[] PROGMEM = "\001" MANU "\000" MODEL "\000" HWVERSION "\000" OlcbCommonVersion ; // last zero in double-quote
    extern const char SNII_const_data[] PROGMEM = "\001" MANU "\000" MODEL "\000" HWVERSION "\000" SWVERSION ; // last zero in double-quote
} // end extern "C"

// PIP Protocol Identification Protocol uses a bit-field to indicate which protocols this node supports
// See 3.3.6 and 3.3.7 in http://openlcb.com/wp-content/uploads/2016/02/S-9.7.3-MessageNetwork-2016-02-06.pdf
uint8_t protocolIdentValue[6] = {   //0xD7,0x58,0x00,0,0,0};
        pSimple | pDatagram | pMemConfig | pPCEvents | !pIdent    | pTeach     | !pStream   | !pReservation, // 1st byte
        pACDI   | pSNIP     | pCDI       | !pRemote  | !pDisplay  | !pTraction | !pFunction | !pDCC        , // 2nd byte
        0, 0, 0, 0                                                                                           // remaining 4 bytes
    };

#define OLCB_NO_BLUE_GOLD

/**
 * userState() is called when JMRI queries the state of an event index.
 */
enum evStates { VALID=4, INVALID=5, UNKNOWN=7 };
uint8_t userState(uint16_t index) {
  Serial.printf("\n%6ld In userState() for event index = 0x%02X", millis(), index);

  // Determine if a TOTI object has this event index.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    if (toti[i]->eventIndexMatches(index)) {
      // This TOTI object has this event index.
      return toti[i]->eventIndexMatchesCurrentState(index) ? VALID : INVALID;
    }
  }

  return UNKNOWN; // In case index is not recognised.
}

// ===== Process Consumer-eventIDs =====
void pceCallback(uint16_t index) {
  // Invoked when an event is consumed; drive pins as needed
  // from index of all events.
  Serial.printf("\npceCallback() called with index=0x%02X", index);
}

void userSoftReset() {}
void userHardReset() {}

// Callback from a Configuration write
// Use this to detect changes in the node's configuration
// This may be useful to take immediate action on a change.
void userConfigWritten(uint32_t address, uint16_t length, uint16_t func) {
  dPS("\nuserConfigWritten: Addr: ", (uint32_t)address); 
  dPS("  Len: ", (uint16_t)length); 
  dPS("  Func: ", (uint8_t)func);

  // Need to do an EEPROM commit as this doesn't always appear to happen!!
  EEPROM.commit();

  // Update the LED's configuration as it may have been changed.
  ledRedConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration));
  ledGreenConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration));
  ledBlueConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration));

  // Update the TOTI's delay as they may have changed.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    toti[i]->setOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].occupiedDelay)));
    toti[i]->setNotOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].notOccupiedDelay)));
  }
}

void sendInitialEvents() {
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    // eventToSend = toti[i]->getEventForCurrentState();
    // if (eventToSend != -1) OpenLcb.produce(eventToSend);
    toti[i]->sendEventsForCurrentState();
  }
}

void sendEventCallbackFunction(uint16_t eventIndexToSend) {
  if (hubConnected) {
    Serial.printf("\n%6ld sendEventCallbackFunction() called. event index=0x%02X", millis(), eventIndexToSend);
    OpenLcb.produce(eventIndexToSend);
  }
}

void initialiseRGBLEDs() {
  // Configure the built in RGB LEDs and turn them off.
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);

  // Determine how the RGB LEDs are configured.
  ledRedConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration));
  ledGreenConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration));
  ledBlueConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration));
}

void initialiseTOTIs() {
  // Store pointers to the TOTI objects in the toti array.
  toti[0] = &toti0;
  toti[1] = &toti1;
  toti[2] = &toti2;
  toti[3] = &toti3;
  toti[4] = &toti4;
  toti[5] = &toti5;

  // Initialise all TOTI objects.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    toti[i]->setEvents(TOTI_EVENT_BASE + (i*2) + 0, TOTI_EVENT_BASE + (i*2) + 1);
    toti[i]->setSendEventCallbackFunction(sendEventCallbackFunction);

    toti[i]->setOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].occupiedDelay)));
    toti[i]->setNotOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].notOccupiedDelay)));

    // toti[i]->print();
  }
}

// Was "NodeID nodeid(NODE_ADDRESS);" which was moved here for version 0.1.19.
// The actual value for Node ID is now set in setup() using data from Preferences or
// uses NODE_ADDRESS if not available in Preferences.
NodeID nodeid;

// The following #include needs nodeid to be already declared.
#include "OpenLCBMid.h"   // Essential - do not move or delete

// ==== Setup does initial configuration ======================
void setup() {
  Serial.begin(115200);

  // Delay to allow Serial port to be established.
  delay(1000);

  // temp for testing -- allows CoolTerm to be connected.
  delay(4000);

  Serial.printf("\n%6ld starting program", millis());
  Serial.printf("\n%6ld            Model: ", millis()); Serial.print(MODEL);
  Serial.printf("\n%6ld Software version: ", millis()); Serial.print(SWVERSION);
  Serial.printf("\n%6ld Compilation date: ", millis()); Serial.print(__DATE__);
  Serial.printf("\n%6ld Compilation time: ", millis()); Serial.print(__TIME__);

  // Create a ConfigurationOTA object and pass in the required parameters.
  ConfigurationOTA configurationOTA;
  configurationOTA.setCredentials(credentials); // A pointer to the credentials data in credentials.h
  configurationOTA.setTimeout(1000); // The 1000 mS timeout is used when connecting to one of potentially many WiFi hubs as not every WiFi hub may be available
  configurationOTA.setCurrentVersion(SWVERSION); // The currently running version of firmware
  configurationOTA.setDefaultNodeID(NodeID(NODE_ADDRESS)); // Used if a Node ID cannot be obtained

  // Connect to a WiFi hub, download the json configuration file and perform all configuration.
  configurationOTA.doConfiguration();

  // Update nodeid according to the Node ID stored in Preferences.
  // If there is no Node ID stored, then use the default of NODE_ADDRESS.
  nodeid = ConfigurationPreferences::getNodeID(NodeID(NODE_ADDRESS));

  // Initialise Olcb with the node id from Preferences.
  Olcb_init(nodeid, RESET_TO_FACTORY_DEFAULTS);

  // NodeID nodeid(NODE_ADDRESS);      // Moved to above the line #include "OpenLCBMid.h"
  // Olcb_init(nodeid, RESET_TO_FACTORY_DEFAULTS);

  initialiseRGBLEDs();
  initialiseTOTIs();

  Serial.printf("\n%6ld Initialisation finished", millis());
}

// ==== Loop ==========================
void loop() {
  // Do OpenLCB/LCC processing.
  Olcb_process();

  // Process any changes to TOTI state and send the appropriate event if required.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    toti[i]->loop();
  }

  /**
   * Connect to the OpenLCB/LCC hub and reconnect if contact has been lost.
   */
  if (hubConnectionMade(ConfigurationPreferences::getWiFiSSID(), ConfigurationPreferences::getWiFiPassword())) {
    ledConfigHubConnected = true; // Turn the blue LED on if configured.

    // This is required so that JMRI is initialised if JMRI starts after the node has started.
    sendInitialEvents();
  }

  if (hubConnectionLost(ConfigurationPreferences::getWiFiSSID(), ConfigurationPreferences::getWiFiPassword())) {
    ledConfigHubConnected = false; // Turn the blue LED off if configured.
  }

  /**
   * Control the LEDs.
   */
  digitalWrite(LED_RED, getLEDState(ledRedConfiguration));
  digitalWrite(LED_GREEN, getLEDState(ledGreenConfiguration));
  digitalWrite(LED_BLUE, getLEDState(ledBlueConfiguration));
}

/**
 * Returns LOW or HIGH to control one of the RGB LEDs.
 */
uint8_t getLEDState(int switchInput) {
  uint8_t ledState;

  switch (switchInput) {
    case LED_CONFIG_NOT_CONFIGURED:
      ledState = HIGH;
      break;
    case LED_CONFIG_HUB_STATE:
      ledState = ledConfigHubConnected ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_1_STATE:
      ledState = toti[0]->isOccupied() ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_2_STATE:
      ledState = toti[1]->isOccupied() ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_3_STATE:
      ledState = toti[2]->isOccupied() ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_4_STATE:
      ledState = toti[3]->isOccupied() ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_5_STATE:
      ledState = toti[4]->isOccupied() ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_6_STATE:
      ledState = toti[5]->isOccupied() ? LOW : HIGH;
      break;
  }
  
  return ledState;
}
