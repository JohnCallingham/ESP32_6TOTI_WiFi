# ESP32_6TOTI_WiFi

This is a program to create an OpenLCB/LCC node. It was developed using PlatformIO to run on an Arduino Nano ESP32. The node is designed to connect over WiFi to the LCC hub provided by JMRI.

## General functionality

1. Provides TOTI (Train On Track Indication) functionality for six TOTIs.
2. Allows the onboard LED to be configured to indicate various events.
3. Allows for remote configuration and remote software updates.

## Detailed functionality

1. Send produced events when each TOTI becomes occupied or not occupied.
2. Allows user configurable delays for occupied and not occupied.
3. Send initial states when connecting to the LCC hub to initialise JMRI.
4. Respond to queries from JMRI.
5. Provides 6 TOTIs.
6. Allows each colour of the on board RGB LED to be user configured to indicate one of;-
    - the state of the connection to the JMRI hub
    - the state of each of the six TOTIs

## Software components
This software uses the following components;-
- OpenLCB_Single_Thread. See https://github.com/openlcb/OpenLCB_Single_Thread
- ESP32WiFiGC. See https://github.com/JohnCallingham/ESP32WiFiGC
- LCC_TOTI. See https://github.com/JohnCallingham/LCC_TOTI
- LCC_CONFIGURATION. See https://github.com/JohnCallingham/LCC_CONFIGURATION
