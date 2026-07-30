# ESP32_6TOTI_WiFi

This is a program to create an OpenLCB/LCC node. It was developed using PlatformIO to run on an Arduino Nano ESP32. The node is designed to connect over WiFi to the LCC hub provided by JMRI.

It is part of a group of node types which use the same codebase. The other node types are;-
- [ESP32_2Servo_2Frog_2TOTI_WiFi](https://github.com/JohnCallingham/ESP32_2Servo_2Frog_2TOTI_WiFi)
- [ESP32_4ToF_WiFi](https://github.com/JohnCallingham/ESP32_4ToF_WiFi)

## General functionality

All of the node types which share the common codebase provide the following functionality;-
- Responds to consumed events and sends produced events, depending on the specific features of the node.
- When initially connected to JMRI's LCC hub the node sends the state of all events so that JMRI knows the current state of the node.
- Responds to queries from JMRI.
- Allows the user to configure the ESP32's built in RGB LED to indicate various states of the node.
- Allows the user to start various testing cycles for the node.
- Allows for remote configuration and remote firmware updates.

## Specific functionality for this node type

- Provides TOTI (Train On Track Indication) functionality for six TOTIs.

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
- [OpenLCB_Single_Thread](https://github.com/openlcb/OpenLCB_Single_Thread)
- [ESP32WiFiGC](https://github.com/JohnCallingham/ESP32WiFiGC)
- [LCC_CONFIGURATION](https://github.com/JohnCallingham/LCC_CONFIGURATION)
- [LCC_TOTI](https://github.com/JohnCallingham/LCC_TOTI)

The following software components are dependencies of one or more of the above components;-
 - [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
 - [DEBOUNCE](https://github.com/JohnCallingham/DEBOUNCE)
 - [LCC_NODE_COMPONENT_BASE](https://github.com/JohnCallingham/LCC_NODE_COMPONENT_BASE)

The PlatformIO Library Dependency Finder handles downloading all dependencies.
