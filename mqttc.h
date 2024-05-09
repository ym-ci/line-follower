/*
 * Copyright (C) 2022 dBm Signal Dynamics Inc.
 *
 * File:            mqttc.h
 * Project:         
 * Date:            March 14, 2022
 * Framework:       Arduino
 * 
 * MQTT Client functions - based on ArduinoMQTTClient library
 * https://www.arduino.cc/reference/en/libraries/arduinomqttclient
 *
 * Hardware Configuration: CETA IoT Robot (Schematic #14-00061A, w. NANO 33 IoT) 
 *
 */

#ifndef MQTTC_H_
#define MQTTC_H_

/*** Include Files ***********************************************************/

// Publish & Subscribe topics
#define PUB_TOPIC_AIO_MONITOR_FEEDS   "YMRobotics/groups/cetaiotrobot71582" // (EDIT) Adafruit Feed Topic ID
#define SUB_FEEDS   "YMRobotics/groups/cetaiotrobot71582" // (EDIT) Adafruit Feed Topic ID

#define PUB_AIO_MONITOR_FEEDS_JSON    "YMRobotics/groups/cetaiotrobot71582/json"


/*** Macros ******************************************************************/

/*** Custom Data Types *******************************************************/
typedef struct {
    String inTopic;         // topicID for most recent received message
    String inPayload;       // message payload for most recent received message
} MQTTC_SUB_MSG_STAT;


/*** Public Function Prototypes ***********************************************/
void mqttcInitialize(void);                                 // Connect to the broker and subscribe for all notifications
void mqttcTasks(void);                                      // Run mqttc background tasks
void mqttcTx(String outTopic, String jsonOutPayload);       // Publish serialized JSON payload to a topic
int mqttcRxIsAvailable(String subTopic);                    // Check if JSON message has been received for a specific subscription topic
String mqttcRx(void);                                       // Retrieve JSON payload for deserialization

#endif /* MQTTC_H_ */
