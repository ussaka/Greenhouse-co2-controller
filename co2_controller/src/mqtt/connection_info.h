/*
 * connection_info.h
 *
 *  Created on: 14.12.2022
 *      Author: mikol
 */

#ifndef MQTT_CONNECTION_INFO_H_
#define MQTT_CONNECTION_INFO_H_

typedef struct {
	const char* ssid;
	const char* ssidpass;
	const char* brokerip;
	int brokerport;
} connection_info;

#endif /* MQTT_CONNECTION_INFO_H_ */
