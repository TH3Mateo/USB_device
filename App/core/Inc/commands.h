/**
 * @file commands.h
 * @brief USB command codes for controlling and querying device states.
 * 
 * @details
 * This header defines the USB command codes used for communication
 * with the thermal control device, including commands to get or set
 * temperature, heater state, LED state, and error codes.
 * 
 * @authors Mateusz Turycz and Aleksander Uliczny
 * @date 2025-05-21
 */

//
// Created by Mateusz Turycz and Aleksander Uliczny on 2025-05-21.
//

#ifndef USB_COMMANDS_H
#define USB_COMMANDS_H


#define REQUEST_CURRENT_TEMPERATURE 0x01
#define SET_TARGET_TEMPERATURE 0x02
#define REQUEST_CURRENT_HEATER_STATE 0x05
#define SET_HEATER_STATE 0x06
#define REQUEST_CURRENT_LED_STATE 0x07
#define SET_LED_STATE 0x08

#define DISTANCE_ERROR 0x11



#endif //USB_COMMANDS_H
