#ifndef LTE_TYPES_H
#define LTE_TYPES_H

/**
 * @file lte_types.h
 * @brief Header file defining LTE event types, command types, and state machine states.
 *
 * This file contains enumerations and structures used for managing LTE events,
 * commands, and the state machine. It provides a standardized way to represent
 * the various states and events that can occur during LTE modem operation.
 *
 * @note This file is part of the LTE modem firmware and should be included in
 *       components that interact with the LTE driver and state machine.
 */

#include <stdint.h>

// LTE event types, command types, and state machine states are defined below. 

/* LTE event types */

typedef enum {
    LTE_EVENT_NONE = 0, // No event
    LTE_EVENT_RX_LINE,  // Modem has sent a line of text (response or URC)
    LTE_EVENT_TIMEOUT, // A command has timed out
    LTE_EVENT_ERROR   // An error has occurred in the state machine
} LteEventType_t;

/* LTE command types */

typedef enum {
    LTE_CMD_NONE = 0, // No command in progress 
    LTE_CMD_ATTACH,   // Attach to the LTE network (AT+CGATT=1)
    LTE_CMD_PDP,      //  pdp activation (AT+CGACT=1,1)
    LTE_CMD_TCP_OPEN, // TCP socket open 
    LTE_CMD_MQTT_CONNECT, // MQTT connect 
    LTE_CMD_MQTT_SUBSCRIBE // MQTT subscribe
} LteCommand_t;

/* LTE state machine states */

typedef enum {
    LTE_SM_INVALID = 0, // Invalid state (used for initialization) 
    LTE_SM_IDLE,        // Idle state 
    LTE_SM_INIT,        // Initialization state (boot sequence) 
    LTE_SM_POWER_ON,    //Modem powering state  
    LTE_SM_WAIT_BOOT,   // booting state 
    LTE_SM_CHECK_SIM,   // SIM checking state 
    LTE_SM_ATTACH,     // Network attaching state 
    LTE_SM_PDP,        // activating PDP state 
    LTE_SM_OPEN_SOCKET, // opening socket state 
    LTE_SM_TCP_CONNECT, // 
    LTE_SM_MQTT_CONNECT, // MQTT connecting state 
    LTE_SM_MQTT_SUBSCRIBE, // MQTT subscribing state 
    LTE_SM_RUN,   // RUn and ready state (modem is ready and fully initilised)

    // Recovery / retry states — must match lte_sm.c
    LTE_SM_REATTACH,         // re attaching state 
    LTE_SM_REPDP,            // re pdp state 
    LTE_SM_REOPEN_SOCKET,    // re open socket state 
    LTE_SM_MQTT_RECONNECT,   // mqtt re connecting state 
    LTE_SM_MQTT_RESUBSCRIBE, // mqtt re subscribing 
    LTE_SM_REBOOT_MODEM,     // rebooting the modem state 
    LTE_SM_ERROR             // error state 
} LteSmState_t;

/* LTE event structure */

typedef struct {
    LteEventType_t type;  // type of the even (see above) 
    const char    *line;  // response from modem 
} LteEvent_t;

#endif /* LTE_TYPES_H */
