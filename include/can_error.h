/**
 * @file can_error.h
 * @brief CAN Driver error / return codes
 */

#ifndef CAN_ERROR_H
#define CAN_ERROR_H

#define CAN_OK                  0
#define CAN_ERR_INVALID_PARAM  -1
#define CAN_ERR_TIMEOUT        -2
#define CAN_ERR_BUSY           -3
#define CAN_ERR_BUFFER_FULL    -4
#define CAN_ERR_NO_DATA        -5
#define CAN_ERR_BUS_OFF        -6
#define CAN_ERR_NOT_INIT       -7
#define CAN_ERR_FILTER_FULL    -8
#define CAN_ERR_NOT_SUPPORTED  -9

#endif /* CAN_ERROR_H */
