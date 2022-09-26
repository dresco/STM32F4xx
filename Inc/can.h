/*

  can.h - CAN bus driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2022 Jon Escombe

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _CAN_H_
#define _CAN_H_

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#include <string.h>
#include <stdio.h>
#include "canbus/canbus.h"

//#define CAN_QUEUE_RX_IN_IRQ

/*
 * Function prototypes
 */
uint8_t can_start(uint32_t);
uint8_t can_stop(void);
void    can_get(void);
uint8_t can_put(canbus_message_t);
bool    can_rx_pending(void);

#endif /* _CAN_H_ */
