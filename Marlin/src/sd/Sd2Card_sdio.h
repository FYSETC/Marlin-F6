/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
//#pragma once

#ifndef _SD2CARD_SDIO_H_
#define _SD2CARD_SDIO_H_

#include "../inc/MarlinConfig.h"

#if ENABLED(SDIO_SUPPORT)

bool SDIO_Init(void);
bool SDIO_ReadBlock(uint32_t block, uint8_t *dst);
bool SDIO_WriteBlock(uint32_t block, const uint8_t *src);

class Sd2Card {
  public:
    bool init(uint8_t sckRateID = 0, uint8_t chipSelectPin = 0) { return SDIO_Init(); }
    bool readBlock(uint32_t block, uint8_t *dst) { return SDIO_ReadBlock(block, dst); }
    bool writeBlock(uint32_t block, const uint8_t *src) { return SDIO_WriteBlock(block, src); }
};

#endif // SDIO_SUPPORT

#endif
