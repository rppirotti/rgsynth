/*
 * Copyright 2016 Rodolfo Pirotti
 *
 * This file is part of RGSynth.
 *
 * RGSynth is free software: you can redistribute it and/or modify
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
 */

#ifndef TYPES_H
#define TYPES_H

/// Some help for Arduino DUE data types
//sizeof(byte)=1
//sizeof(char)=1
//sizeof(short)=2
//sizeof(int)=4
//sizeof(long)=4
//sizeof(long long)=8
//sizeof(bool)=1
//sizeof(boolean)=1
//sizeof(float)=4
//sizeof(double)=8
//sizeof(char*)=4
//sizeof(int*)=4
//sizeof(long*)=4
//sizeof(float*)=4
//sizeof(double*)=4
//sizeof(void*)=4
//signed long
///
///////////////////////////////////////////////////////////////////////////////
/// @brief Data types definition
///////////////////////////////////////////////////////////////////////////////
typedef  char                 int8;
typedef  unsigned char        uint8;
typedef  short                int16;
typedef  unsigned short       uint16;
typedef  int                  int32;
typedef  unsigned int         uint32;
typedef  long long            int64;
typedef  unsigned long long   uint64;

#endif

