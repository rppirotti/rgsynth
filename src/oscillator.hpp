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

#ifndef OSCILLATOR_H
#define OSCILLATOR_H

#include "types.h"

#define  WAVEFORM_SAMPLE_SIZE       1024   ///< Number of waveform points for each sample

///////////////////////////////////////////////////////////////////////////////
/// @brief Sine waveform sample
///////////////////////////////////////////////////////////////////////////////
const int16 sineSample [ 1024 ] =
{
   0,12,25,37,50,62,75,87,100,112,125,138,150,163,175,188,200,213,225,238,250,263,275,287,300,
   312,325,337,349,362,374,387,399,411,423,436,448,460,472,485,497,509,521,533,545,558,570,582,
   594,606,618,630,642,654,665,677,689,701,713,724,736,748,760,771,783,794,806,818,829,840,852,
   863,875,886,897,909,920,931,942,953,964,976,987,998,1008,1019,1030,1041,1052,1063,1073,1084,
   1095,1105,1116,1126,1137,1147,1158,1168,1178,1188,1199,1209,1219,1229,1239,1249,1259,1269,
   1279,1288,1298,1308,1317,1327,1337,1346,1355,1365,1374,1383,1393,1402,1411,1420,1429,1438,
   1447,1456,1465,1473,1482,1491,1499,1508,1516,1525,1533,1541,1550,1558,1566,1574,1582,1590,
   1598,1605,1613,1621,1629,1636,1644,1651,1659,1666,1673,1680,1687,1695,1702,1708,1715,1722,
   1729,1736,1742,1749,1755,1762,1768,1774,1781,1787,1793,1799,1805,1811,1816,1822,1828,1834,
   1839,1845,1850,1855,1861,1866,1871,1876,1881,1886,1891,1895,1900,1905,1909,1914,1918,1923,
   1927,1931,1935,1939,1943,1947,1951,1955,1958,1962,1966,1969,1972,1976,1979,1982,1985,1988,
   1991,1994,1997,1999,2002,2005,2007,2010,2012,2014,2016,2018,2021,2022,2024,2026,2028,2030,
   2031,2033,2034,2035,2037,2038,2039,2040,2041,2042,2043,2043,2044,2045,2045,2046,2046,2046,
   2046,2046,2047,2046,2046,2046,2046,2046,2045,2045,2044,2043,2043,2042,2041,2040,2039,2038,
   2037,2035,2034,2033,2031,2030,2028,2026,2024,2022,2021,2018,2016,2014,2012,2010,2007,2005,
   2002,1999,1997,1994,1991,1988,1985,1982,1979,1976,1972,1969,1966,1962,1958,1955,1951,1947,
   1943,1939,1935,1931,1927,1923,1918,1914,1909,1905,1900,1895,1891,1886,1881,1876,1871,1866,
   1861,1855,1850,1845,1839,1834,1828,1822,1816,1811,1805,1799,1793,1787,1781,1774,1768,1762,
   1755,1749,1742,1736,1729,1722,1715,1708,1702,1695,1687,1680,1673,1666,1659,1651,1644,1636,
   1629,1621,1613,1605,1598,1590,1582,1574,1566,1558,1550,1541,1533,1525,1516,1508,1499,1491,
   1482,1473,1465,1456,1447,1438,1429,1420,1411,1402,1393,1383,1374,1365,1355,1346,1337,1327,
   1317,1308,1298,1288,1279,1269,1259,1249,1239,1229,1219,1209,1199,1188,1178,1168,1158,1147,
   1137,1126,1116,1105,1095,1084,1073,1063,1052,1041,1030,1019,1008,998,987,976,964,953,942,
   931,920,909,897,886,875,863,852,840,829,818,806,794,783,771,760,748,736,724,713,701,689,
   677,665,654,642,630,618,606,594,582,570,558,545,533,521,509,497,485,472,460,448,436,423,
   411,399,387,374,362,349,337,325,312,300,287,275,263,250,238,225,213,200,188,175,163,150,
   138,125,112,100,87,75,62,50,37,25,12,0,-12,-25,-37,-50,-62,-75,-87,-100,-112,-125,-138,-150,
   -163,-175,-188,-200,-213,-225,-238,-250,-263,-275,-287,-300,-312,-325,-337,-349,-362,-374,
   -387,-399,-411,-423,-436,-448,-460,-472,-485,-497,-509,-521,-533,-545,-558,-570,-582,-594,
   -606,-618,-630,-642,-654,-665,-677,-689,-701,-713,-724,-736,-748,-760,-771,-783,-794,-806,
   -818,-829,-840,-852,-863,-875,-886,-897,-909,-920,-931,-942,-953,-964,-976,-987,-998,-1008,
   -1019,-1030,-1041,-1052,-1063,-1073,-1084,-1095,-1105,-1116,-1126,-1137,-1147,-1158,-1168,
   -1178,-1188,-1199,-1209,-1219,-1229,-1239,-1249,-1259,-1269,-1279,-1288,-1298,-1308,-1317,
   -1327,-1337,-1346,-1355,-1365,-1374,-1383,-1393,-1402,-1411,-1420,-1429,-1438,-1447,-1456,
   -1465,-1473,-1482,-1491,-1499,-1508,-1516,-1525,-1533,-1541,-1550,-1558,-1566,-1574,-1582,
   -1590,-1598,-1605,-1613,-1621,-1629,-1636,-1644,-1651,-1659,-1666,-1673,-1680,-1687,-1695,
   -1702,-1708,-1715,-1722,-1729,-1736,-1742,-1749,-1755,-1762,-1768,-1774,-1781,-1787,-1793,
   -1799,-1805,-1811,-1816,-1822,-1828,-1834,-1839,-1845,-1850,-1855,-1861,-1866,-1871,-1876,
   -1881,-1886,-1891,-1895,-1900,-1905,-1909,-1914,-1918,-1923,-1927,-1931,-1935,-1939,-1943,
   -1947,-1951,-1955,-1958,-1962,-1966,-1969,-1972,-1976,-1979,-1982,-1985,-1988,-1991,-1994,
   -1997,-1999,-2002,-2005,-2007,-2010,-2012,-2014,-2016,-2018,-2021,-2022,-2024,-2026,-2028,
   -2030,-2031,-2033,-2034,-2035,-2037,-2038,-2039,-2040,-2041,-2042,-2043,-2043,-2044,-2045,
   -2045,-2046,-2046,-2046,-2046,-2046,-2047,-2046,-2046,-2046,-2046,-2046,-2045,-2045,-2044,
   -2043,-2043,-2042,-2041,-2040,-2039,-2038,-2037,-2035,-2034,-2033,-2031,-2030,-2028,-2026,
   -2024,-2022,-2021,-2018,-2016,-2014,-2012,-2010,-2007,-2005,-2002,-1999,-1997,-1994,-1991,
   -1988,-1985,-1982,-1979,-1976,-1972,-1969,-1966,-1962,-1958,-1955,-1951,-1947,-1943,-1939,
   -1935,-1931,-1927,-1923,-1918,-1914,-1909,-1905,-1900,-1895,-1891,-1886,-1881,-1876,-1871,
   -1866,-1861,-1855,-1850,-1845,-1839,-1834,-1828,-1822,-1816,-1811,-1805,-1799,-1793,-1787,
   -1781,-1774,-1768,-1762,-1755,-1749,-1742,-1736,-1729,-1722,-1715,-1708,-1702,-1695,-1687,
   -1680,-1673,-1666,-1659,-1651,-1644,-1636,-1629,-1621,-1613,-1605,-1598,-1590,-1582,-1574,
   -1566,-1558,-1550,-1541,-1533,-1525,-1516,-1508,-1499,-1491,-1482,-1473,-1465,-1456,-1447,
   -1438,-1429,-1420,-1411,-1402,-1393,-1383,-1374,-1365,-1355,-1346,-1337,-1327,-1317,-1308,
   -1298,-1288,-1279,-1269,-1259,-1249,-1239,-1229,-1219,-1209,-1199,-1188,-1178,-1168,-1158,
   -1147,-1137,-1126,-1116,-1105,-1095,-1084,-1073,-1063,-1052,-1041,-1030,-1019,-1008,-998,
   -987,-976,-964,-953,-942,-931,-920,-909,-897,-886,-875,-863,-852,-840,-829,-818,-806,-794,
   -783,-771,-760,-748,-736,-724,-713,-701,-689,-677,-665,-654,-642,-630,-618,-606,-594,-582,
   -570,-558,-545,-533,-521,-509,-497,-485,-472,-460,-448,-436,-423,-411,-399,-387,-374,-362,
   -349,-337,-325,-312,-300,-287,-275,-263,-250,-238,-225,-213,-200,-188,-175,-163,-150,-138,
   -125,-112,-100,-87,-75,-62,-50,-37,-25,-12
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Implements oscillator module
///////////////////////////////////////////////////////////////////////////////
class Oscillator
{
public:

   ///////////////////////////////////////////////////////////////////////////////
   /// @brief Supported waveform types
   ///////////////////////////////////////////////////////////////////////////////
   typedef enum
   {
      eSQUARE_WAVE,
      eSAWTOOTH_WAVE,
      eSINE_WAVE
   } EnOscilatorMode;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Ctor
   /// @param oscilatorMode  oscillator waveform type
   ////////////////////////////////////////////////////////////////////////////
   Oscillator( EnOscilatorMode oscilatorMode = eSQUARE_WAVE)
   : m_currentPhaseIncrement  ( 0 )
   , m_samplePhase     ( 0 )
   , m_pWaveformSample ( 0 )
   , m_portamentoTime ( 0 )
   , m_sweepPhaseIncrement ( 0 )
   , m_output          ( 0 )
   , m_frequencyMultipler ( 65536 )
   , m_fineTunning     ( 65536 )
   , m_octave ( 0 )
   , m_oscilatorMode ( oscilatorMode )
   {
      updateWaveformSample();
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Implement oscillator waveform generation.
   /// @details Implement oscillator waveform generation. Must be called
   //           in the sample rate period.
   /// @param  None.
   /// @return Current audio value.
   ////////////////////////////////////////////////////////////////////////////
   inline int16 process ()
   {
      uint64   freqMulti = (uint64)((uint64)m_currentPhaseIncrement * m_frequencyMultipler);
      freqMulti = (uint32)(freqMulti >> 16);
      m_samplePhase += freqMulti;
      m_samplePhase &= ((WAVEFORM_SAMPLE_SIZE << 16) - 1);

      switch ( m_oscilatorMode )
      {
         case eSINE_WAVE:
            m_output = m_pWaveformSample[m_samplePhase >> 16];
            break;

         case eSQUARE_WAVE:
            if ( m_samplePhase < ( (WAVEFORM_SAMPLE_SIZE/2) << 16) )
            {
               m_output = 2047;
            }
            else
            {
               m_output = -2047;
            }
            break;

         case eSAWTOOTH_WAVE:
            m_output = (int16)(m_samplePhase >> 14) - (int16)2048;
            break;
      }

      int32 t  = (m_samplePhase << 4) / 1024;    // [12.20]=[12.20]/[11.0]
      int32 dt = (m_currentPhaseIncrement << 4) / 1024; // [12.20]=[12.20]/[11.0]

      if ( eSQUARE_WAVE == m_oscilatorMode )
      {
         m_output += (int64)((polyBlep ( t, dt ) >> 12) * (int64)2047) >> 8;
         t = t + (1 << 19);      // + 0.5
         if ( t >= (1 << 20) )   // if ( t >= 1)
         {
            t = t - (1 << 20);   // t = t - 1;
         }
         m_output -= (int64)((polyBlep (t, dt)   >> 12) * (int64)2047) >> 8;
      }
      else if ( eSAWTOOTH_WAVE == m_oscilatorMode )
      {
         m_output -= (int64)((polyBlep (t, dt)   >> 12) * (int64)2048) >> 8;
      }

      // Check if the delta to the target phase increment is higher than each increment step
      // If it is, add to the accumulator, otherwise, force the desired value.
      if ( abs((int32)m_currentPhaseIncrement - (int32)m_targetPhaseIncrement)  > abs(m_sweepPhaseIncrement) )
      {
         m_currentPhaseIncrement = m_currentPhaseIncrement + m_sweepPhaseIncrement;
      }
      else
      {
         m_currentPhaseIncrement = m_targetPhaseIncrement;
      }

      return m_output;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Configure portamento time
   /// @param portamentoTime  Time of portamento in units of sample rate interval
   ////////////////////////////////////////////////////////////////////////////
   void SetPortamento ( uint32 portamentoTime )
   {
      m_portamentoTime = portamentoTime;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Set a frequency multiplier value
   /// @param frequencyMultiplier  Multiplier value, 16.16 fixed point format
   ////////////////////////////////////////////////////////////////////////////
   inline void SetFrequencyMultiplier ( int32 frequencyMultiplier )
   {
      m_frequencyMultipler = frequencyMultiplier;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Select waveform type
   ////////////////////////////////////////////////////////////////////////////
   void SetOscilatorMode ( EnOscilatorMode newOscilatorMode )
   {
      m_oscilatorMode = newOscilatorMode;
      updateWaveformSample();
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Set the new phase increment value - frequency of the oscillator
   ////////////////////////////////////////////////////////////////////////////
   void SetPhaseIncrement ( uint32 phaseIncrement )
   {
      // Apply fine tunning
      uint32 targetPhaseIncrement = (uint64)((uint64)phaseIncrement * (uint64)m_fineTunning) >> 16;

      // Apply octave changes
      if ( m_octave > 0 )
      {
         targetPhaseIncrement = targetPhaseIncrement << m_octave;
      }
      else if ( m_octave < 0 )
      {
         targetPhaseIncrement = targetPhaseIncrement >> abs(m_octave);
      }

      if ( 0 == m_portamentoTime )
      {
         noInterrupts();
         m_currentPhaseIncrement = targetPhaseIncrement;
         m_targetPhaseIncrement  = targetPhaseIncrement;
         m_sweepPhaseIncrement   = 0;
         interrupts();
      }
      else
      {
         noInterrupts();
         m_targetPhaseIncrement  = targetPhaseIncrement;
         m_sweepPhaseIncrement   = ( (int32)targetPhaseIncrement - (int32)m_currentPhaseIncrement ) / (int32)m_portamentoTime + 1;
         interrupts();
      }
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Ctor
   ////////////////////////////////////////////////////////////////////////////
   inline int16 getOutput ()
   {
      return m_output;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Ctor
   ////////////////////////////////////////////////////////////////////////////
   void  SetPulseWidth (uint8 pulseWidth )
   {

   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Ctor
   ////////////////////////////////////////////////////////////////////////////
   void SetFineTunning ( int32 fineTunning )
   {
      m_fineTunning = fineTunning;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Ctor
   ////////////////////////////////////////////////////////////////////////////
   void  SetOctave ( int8 octave )
   {
      m_octave = octave;
   }

private:

   ///////////////////////////////////////////////////////////////////////////////
   /// @brief
   ///////////////////////////////////////////////////////////////////////////////
   void  updateWaveformSample ()
   {
      // We are using the trivial oscillator for square and sawtooth, so we
      // need only to set the wavetable for the sine wave for now.
      switch ( m_oscilatorMode )
      {
         case eSINE_WAVE:
            m_pWaveformSample = &sineSample[0];
            break;
         case eSQUARE_WAVE:
            break;
         case eSAWTOOTH_WAVE:
            break;
      }
   }

   ///////////////////////////////////////////////////////////////////////////////
   /// @brief PolyBLEP algorithm implementation in 32bits fixed format
   ///////////////////////////////////////////////////////////////////////////////
   inline int32 polyBlep(int32 t, int32 dt)
   {
      //  t = [12.20] - sample phase / 1024
      // dt = [12.20] - phase increment / 1024
      // 0 <= t < 1
      if (t < dt)
      {
         t = (t << 10) / dt;    // [12.20]/[12.20] --> [22.10]=[2.30]/[12.20]
         return ((t + t) << 10) // [12.20]
               - t*t            // [12.20]=[22.10]*[22.10]
               - (1 << 20);     // [1.20]
               // return 12.20
      }
      // -1 < t < 0
      else if (t > ((1 << 20) - dt))
      {
         t = ( (t << 10) - (1 << 30) ) / dt;    // [12.20]/[12.20] --> [22.10]=[2.30]/[12.20]
         return t*t  // [12.20]=[22.10]*[22.10]
               +  ((t + t + (1 << 10)) << 10); // 12.20
      }
      // 0 otherwise
      return 0;
   }

   uint32   m_currentPhaseIncrement;
   uint32   m_targetPhaseIncrement;
   int32    m_sweepPhaseIncrement;
   uint32   m_portamentoTime;
   uint32   m_samplePhase;
   int8     m_octave;
   const int16*   m_pWaveformSample;
   int16    m_output;
   int32    m_frequencyMultipler;      // Allow frequency modulation
   uint32   m_fineTunning;
   EnOscilatorMode   m_oscilatorMode;
};

#endif











