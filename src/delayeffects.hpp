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

#include "types.h"

///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
class DelayWithFeedback
{
public:

   static const   int32 s_HighFeedbackGain = 0x0000E000;
   static const   int32 s_LowFeedbackGain  = 0x00009000;

   DelayWithFeedback()
   : m_feedbackGain     (s_LowFeedbackGain)
   , m_delayInputIndex  ( 12000 )
   , m_delayOutputIndex ( 0 )
   {
      memset (&m_delaySamples[0], 0, sizeof(m_delaySamples));
   }

   ////////////////////////////////////////////////////////////////////////////
   ///
   ////////////////////////////////////////////////////////////////////////////
   inline int16 process (int16 input)
   {
      int64 feedback   = m_delaySamples [ m_delayOutputIndex ] * m_feedbackGain;
      int32 delayInput = ( input >> 2)  + (feedback >> 16);
      m_delaySamples [ m_delayInputIndex ] = (delayInput);

      if ( ++m_delayInputIndex > 23999 )
      {
         m_delayInputIndex = 0;
      }
      if ( ++m_delayOutputIndex > 23999 )
      {
         m_delayOutputIndex = 0;
      }

      return m_delaySamples [ m_delayOutputIndex ];
   }

   ////////////////////////////////////////////////////////////////////////////
   ///
   ////////////////////////////////////////////////////////////////////////////
   void SetFeedbackGain(int32 feedbackGain)
   {
      m_feedbackGain = feedbackGain;
   }

   void SetWetValue ( uint8 wetValue )
   {

   }

private:
   int16    m_delaySamples [ 24000 ];
   int32    m_feedbackGain;
   uint16   m_delayInputIndex;
   uint16   m_delayOutputIndex;
};




///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
class RotarySpeaker
{
public:

   static const   uint16 sm_delayLineSize     = 128;
   static const   uint16 sm_delayLineLengthMask = (sm_delayLineSize - 1);
   static const   uint16 sm_defaultLength     = 16;
   static const   uint16 sm_defaultPhase      = 8;

   RotarySpeaker()
   : m_writePtr ( 0 )
   , m_phase ( sm_defaultPhase )
   , m_delayLineLength ( sm_defaultLength )
   , m_leftChannel ( 0 )
   , m_rightChannel ( 0 )
   {
      memset (&m_delayLine[0], 0, sizeof(m_delayLine));
      m_readPtr       = (m_writePtr - sm_defaultLength) & sm_delayLineLengthMask;
      m_readPhasedPtr = (m_writePtr - sm_defaultLength + m_phase) & sm_delayLineLengthMask;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   /// @return LEFT channel value
   ////////////////////////////////////////////////////////////////////////////
   inline int16 process (int16 input, uint32 modulation)
   {
      int16    rightChannel, leftChannel = 0;
      uint32   amplitudeMod        = (1 << 16) + modulation;
      uint32   amplitudeModShifted = (1 << 16) - modulation;

      m_delayLine [ m_writePtr ] = input >> 1;

      m_readPtr = ( (int16)m_writePtr
                  - ( ( (uint32)m_delayLineLength * (uint32)modulation ) >> 16 ) )
                  & sm_delayLineLengthMask;

      leftChannel = m_delayLine [ m_readPtr ];
      leftChannel = (leftChannel * amplitudeMod) >> 16;

      m_readPhasedPtr = ( (int16)m_writePtr
                        - ( (uint32)m_delayLineLength * (uint32)(( (~modulation + 1) & 0x1FFFF ) >> 16 ) ) )
                        & sm_delayLineLengthMask;
      m_readPhasedPtr = m_readPtr;

      rightChannel    = m_delayLine [ m_readPhasedPtr ];
      rightChannel    = (rightChannel * amplitudeModShifted) >> 16;

      m_leftChannel     = leftChannel + ( (uint32)rightChannel * (uint32)45875 >> 16);
      m_rightChannel    = rightChannel + ( (uint32)leftChannel * (uint32)45875 >> 16);

      m_writePtr++;
      m_writePtr = m_writePtr & sm_delayLineLengthMask;

      return m_leftChannel;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   /// @return RIGHT channel value
   ////////////////////////////////////////////////////////////////////////////
   inline int16 getRightChannel ()
   {
      return m_rightChannel;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetDelayLineLength ( uint16 length )
   {
      m_delayLineLength = length;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetWetValue ( uint8 wetValue )
   {

   }

   uint16 getReadPtr()
   {
      return m_readPtr;
   }
   uint16 getWritePtr()
   {
      return m_writePtr;
   }

private:
   int16    m_delayLine [ sm_delayLineSize ];
   uint16   m_writePtr;
   uint16   m_readPtr;
   uint16   m_readPhasedPtr;
   uint16   m_phase;
   int16    m_leftChannel;
   int16    m_rightChannel;
   uint16   m_delayLineLength;
};







