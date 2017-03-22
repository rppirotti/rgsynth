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

#ifndef ENVELOPE_GENERATOR_H
#define ENVELOPE_GENERATOR_H

#include "types.h"

///////////////////////////////////////////////////////////////////////////////
/// @class EnvelopeGenerator
/// @brief This class implements an envelope generator
/// @details Multiple instances of this class can be instantiated for each
///          envelope generator module we want to have
///////////////////////////////////////////////////////////////////////////////
class EnvelopeGenerator
{
public:

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   typedef enum
   {
      eOFF,
      eATTACK,
      eDECAY,
      eSUSTAIN,
      eRELEASE
   } EnEnvelolpeGeneratorState;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   typedef enum
   {
      eMOOG_MODE,
      eARP_MODE
   } EnTriggerMode;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Ctor
   ////////////////////////////////////////////////////////////////////////////
   EnvelopeGenerator()
   : m_currentState     ( eOFF )
   , m_attack   ( 0 )
   , m_decay    ( 0 )
   , m_sustain     ( 65536 )
   , m_release  ( 0 )
   , m_attackIncrement  ( 0 )
   , m_decayIncrement   ( 0 )
   , m_releaseIncrement ( 0 )
   , m_output           ( 0 )
   , m_elapsedTime      ( 0 )
   , m_triggerMode       ( eMOOG_MODE )
   , m_previousGateState ( false )
   {
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Main method to get current EG output multipliter value
   ////////////////////////////////////////////////////////////////////////////
   inline int32   getOutput ()
   {
      return m_output;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief This function should be called periodically to process EG
   /// @TODO: need to check the need to protect some lines of code to be
   ///        interrupted by the other timer (SampleRate Timer)
   ////////////////////////////////////////////////////////////////////////////
   void Process ()
   {
      switch ( m_currentState )
      {
         case eOFF:
         {
            m_output = 0;
            m_elapsedTime = 0;
         }
         break;

         case eATTACK:
         {
            m_output += m_attackIncrement;
            if ( m_output >= MAX_OUTPUT_VALUE )
            {
               m_output = MAX_OUTPUT_VALUE;
               m_currentState = eDECAY;
            }
         }
         break;

         case eDECAY:
         {
            m_output -= m_decayIncrement;
            if ( m_output <= (int32)m_sustain )
            {
               m_output = m_sustain;
               m_currentState = eSUSTAIN;
            }
         }
         break;

         case eSUSTAIN:
         {
#ifdef WIN32
            static int releaseSimulationTime = 0;
            if (releaseSimulationTime++ >= 300)
            {
               m_currentState = eRELEASE;
               releaseSimulationTime = 0;
            }
#endif
         }
         break;

         case eRELEASE:
         {
            noInterrupts();
            m_output -= m_releaseIncrement;
            if ( m_output <= 0 )
            {
               m_output = 0;
               m_currentState = eOFF;
            }
            interrupts();
         }
         break;
      }

   }

   ////////////////////////////////////////////////////////////////////////////
   /// @Setters
   ////////////////////////////////////////////////////////////////////////////
   void SetAttack(uint32 attack)
   {
      m_attack  = attack;
      UpdateAttackIncrement();
   }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetDecay(uint32 decay)
   {
      m_decay   = decay;
      UpdateDecayIncrement();
   }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetRelease(uint32 release)
   {
      m_release = release;
      UpdateReleaseIncrement();
   }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetSustain(uint32 sustain)
   {
      m_sustain  = sustain;
      UpdateDecayIncrement();
      UpdateReleaseIncrement();
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetTriggerMode ( EnTriggerMode triggerMode )
   {
      m_triggerMode = triggerMode;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetGate ( bool gate )
   {
      if ( true == gate && false == m_previousGateState )
      {
         GoToAttackState();
         m_previousGateState = true;
      }
      else if ( false == gate && true == m_previousGateState )
      {
         GotToReleaseState();
         m_previousGateState = false;
      }
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetTrigger( bool trigger )
   {
      if ( eMOOG_MODE == m_triggerMode && true == trigger )
      {
         GoToAttackState();
      }
   }

private:

   static const int32 MAX_OUTPUT_VALUE = 0x10000;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void  GoToAttackState()
   {
      noInterrupts();
      m_currentState = eATTACK;
      m_output = 0;
      interrupts();
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void GotToReleaseState()
   {
      m_currentState = eRELEASE;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void UpdateAttackIncrement ()
   {
      m_attackIncrement  = MAX_OUTPUT_VALUE / (m_attack + 1);
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void UpdateDecayIncrement ()
   {
      m_decayIncrement  = (MAX_OUTPUT_VALUE - m_sustain) / (m_decay + 1);
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void UpdateReleaseIncrement ()
   {
      m_releaseIncrement = (m_sustain) / (m_release + 1);
   }


   EnEnvelolpeGeneratorState   m_currentState;
   uint32      m_attack;
   uint32      m_decay;
   uint32      m_sustain;
   uint32      m_release;

   int32      m_attackIncrement;
   int32      m_decayIncrement;
   int32      m_releaseIncrement;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief EG output multiplier value
   /// @details EG output multiplier value. Fixed point format.
   ///          INTEGER (16 bits) + FRAC (16 bits)
   ////////////////////////////////////////////////////////////////////////////
   int32       m_output;
   ////////////////////////////////////////////////////////////////////////////
   /// @brief Elapsed time of each EG state machine
   ////////////////////////////////////////////////////////////////////////////
   uint32      m_elapsedTime;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   EnTriggerMode        m_triggerMode;

   bool        m_previousGateState;

};

#endif











