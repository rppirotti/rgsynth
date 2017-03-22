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

#ifndef HAMMOND_FUNCTIONS_H
#define HAMMOND_FUNCTIONS_H

#define  HAMMOND_DRAWBARS  9
#define  HAMMOND_GEARS     12
#define  HAMMOND_WHEELS    96
#define  HAMMOND_LFOS      2

#define  DRAWBAR_16        A11
#define  DRAWBAR_5_13      A1
#define  DRAWBAR_8         A2
#define  DRAWBAR_4         A3

#define  DRAWBAR_2_23      A4
#define  DRAWBAR_2         A5

#define  DRAWBAR_1_35      A6
#define  DRAWBAR_1_13      A10
#define  DRAWBAR_1         A8

static   uint32   hammondSamplePhase      [ HAMMOND_GEARS      ] = { 0 };
static   int32    hammondWheelIntensity   [ LAST_MIDI_NOTE + 1 ] = { 0 };
static   uint8    hammondDrawBarValue     [ HAMMOND_DRAWBARS   ] = { 0 };
static   int8     hammondDrawbarsHarmonic [ HAMMOND_DRAWBARS   ] = { -12, 7, 0, 12, 19, 24, 28, 31, 36};

extern   NoteController* pNoteController;

///////////////////////////////////////////////////////////////////////////////
/// @brief Sine waveform sample used for additive synthesis module
///////////////////////////////////////////////////////////////////////////////
int16 hammondSineSample [ 1024 ] = { 0 };

///////////////////////////////////////////////////////////////////////////////
/// @brief LFO calculated for 2Hz and 6Hz, based on the 1Khz timer.
///////////////////////////////////////////////////////////////////////////////
static   uint32   hammondLFOPhaseIncrement   [ HAMMOND_LFOS ]  = { (2 << 16) + 36700, (7 << 16) + 11010 };
static   uint32   hammondLFOSamplePhase = 0;
static   int32    hammondLFO    = 0;
static   int32    hammondLFO_90 = 0;
static   uint32   rotarySpeaker0 = 0;
static   uint32   rotarySpeaker90 = 0;

void UpdateDrawbarsValue ()
{
#ifndef WIN32
   hammondDrawBarValue [ 0 ] = analogRead ( DRAWBAR_16   ) >> 8;
   hammondDrawBarValue [ 1 ] = analogRead ( DRAWBAR_5_13 ) >> 8;
   hammondDrawBarValue [ 2 ] = analogRead ( DRAWBAR_8    ) >> 8;
   hammondDrawBarValue [ 3 ] = analogRead ( DRAWBAR_4    ) >> 8;
   hammondDrawBarValue [ 4 ] = analogRead ( DRAWBAR_2_23 ) >> 8;
   hammondDrawBarValue [ 5 ] = analogRead ( DRAWBAR_2    ) >> 8;
   hammondDrawBarValue [ 6 ] = analogRead ( DRAWBAR_1_35 ) >> 8;
   hammondDrawBarValue [ 7 ] = analogRead ( DRAWBAR_1_13 ) >> 8;
   hammondDrawBarValue [ 8 ] = analogRead ( DRAWBAR_1    ) >> 8;
#else
   hammondDrawBarValue[0] = 0;
   hammondDrawBarValue[1] = 0;
   hammondDrawBarValue[2] = 16;
   hammondDrawBarValue[3] = 0;
   hammondDrawBarValue[4] = 8;
   hammondDrawBarValue[5] = 4;
   hammondDrawBarValue[6] = 0;
   hammondDrawBarValue[7] = 0;
   hammondDrawBarValue[8] = 0;
#endif
}

void ProcessNotesHammond( )
{
   static uint32 temporary [ LAST_MIDI_NOTE + 1 ];
   memset (&temporary[0], 0, sizeof(temporary));
   bool  isLowestFound = false;

   for (int8 noteIndex = pNoteController->getLowestSupportedNote(); noteIndex < pNoteController->getHighestSupportedNote(); noteIndex++ )
   {
      if ( NoteController::sm_NoPressedNote != pNoteController->operator ()( noteIndex ) )
      {
         for ( int drawbarIndex = 0; drawbarIndex < HAMMOND_DRAWBARS; drawbarIndex++ )
         {
            temporary [ (int8)((int8)noteIndex + (int8)hammondDrawbarsHarmonic [ drawbarIndex ]) ] += hammondDrawBarValue [ drawbarIndex ];
            if ( false == isLowestFound )
            {
               isLowestFound = true;
              // temporary [ (int8)((int8)noteIndex - 12 + (int8)hammondDrawbarsHarmonic [ drawbarIndex ]) ] += hammondDrawBarValue [ drawbarIndex ];
            }
         }
      }
   }
   noInterrupts();
   // TODO: copy only used notes
   memcpy (&hammondWheelIntensity[FIRST_OSCILLATOR], &temporary[FIRST_OSCILLATOR], (LAST_MIDI_NOTE + 1 - FIRST_OSCILLATOR) * 4);
   interrupts();
}


#endif







