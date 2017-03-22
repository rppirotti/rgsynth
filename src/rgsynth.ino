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

#ifdef WIN32

#include <iostream>
#include <fstream>

#define noInterrupts()
#define interrupts()
#define digitalRead( x )   false
#define analogRead( x )    0

typedef unsigned char byte;
#else

#include <DueTimer.h>
#include <MIDI.h>

MIDI_CREATE_INSTANCE (HardwareSerial, Serial1, Midi);
#endif

#define ROTTARY_SPEAKER
#define  MOOG_LPF
//#define  SIMPLE_LPF
#define  MIDI_ENABLE
//#define  INTERNAL_DAC
#define  TDA1543A
#define  KEYBED

#if defined(INTERNAL_DAC) && defined(TDA1543A)
#error "Cannot define Internal DAC and TDA1543A together."
#endif
#if defined(MOOG_LPF) && defined(SIMPLE_LPF)
#error "Cannot define two filters at the same time."
#endif

#define  ADSR_FRAC_PART_WIDTH       16
#define  SAMPLE_RATE       48000           ///< Sample output rate in Hz
#define  LAST_MIDI_NOTE    120             ///< C9

#define  FIRST_MIDI_NOTE_MOOG   24              ///< C2
#define  LAST_MIDI_NOTE_MOOG    96              ///< C8

#define  FIRST_MIDI_NOTE_HAM   36              ///< C2
#define  LAST_MIDI_NOTE_HAM    96              ///< C8

#define  NUMBER_OF_OSCILLATORS      96
#define  FIRST_OSCILLATOR     FIRST_MIDI_NOTE_MOOG

#define  DAC_ZERO_OFFSET   (2047)

#define  ADSR_POT_CTRL     36
#define  EG_MODE           37
#define  NOTE_PRIOR_SEL1   38
#define  NOTE_PRIOR_SEL2   39
#define  IS_HAMMOND        40

#define  VCO1_WAVE_SEL1    41
#define  VCO1_WAVE_SEL2    42
#define  PORTAMENTO_ENABLE 43
#define  VCO2_ENABLE       44
#define  VCO2_WAVE_SEL1    45
#define  VCO2_WAVE_SEL2    46

#define  EG_CUTOFF_ALL     47
#define  EG_CUTOFF_VCO2    48

#define  LFO1_VCO1         49

#define  DELAY_SEL1        50
#define  DELAY_SEL2        51

#define  ATTACK_POT        A11
#define  DECAY_POT         A1
#define  SUSTAIN_POT       A2
#define  RELEASE_POT       A3

#define  PORTAMENTO_VCO1   A4
#define  PORTAMENTO_VCO2   A5

#define  FREQUENCY_POT     A6
#define  RESSONANCE_POT    A10
#define  LFO1_POT          A8

#define  VCO1_TUN          A9
//#define  VCO2_TUN          A10
#define  VCO2_TUN          A7


#define TIME_PROFILING_FUNCTION_CALL( FUNCTION, DELAY ) \
         if ( delayMeasure <= DELAY ) \
         { \
            delayMeasure++; \
         } \
         else \
         { \
            if ( timeCountIndex < MEASURE_QUANTITY ) \
            { \
               noInterrupts(); \
               uint32 startTime  = micros(); \
               FUNCTION; \
               uint32 finishTime = micros(); \
               interrupts(); \
               timeCount [ timeCountIndex++ ] = finishTime - startTime; \
            } \
         } \

#define REFRESH_ANALOG_PARAMETER( VAR, PIN, LSB_CANCEL_MASK, FUNCTION, PRINT ) \
         { \
            uint16 temp = ( analogRead ( PIN ) & (uint16)~LSB_CANCEL_MASK ); \
            if ( VAR !=  temp ) \
            { \
               VAR = temp; \
               FUNCTION ( ); \
               if ( true == PRINT ) \
               { \
                  Serial.print(#VAR); \
                  Serial.print(": "); \
                  Serial.println(VAR, DEC); \
                  delay (100); \
               } \
            } \
         } \


#define READ_DIGITAL_SWITCHES_AND_EXECUTE( VAR, SWITCH01, SWITCH02, FCT1, FCT2, FCT3) \
            if ( VAR != ((digitalRead (SWITCH02) << 1) +  digitalRead (SWITCH01)) ) \
            { \
               VAR = ((digitalRead (SWITCH02) << 1) +  digitalRead (SWITCH01)); \
               switch ( VAR ) \
               { \
                  case 2: \
                     FCT1(); \
                  break; \
                  case 3: \
                     FCT2(); \
                  break; \
                  case 1: \
                     FCT3(); \
                  break; \
               } \
            } \


#include "envelopegenerator.hpp"
#include "delayeffects.hpp"
#include "oscillator.hpp"
#include "envelopegenerator.hpp"
#include "filter.hpp"
#include "notecontroller.hpp"
#include "hammondfunctions.hpp"
#include "i2s.hpp"
#include "phaseincrement.hpp"

bool     ADSRPotCtrl      = false;
uint8    ADSR_Filter_Sel  =  0;
bool     EG_Mode          = false;
uint8    vco1WaveSel      = 0;
uint8    vco2WaveSel      = 0;
bool     portamentoEnable = false;
bool     Vco2Enable       = false;
bool     Lfo1Vco1Enable   = false;
uint8    notePriotiryMode = 0;

uint8    delaySel         = 2;
uint8    vcfEgSel         = 0;

bool     lfoMix_Sel       = false;

uint16    attackPot      = 0;
uint16    decayPot       = 0;
uint16    sustainPot     = 0;
uint16    releasePot     = 0;
uint16    frequencyPot   = 0;
uint16    ressonancePot  = 0;
uint16    portamentoVCO1 = 0;
uint16    portamentoVCO2 = 0;
uint16    LFO1Pot        = 0;
uint16    VCO1Tun        = 0;
uint16    VCO2Tun        = 0;

bool     splitCalc      = true;
bool     isHammond      = false;
bool     shouldPrint    = false;

/// (1024/48000) --> converted to 16 bits frac
const    uint16   s_LFO_phaseIncrementMultiplier = 0x0576;

uint32   lfoPhaseIncrement [ 1 ] = { ( 0 << 16 ) + 6990 };

bool     isTimeToCalculateEnvelopeGenerator = false;

int64    bassOutput   = 0;
int64    trebleOutput = 0;

int64    analogOutput      = 0;
int64    analogMixedOutput = 0;
int64    analogLeftOutput  = 0;
int64    analogRightOutput = 0;

#define  MEASURE_QUANTITY  4096
#define  DELAY_MEASURE     (48000 * 10)
long     timeCount [ MEASURE_QUANTITY ];
uint32   timeCountIndex = 0;
uint32   delayMeasure = 0;
bool     resultsPrinted = false;
static   int8     timer25ms = 0;

///////////////////////////////////////////////////////////////////////////////
/// @brief Amplifier Envelope Generator instance
///////////////////////////////////////////////////////////////////////////////
EnvelopeGenerator*   pAmplifierEnvelopeGenerator   = 0;
EnvelopeGenerator*   pFilterEnveopeGenerator       = 0;
Oscillator*          pOscillator01                 = 0;
Oscillator*          pOscillator02                 = 0;
Oscillator*          pOscillatorLFO1               = 0;
DelayWithFeedback*   pDelayWithFeedback            = 0;
RotarySpeaker*       pRotarySpeaker                = 0;
NoteController*      pNoteController               = 0;
I2sJapaneseFormat*   pTda1543a                     = 0;
PhaseIncrement       phaseIncrement;                     // Not a pointer.

BaseFilter*          pFilter            = 0;

void ProcessI2sInterrupt(I2sJapaneseFormat::EnChannel channel);

///////////////////////////////////////////////////////////////////////////////
/// @brief Initializa all modules and run Hammond mode
///////////////////////////////////////////////////////////////////////////////
void InitializeHammondMode ()
{
   noInterrupts();

   memset (&hammondDrawBarValue[0]   , 0, sizeof(hammondDrawBarValue));
   memset (&hammondSamplePhase[0]    , 0, sizeof(hammondSamplePhase));
   memset (&hammondWheelIntensity[0] , 0, sizeof(hammondWheelIntensity));

   uint32   hammondLFOSamplePhase = 0;
   int32    hammondLFO = 0;
   uint32   rotarySpeaker0 = 0;
   uint32   rotarySpeaker90 = 0;

   delete ( pNoteController );
   delete ( pRotarySpeaker );

   pNoteController = new NoteController ( FIRST_MIDI_NOTE_HAM, LAST_MIDI_NOTE_HAM );
   phaseIncrement.SetSampleRate24kHz();
   pRotarySpeaker  = new RotarySpeaker;

   for (int i = 0; i < WAVEFORM_SAMPLE_SIZE; i++ )
   {
      hammondSineSample[i] = sineSample [i] >> 4;
   }

#ifdef TDA1543A
   pTda1543a->configureTransmitter( 32, 24000, I2sJapaneseFormat::eAUDIO_STEREO, I2sJapaneseFormat::eCLOCK_MODE_INTERNAL_CLOCK, &ProcessI2sInterrupt );
   pTda1543a->Start();
#endif

   interrupts();
}


///////////////////////////////////////////////////////////////////////////////
/// @brief Initializa all modules and run Virtual Analog mode
///////////////////////////////////////////////////////////////////////////////
void InitializeVirtualAnalogMode ()
{
   noInterrupts();

   delete ( pAmplifierEnvelopeGenerator );
   delete ( pFilterEnveopeGenerator );
   delete ( pOscillator01 );
   delete ( pOscillator02 );
   delete ( pDelayWithFeedback );
   delete ( pOscillatorLFO1 );
   delete ( pNoteController );
   delete ( pFilter );

   pAmplifierEnvelopeGenerator   = new EnvelopeGenerator();
   pFilterEnveopeGenerator       = new EnvelopeGenerator();
   pOscillator01                 = new Oscillator( );
   pOscillator02                 = new Oscillator( );
   pDelayWithFeedback            = new DelayWithFeedback();
   pOscillatorLFO1               = new Oscillator( Oscillator::eSINE_WAVE );
   pNoteController               = new NoteController ( FIRST_MIDI_NOTE_MOOG, LAST_MIDI_NOTE_MOOG);
   phaseIncrement.SetSampleRate48kHz();

#ifdef MOOG_LPF
   pFilter = new IntFracMoogFilter();
   pFilter->SetCutoff( 1 << 16 );
   pFilter->SetResonance( 1 );
#endif
#ifdef SIMPLE_LPF
   pFilter = new SimpleLowPassFilter ();
   pFilter->SetCutoff(1 << 16);
   pFilter->SetResonance( 1 );
#endif

   pAmplifierEnvelopeGenerator->SetAttack  (100);
   pAmplifierEnvelopeGenerator->SetDecay   (50);
   pAmplifierEnvelopeGenerator->SetSustain    (65536);
   pAmplifierEnvelopeGenerator->SetRelease (150);

   pFilterEnveopeGenerator->SetAttack  (100);
   pFilterEnveopeGenerator->SetDecay   (100);
   pFilterEnveopeGenerator->SetSustain    (65536);
   pFilterEnveopeGenerator->SetRelease (100);

#ifdef TDA1543A
   pTda1543a->configureTransmitter( 32, 48000, I2sJapaneseFormat::eAUDIO_STEREO, I2sJapaneseFormat::eCLOCK_MODE_INTERNAL_CLOCK, &ProcessI2sInterrupt );
   pTda1543a->Start();
#endif

   interrupts();
}

///////////////////////////////////////////////////////////////////////////////
/// @brief Timer0 interrupt processing - Sample Rate audio generation
///////////////////////////////////////////////////////////////////////////////
#ifdef INTERNAL_DAC
void SampleRateTimerHandler ()
{
   if ( true == LFO1Enable )
   {
      pOscillator01->SetFrequencyMultiplier( (int64)(((int64)1 << 16) - ((int64)2047 >> 2)) );
   }
   if ( 2 != LFO2Sel )
   {
      pOscillator02->SetFrequencyMultiplier( (int64)(((int64)1 << 16) - ((int64)pOscillatorLFO1->process()>> 2)) );
   }
   if ( true == lfoMix_Sel )
   {
      analogOutput = (pOscillator01->process()) + ( pOscillator02->process() >> 1 );
   }
   else
   {
      pOscillator02->process();
      analogOutput = pOscillator01->process();
   }

   analogOutput = pFilter->process(analogOutput);

   analogOutput = ( analogOutput * pAmplifierEnvelopeGenerator->getOutput() ) >> ADSR_FRAC_PART_WIDTH;
   if ( 2 != delaySel )
   {
      analogOutput =  (analogOutput >> 1) + (pDelayWithFeedback->process( analogOutput >> 1 ) >> 1 );
   }
#ifndef WIN32
#ifdef INTERNAL_DAC
   analogWrite (DAC0, (analogOutput)+ DAC_ZERO_OFFSET);
#endif
#endif

}
#endif


///////////////////////////////////////////////////////////////////////////////
/// @brief I2S driver interrupt process - Sample rate audio generation
///////////////////////////////////////////////////////////////////////////////
#ifdef TDA1543A
void ProcessI2sInterrupt ( I2sJapaneseFormat::EnChannel channel )
{
   if ( false == isHammond )
   {
      if ( true == splitCalc )
      {
         splitCalc = false;
         if ( true == Lfo1Vco1Enable )
         {
            pOscillator01->SetFrequencyMultiplier( (int64)(((int64)1 << 16) + ((int64)pOscillatorLFO1->process() >> 2))  );
         }

         pOscillator01->process();
         pOscillator02->process();

         pTda1543a->Write((int16)analogMixedOutput );
      }
      else
      {
         splitCalc = true;

         if ( 1 == vcfEgSel )
         {
            analogOutput  = pFilter->process( pOscillator01->getOutput() );
            if ( true == Vco2Enable )
            {
               analogOutput += pOscillator02->getOutput() >> 1;
            }
         }
         else
         {
            if ( true == Vco2Enable )
            {
               analogOutput = pFilter->process( (int64)pOscillator01->getOutput() + (int64)pOscillator02->getOutput() >> 1);
            }
            else
            {
              analogOutput = pFilter->process( (int64)pOscillator01->getOutput() );
            }          
         }

         analogOutput = ( analogOutput * pAmplifierEnvelopeGenerator->getOutput() ) >> ADSR_FRAC_PART_WIDTH;
         if ( 2 != delaySel )
         {
            analogOutput =  (analogOutput >> 1) + (pDelayWithFeedback->process( analogOutput >> 1 ) >> 1 );
         }
         analogMixedOutput = analogOutput;
         pTda1543a->Write( (int16)analogMixedOutput );
      }
   }
   else
   {
      if ( true == splitCalc )
      {
         splitCalc = false;

         bassOutput   = 0;
         trebleOutput = 0;
         for (uint32 i=0; i < 6; i++)
         {
#ifdef ROTTARY_SPEAKER
            hammondSamplePhase [ i ] += phaseIncrement [ FIRST_OSCILLATOR + NUMBER_OF_OSCILLATORS - HAMMOND_GEARS + i];
#else
            hammondSamplePhase [ i ] += phaseIncrement [ FIRST_OSCILLATOR + NUMBER_OF_OSCILLATORS - HAMMOND_GEARS + i] + (hammondLFO << 7);
#endif
            // Compute lowest octave (C2 = 32,7Hz)
            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 22) & (WAVEFORM_SAMPLE_SIZE - 1)  ] * hammondWheelIntensity [ FIRST_OSCILLATOR + i ];

            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 21) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 12 + FIRST_OSCILLATOR ];
            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 20) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 24 + FIRST_OSCILLATOR ];
            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 19) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 36 + FIRST_OSCILLATOR ];
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 18) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 48 + FIRST_OSCILLATOR ];
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 17) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 60 + FIRST_OSCILLATOR];
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 16) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 72 + FIRST_OSCILLATOR];

            // Compute highest octave (C9)
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 15) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 80 + FIRST_OSCILLATOR];

         }

         pTda1543a->Write( analogRightOutput );

      }
      else
      {
         splitCalc = true;
         for (uint32 i=6; i < HAMMOND_GEARS; i++)
         {
#ifdef ROTTARY_SPEAKER
            hammondSamplePhase [ i ] += phaseIncrement [ FIRST_OSCILLATOR + NUMBER_OF_OSCILLATORS - HAMMOND_GEARS + i];
#else
            hammondSamplePhase [ i ] += phaseIncrement [ FIRST_OSCILLATOR + NUMBER_OF_OSCILLATORS - HAMMOND_GEARS + i] + (hammondLFO << 7);
#endif

            // Compute lowest octave (C2)
            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 22) & (WAVEFORM_SAMPLE_SIZE - 1)  ] * hammondWheelIntensity [ i + FIRST_OSCILLATOR ];

            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 21) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 12 + FIRST_OSCILLATOR ];
            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 20) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 24 + FIRST_OSCILLATOR ];
            bassOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 19) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 36 + FIRST_OSCILLATOR ];
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 18) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 48 + FIRST_OSCILLATOR ];
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 17) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 60 + FIRST_OSCILLATOR];
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 16) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 72 + FIRST_OSCILLATOR];
            trebleOutput += hammondSineSample [ (hammondSamplePhase [ i ] >> 15) & (WAVEFORM_SAMPLE_SIZE - 1) ] * hammondWheelIntensity [ i + 80 + FIRST_OSCILLATOR];
         }

#ifdef ROTTARY_SPEAKER
         analogLeftOutput  = pRotarySpeaker->process( bassOutput + trebleOutput, (1 << 16 ) - ( (hammondLFO + 128) << 8 ));
         analogRightOutput = pRotarySpeaker->getRightChannel();
#else
         analogLeftOutput  = bassOutput + trebleOutput;
         analogRightOutput = analogLeftOutput ;
#endif
         pTda1543a->Write( analogLeftOutput );
      }
   }

}
#endif

///////////////////////////////////////////////////////////////////////////////
/// @brief Timer1 interrupt processing. Time-base for time-dependent functions.
///////////////////////////////////////////////////////////////////////////////
void ADSRTimerHandler ()
{
   isTimeToCalculateEnvelopeGenerator = true;

   if ( true == isHammond )
   {
      static bool    isSpeedingUp   = false;
      static bool    isSpeedingDown = false;
      static uint32  currentPhaseIncrement = 0;
      static uint8   lastState      = 0;

      switch ( delaySel )
      {
         case 2: // OFF
         {
            if ( 0 != lastState )
            {
//               if ( currentPhaseIncrement > hammondLFOPhaseIncrement [ 0 ] )
//               {
//                  currentPhaseIncrement -= 120;
//               }
//               else if ( currentPhaseIncrement < hammondLFOPhaseIncrement [ 0 ] && currentPhaseIncrement > 40 )
//               {
//                  currentPhaseIncrement -= 40;
//               }
//               else
//               {
//                  currentPhaseIncrement = 0;
//                  lastState = 0;
//               }
               if ( currentPhaseIncrement > 40 )
               {
                  currentPhaseIncrement -= 40;
               }
               else
               {
                  currentPhaseIncrement = 0;
                  lastState = 0;
               }

            }

         }
         break;
         case 3: // LOW
         {
            if ( 0 == lastState )
            {
               if ( currentPhaseIncrement < hammondLFOPhaseIncrement [ 0 ] )
               {
                  currentPhaseIncrement += 40;
               }
               else
               {
                  currentPhaseIncrement = hammondLFOPhaseIncrement [ 0 ];
                  lastState = 1;
               }
            }
            else if ( 2 == lastState )
            {
               if ( currentPhaseIncrement > hammondLFOPhaseIncrement [ 0 ] )
               {
                  currentPhaseIncrement -= 120;
               }
               else
               {
                  currentPhaseIncrement = hammondLFOPhaseIncrement [ 0 ];
                  lastState = 1;
               }
            }
         }
         break;
         case 1: // HIGH
         {
               if ( currentPhaseIncrement < hammondLFOPhaseIncrement [ 1 ] )
               {
                  currentPhaseIncrement += 120;
               }
               else
               {
                  currentPhaseIncrement = hammondLFOPhaseIncrement [ 1 ];
                  lastState = 2;
               }
         }
         break;
      }
      hammondLFOSamplePhase += currentPhaseIncrement;
      hammondLFOSamplePhase &= ((WAVEFORM_SAMPLE_SIZE << 16) - 1);
      hammondLFO    = hammondSineSample [ hammondLFOSamplePhase >> 16 ];
      hammondLFO_90 = hammondSineSample [ (hammondLFOSamplePhase + (WAVEFORM_SAMPLE_SIZE / 4)) >> 16 ];

   }
}

///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
void MidiHandleNoteOn  ( byte channel, byte note, byte velocity  )
{
   pNoteController->HandleNoteOn ( (int8)channel, (int8)note );
}

///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
void MidiHandleNoteOff ( byte channel, byte note, byte velocity  )
{
   pNoteController->HandleNoteOff( (int8)channel, (int8)note );
}

///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
void setup()
{
   // put your setup code here, to run once:

#if !defined(WIN32)// && !defined(linux)
   // Set analog pins to inputs
   analogReadResolution ( 12 );  // 12 bits ADC read

   pinMode(A1,  INPUT);
   pinMode(A2,  INPUT);
   pinMode(A3,  INPUT);
   pinMode(A4,  INPUT);
   pinMode(A5,  INPUT);
   pinMode(A6,  INPUT);
   pinMode(A7,  INPUT);
   pinMode(A8,  INPUT);
   pinMode(A9,  INPUT);
   pinMode(A10, INPUT);
   pinMode(A11, INPUT);

   pinMode (22, INPUT_PULLUP);
   pinMode (25, INPUT_PULLUP);
   pinMode (26, INPUT_PULLUP);
   pinMode (27, INPUT_PULLUP);
   pinMode (28, INPUT_PULLUP);
   pinMode (29, INPUT_PULLUP);
   pinMode (30, INPUT_PULLUP);
   pinMode (31, INPUT_PULLUP);
   pinMode (32, INPUT_PULLUP);
   pinMode (33, INPUT_PULLUP);
   pinMode (34, INPUT_PULLUP);
   pinMode (35, INPUT_PULLUP);
   pinMode (36, INPUT_PULLUP);
   pinMode (37, INPUT_PULLUP);
   pinMode (38, INPUT_PULLUP);
   pinMode (39, INPUT_PULLUP);
   pinMode (40, INPUT_PULLUP);
   pinMode (41, INPUT_PULLUP);
   pinMode (42, INPUT_PULLUP);
   pinMode (43, INPUT_PULLUP);
   pinMode (44, INPUT_PULLUP);
   pinMode (45, INPUT_PULLUP);
   pinMode (46, INPUT_PULLUP);
   pinMode (47, INPUT_PULLUP);
   pinMode (48, INPUT_PULLUP);
   pinMode (49, INPUT_PULLUP);
   pinMode (50, INPUT_PULLUP);
   pinMode (51, INPUT_PULLUP);

#ifdef KEYBED
   pinMode (2,  OUTPUT);  // Keyscan 00
   pinMode (3,  OUTPUT);  // Keyscan 01
   pinMode (4,  OUTPUT);  // Keyscan 02
   pinMode (5,  OUTPUT);  // Keyscan 03
   pinMode (14,  OUTPUT);  // Keyscan 04
   pinMode (15,  OUTPUT);  // Keyscan 05
   pinMode (16,  OUTPUT);  // Keyscan 06

   pinMode (6,  INPUT);   // Keyscan 07
   pinMode (7, INPUT);   // Keyscan 08
   pinMode (8, INPUT);   // Keyscan 09
   pinMode (9, INPUT);   // Keyscan 10
   pinMode (10, INPUT);   // Keyscan 11
   pinMode (11, INPUT);   // Keyscan 12
   pinMode (12, INPUT);   // Keyscan 13
   pinMode (13, INPUT);   // Keyscan 14
#endif

   // ADSR timer
   Timer1.attachInterrupt(ADSRTimerHandler).setFrequency(1000).start();

   Midi.setHandleNoteOn  ( MidiHandleNoteOn  );
   Midi.setHandleNoteOff ( MidiHandleNoteOff );
   Midi.begin(MIDI_CHANNEL_OMNI);            // Listen to all channels

   Serial.begin (115200, SERIAL_8N1);
#endif

#if defined(INTERNAL_DAC) && ( !defined(WIN32) )//&& !defined(linux) )
   // Sample Rate timer
   Timer0.attachInterrupt(SampleRateTimerHandler).setFrequency(SAMPLE_RATE).start();
   // Set DAC writes to 12 bits.
   analogWriteResolution(12);
#endif

   memset (&timeCount[0], 0, sizeof(timeCount));

#ifdef TDA1543A
   pTda1543a                  = new I2sJapaneseFormat( "Tda1543A" );
#endif   

   if ( true == isHammond )
   {
      InitializeHammondMode();
   }
   else
   {
      InitializeVirtualAnalogMode();
   }

}

void Vco2PortamentoSetup ()
{
   if ( true == portamentoEnable )
   {
      pOscillator02->SetPortamento( portamentoVCO2 << 2 );
   }
   else
   {
      pOscillator02->SetPortamento( 0 );
   }
}
void Vco1PortamentoSetup ()
{
   if ( true == portamentoEnable )
   {
      pOscillator01->SetPortamento( portamentoVCO1 << 2 );
   }
   else
   {
      pOscillator01->SetPortamento( 0 );
   }
}
void EgReleaseSetup ()
{
   if ( true == ADSRPotCtrl )
   {
      pFilterEnveopeGenerator->SetRelease( releasePot );
   }
   else
   {
      pAmplifierEnvelopeGenerator->SetRelease  ( releasePot );
   }
}
void EgDecaySetup ()
{
   if ( true == ADSRPotCtrl )
   {
      pFilterEnveopeGenerator->SetDecay( decayPot );
   }
   else
   {
      pAmplifierEnvelopeGenerator->SetDecay  ( decayPot );
   }
}
void EgAttackSetup ()
{
   if ( true == ADSRPotCtrl )
   {
      pFilterEnveopeGenerator->SetAttack  ( attackPot );
   }
   else
   {
      pAmplifierEnvelopeGenerator->SetAttack  ( attackPot );
   }
}
void EgSustainSetup()
{
   if ( true == ADSRPotCtrl )
   {
      pFilterEnveopeGenerator->SetSustain( (1<<16) - ( (1<<16) - (sustainPot << 4)) );
   }
   else
   {
      pAmplifierEnvelopeGenerator->SetSustain( (1<<16) - ( (1<<16) - (sustainPot << 4)) );
   }
}
void VcfCutoffSetup ()
{
#ifdef MOOG_LPF
      pFilter->SetCutoff( frequencyPot << 3 );
#endif
#ifdef SIMPLE_LPF
      pFilter->SetCutoff( frequencyPot << 4 );
#endif
}
void VcfResonanceSetup ()
{
#ifdef MOOG_LPF
      pFilter->SetResonance( ressonancePot << 4 );
#endif
#ifdef SIMPLE_LPF
      pFilter->SetResonance( ressonancePot << 3 );
#endif
}

void Lfo1FrequencySetup()
{
   pOscillatorLFO1->SetPhaseIncrement( ((int32)(LFO1Pot >> 4) * (int32)s_LFO_phaseIncrementMultiplier) >> 4 );
}

void Vco1FineTunning()
{
   pOscillator01->SetFineTunning( (1 << 16) + ( VCO1Tun - 2048 ) );
}

void Vco2FineTunning()
{
   pOscillator02->SetFineTunning( (1 << 16) + ( VCO2Tun - 2048 ) );
}

///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
#ifndef WIN32
void RefreshSwitchesAndPots ()
{
   READ_DIGITAL_SWITCHES_AND_EXECUTE ( notePriotiryMode, NOTE_PRIOR_SEL1, NOTE_PRIOR_SEL2,
                                          [](void){pNoteController->SetNotePriorityMode( NoteController::eLAST );},
                                          [](void){pNoteController->SetNotePriorityMode( NoteController::eMOOG_MODE );},
                                          [](void){pNoteController->SetNotePriorityMode( NoteController::eARP_MODE );});

   READ_DIGITAL_SWITCHES_AND_EXECUTE ( vco1WaveSel, VCO1_WAVE_SEL1, VCO1_WAVE_SEL2,
                                          [](void){pOscillator01->SetOscilatorMode(Oscillator::eSQUARE_WAVE);},
                                          [](void){pOscillator01->SetOscilatorMode(Oscillator::eSAWTOOTH_WAVE);},
                                          [](void){pOscillator01->SetOscilatorMode(Oscillator::eSINE_WAVE);});

   READ_DIGITAL_SWITCHES_AND_EXECUTE ( vco2WaveSel, VCO2_WAVE_SEL1, VCO2_WAVE_SEL2,
                                          [](void){pOscillator02->SetOscilatorMode(Oscillator::eSQUARE_WAVE);} ,
                                          [](void){pOscillator02->SetOscilatorMode(Oscillator::eSAWTOOTH_WAVE);},
                                          [](void){pOscillator02->SetOscilatorMode(Oscillator::eSINE_WAVE);});

   READ_DIGITAL_SWITCHES_AND_EXECUTE ( vcfEgSel, EG_CUTOFF_ALL, EG_CUTOFF_VCO2,
                                          [](void){},
                                          [](void){},
                                          [](void){});

   READ_DIGITAL_SWITCHES_AND_EXECUTE ( delaySel, DELAY_SEL1, DELAY_SEL2,
                                          [](void){},
                                          [](void){pDelayWithFeedback->SetFeedbackGain(DelayWithFeedback::s_LowFeedbackGain);},
                                          [](void){pDelayWithFeedback->SetFeedbackGain(DelayWithFeedback::s_HighFeedbackGain);});

   ADSRPotCtrl = digitalRead ( ADSR_POT_CTRL );

   if ( EG_Mode != digitalRead (EG_MODE) )
   {
      EG_Mode = digitalRead (EG_MODE);
      if ( true == EG_Mode )
      {
         pOscillator01->SetOctave(2);
      }
      else
      {
         pOscillator01->SetOctave(0);
      }
   }

   
   if ( isHammond != digitalRead (IS_HAMMOND) )
   {
      isHammond = digitalRead (IS_HAMMOND);
      if ( true == isHammond )
      {
         InitializeHammondMode();
      }
      else
      {
         InitializeVirtualAnalogMode();
      }
   }


   if ( portamentoEnable != digitalRead(PORTAMENTO_ENABLE) )
   {
      if ( true == digitalRead(PORTAMENTO_ENABLE) )
      {
         portamentoEnable = true;
      }
      else
      {
         portamentoEnable = false;
      }
   }

   if ( Vco2Enable != digitalRead ( VCO2_ENABLE ) )
   {
      if ( true == digitalRead ( VCO2_ENABLE ) )
      {
         Vco2Enable = true;
      }
      else
      {
         Vco2Enable = false;
      }
   }


   if ( Lfo1Vco1Enable != digitalRead (LFO1_VCO1) )
   {
      if ( true == digitalRead (LFO1_VCO1) )
      {
         Lfo1Vco1Enable = true;
      }
      else
      {
         Lfo1Vco1Enable = false;
         pOscillator01->SetFrequencyMultiplier ( 1 << 16 );
      }
   }

   //                          Var         ,   Input        , Ignore,    Setter Fct       , Debug print
   REFRESH_ANALOG_PARAMETER( attackPot     , ATTACK_POT     , 0x0F  , EgAttackSetup       , shouldPrint);
   REFRESH_ANALOG_PARAMETER( sustainPot    , SUSTAIN_POT    , 0x0F  , EgSustainSetup      , shouldPrint);
   REFRESH_ANALOG_PARAMETER( decayPot      , DECAY_POT      , 0x0F  , EgDecaySetup        , shouldPrint);
   REFRESH_ANALOG_PARAMETER( releasePot    , RELEASE_POT    , 0x0F  , EgReleaseSetup      , shouldPrint);
   REFRESH_ANALOG_PARAMETER( portamentoVCO1, PORTAMENTO_VCO1, 0x0F  , Vco1PortamentoSetup , shouldPrint);
   REFRESH_ANALOG_PARAMETER( portamentoVCO2, PORTAMENTO_VCO2, 0x0F  , Vco2PortamentoSetup , shouldPrint);
   REFRESH_ANALOG_PARAMETER( frequencyPot  , FREQUENCY_POT  , 0x0F  , VcfCutoffSetup      , shouldPrint);
   REFRESH_ANALOG_PARAMETER( ressonancePot , RESSONANCE_POT , 0x0F  , VcfResonanceSetup   , shouldPrint);
   REFRESH_ANALOG_PARAMETER( LFO1Pot       , LFO1_POT       , 0x0F  , Lfo1FrequencySetup  , shouldPrint);
   REFRESH_ANALOG_PARAMETER( VCO1Tun       , VCO1_TUN       , 0x0F  , Vco1FineTunning     , shouldPrint);
   REFRESH_ANALOG_PARAMETER( VCO2Tun       , VCO2_TUN       , 0x0F  , Vco2FineTunning     , shouldPrint);

}
#endif


///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
#ifndef WIN32
void loop()
{
   // put your main code here, to run repeatedly:

   while ( true )
   {
#ifdef MIDI_ENABLE
      Midi.read();

      if ( true == isHammond )
      {
         ProcessNotesHammond();
      }
      else
      {
         pNoteController->ProcessNotesVirtualAnalog();
         pAmplifierEnvelopeGenerator->SetGate   ( pNoteController->getGate()    );
         pFilterEnveopeGenerator    ->SetGate   ( pNoteController->getGate()    );
         if ( true == pNoteController->getTrigger() )
         {
            pAmplifierEnvelopeGenerator->SetTrigger( true );
            pFilterEnveopeGenerator    ->SetTrigger( true );
            pOscillator01->SetPhaseIncrement( phaseIncrement [ pNoteController->getCurrentNote() ] );
            pOscillator02->SetPhaseIncrement( phaseIncrement [ pNoteController->getCurrentNote() ] );
         }
      }
#endif
#ifdef KEYBED
      pNoteController->ScanKeybed();
#endif

      if ( true == isTimeToCalculateEnvelopeGenerator )
      {
         static uint8 eg = 0;
         isTimeToCalculateEnvelopeGenerator = false;

         if ( ++timer25ms >= 10 )
         {
            RefreshSwitchesAndPots();
            timer25ms = 0;
         }

         if ( true == isHammond )
         {
            UpdateDrawbarsValue();
         }
         else
         {
            if ( false == resultsPrinted && timeCountIndex >= MEASURE_QUANTITY )
            {
               uint32   maxValue = 0x0;
               uint32   minValue = 0xFFFFFFFF;
               for ( int j = 0; j < MEASURE_QUANTITY; j++ )
               {
                  if ( timeCount[j] > maxValue )
                  {
                     maxValue = timeCount[j];
                  }
                  if ( timeCount[j] < minValue )
                  {
                     minValue = timeCount[j];
                  }
               }
               Serial.print ("MaxValue: ");
               Serial.println (maxValue);
               Serial.print ("MinValue: ");
               Serial.println (minValue);
               Serial.println ("--Values--");
               for (int i = 0; i < MEASURE_QUANTITY; i++ )
               {
                  Serial.print (timeCount[i]);
                  Serial.println (",");
               }
               resultsPrinted = true;
            }

            pAmplifierEnvelopeGenerator->Process();
            pFilterEnveopeGenerator    ->Process();

#ifdef SIMPLE_LPF
            static bool selectionChange = false;
            if ( 2 != vcfEgSel )
            {
               pFilter->SetCutoffMultiplier( (1 << 16) - pFilterEnveopeGenerator->getOutput() );
               selectionChange = false;
            }
            else
            {
               if ( false == selectionChange )
               {
                  pFilter->SetCutoffMultiplier( 1 << 16 );
                  selectionChange = true;
               }
            }
#endif
         }
      }
   }

}
#endif

///////////////////////////////////////////////////////////////////////////////
/// @brief
///////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
int main()
{
   using namespace std;

   // Call Arduino setup function
   setup();
   // Simulate note pressed
   MidiHandleNoteOn(1, 60, 127);

   if (true == isHammond)
   {
      UpdateDrawbarsValue();
   }

   for (int i = 0; i < (1 * 96000); i++)
   {
      if ( false == isHammond )
      {
         pNoteController->ProcessNotesVirtualAnalog();
         pAmplifierEnvelopeGenerator->SetGate(pNoteController->getGate());
         if (true == pNoteController->getTrigger())
         {
            pAmplifierEnvelopeGenerator->SetTrigger(true);
            pOscillator01->SetPhaseIncrement(phaseIncrement[pNoteController->getCurrentNote()]);
            pOscillator02->SetPhaseIncrement(phaseIncrement[pNoteController->getCurrentNote()]);
         }
      }
      else
      {
         ProcessNotesHammond ();
      }

#ifdef TDA1543A
      ProcessI2sInterrupt( I2sJapaneseFormat::eLEFT_CHANNEL );
      if (i % 96 == 0)
      {
         ADSRTimerHandler();
      }
#endif
      if (true == isTimeToCalculateEnvelopeGenerator)
      {
         isTimeToCalculateEnvelopeGenerator = false;

         pAmplifierEnvelopeGenerator->Process();
         pFilterEnveopeGenerator->Process();
         //pFilter->SetCutoffMultiplier( pFilterEnveopeGenerator->getOutput() );
      }
   }

   return 0;
}
#endif


