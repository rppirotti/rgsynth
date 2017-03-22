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

#ifndef PHASE_INCREMENT_H
#define PHASE_INCREMENT_H


struct PhaseIncrement
{
   PhaseIncrement()
   {
      memset (&m_phaseIncrement[0] , 0, sizeof(m_phaseIncrement));
   }

   inline uint32 operator [] ( uint8 note ) const
   {
      return m_phaseIncrement[note];
   }

   void  SetSampleRate24kHz ()
   {
      InitializeArray ( true );
   }
   void  SetSampleRate48kHz ()
   {
      InitializeArray ( false );
   }

private:

   ///////////////////////////////////////////////////////////////////////////////
   /// @brief Fixed point phase increment value for each note, based on sample rate.
   /// @details Each position on this arrays is a int+frac value, to be
   ///          accumulated each sample rate period. The int value must be
   ///          used to index the sample waveform array.
   ///          Integer = 16 bits / Frac = 16 bits
   ///          The phase inc value is = (1/SampleRate)/((1/NoteFreq)/SampleSize)
   ///                              Ex.: (1/48000)/((1/440)/1024)=9,386666662
   ///////////////////////////////////////////////////////////////////////////////
   uint32   m_phaseIncrement [ LAST_MIDI_NOTE + 1 ];

   void InitializeArray ( bool is24kHz )
   {

      //C2 = 32,7 Hz
       m_phaseIncrement[24] =  ((0 << 16) + 45717);
       m_phaseIncrement[25] =  ((0 << 16) + 48444);
       //D2
       m_phaseIncrement[26] =  ((0 << 16) + 51324);
       m_phaseIncrement[27] =  ((0 << 16) + 54372);
       //E2
       m_phaseIncrement[28] =  ((0 << 16) + 57601);
       //F2
       m_phaseIncrement[29] =  ((0 << 16) + 61027);
       m_phaseIncrement[30] =  ((0 << 16) + 64662);
       //G2
       m_phaseIncrement[31] =  ((1 << 16) + 2970);
       m_phaseIncrement[32] =  ((1 << 16) + 7039);
       //A2
       m_phaseIncrement[33] =  ((1 << 16) + 11359);
       m_phaseIncrement[34] = ((1 << 16) + 15931);
       //B2
       m_phaseIncrement[35] = ((1 << 16) + 20872);

       //C3
       m_phaseIncrement[36] =  ((1 << 16) + 25913);
       m_phaseIncrement[37] =  ((1 << 16) + 31352);
       //D3
       m_phaseIncrement[38] =  ((1 << 16) + 37112);
       m_phaseIncrement[39] =  ((1 << 16) + 43208);
       //E3
       m_phaseIncrement[40] =  ((1 << 16) + 49681);
       //F3
       m_phaseIncrement[41] =  ((1 << 16) + 56532);
       m_phaseIncrement[42] =  ((1 << 16) + 63788);
       //G3
       m_phaseIncrement[43] =  ((2 << 16) + 5941);
       m_phaseIncrement[44] =  ((2 << 16) + 14092);
       //A3
       m_phaseIncrement[45] = ((2 << 16) + 22719);
       m_phaseIncrement[46] = ((2 << 16) + 31862);
       //B3
       m_phaseIncrement[47] = ((2 << 16) + 41551);

       //C4.
       m_phaseIncrement[48] =  ((2 << 16) + 51813);
       m_phaseIncrement[49] =  ((2 << 16) + 62690);
       //D4
       m_phaseIncrement[50] =  ((3 << 16) + 8675);
       m_phaseIncrement[51] =  ((3 << 16) + 20880);
       //E4
       m_phaseIncrement[52] =  ((3 << 16) + 33813);
       //F4
       m_phaseIncrement[53] =  ((3 << 16) + 47514);
       m_phaseIncrement[54] =  ((3 << 16) + 62040);
       //G4
       m_phaseIncrement[55] =  ((4 << 16) + 11883);
       m_phaseIncrement[56] =  ((4 << 16) + 28171);
       //A4
       m_phaseIncrement[57] =  ((4 << 16) + 45438);
       m_phaseIncrement[58] = ((4 << 16) + 63725);
       //B4
       m_phaseIncrement[59] = ((5 << 16) + 17567);

       // C5. Phase increment calculated value = 5,58144
       m_phaseIncrement[60] =  ((5 << 16) + 38105);
       // C5#
       m_phaseIncrement[61] =  ((5 << 16) + 59845);
       // D5
       m_phaseIncrement[62] =  ((6 << 16) + 17350);
       // D5#
       m_phaseIncrement[63] =  ((6 << 16) + 41775);
       // E5
       m_phaseIncrement[64] =  ((7 << 16) + 2104);
       // F5
       m_phaseIncrement[65] =  ((7 << 16) + 29506);
       // F5#
       m_phaseIncrement[66] =  ((7 << 16) + 58531);
       // G5
       m_phaseIncrement[67] =  ((8 << 16) + 23753);
       // G5#
       m_phaseIncrement[68] =  ((8 << 16) + 56343);
       // A5
       m_phaseIncrement[69] =  ((9 << 16) + 24340);
       // A5#
       m_phaseIncrement[70] = ((9 << 16) + 61914);
       // B5
       m_phaseIncrement[71] = ((10 << 16) + 35134);
       // C6
       m_phaseIncrement[72] = ((11 << 16) + 10660);
       m_phaseIncrement[73] = ((11 << 16) + 54169);
       //D6
       m_phaseIncrement[74] = ((12 << 16) + 34714);
       m_phaseIncrement[75] = ((13 << 16) + 18000);
       //E6
       m_phaseIncrement[76] = ((14 << 16) + 4208);
       //F6
       m_phaseIncrement[77] = ((14 << 16) + 59013);
       m_phaseIncrement[78] = ((15 << 16) + 51541);
       //G6
       m_phaseIncrement[79] = ((16 << 16) + 47521);
       m_phaseIncrement[80] = ((17 << 16) + 47164);
       //A6
       m_phaseIncrement[81] = ((18 << 16) + 50681);
       m_phaseIncrement[82] = ((19 << 16) + 58307);
       //B6
       m_phaseIncrement[83] = ((21 << 16) + 4746);
       //C7
       m_phaseIncrement[84] = ((22 << 16) + 21321);
       m_phaseIncrement[85] = ((23 << 16) + 42788);
       //D7
       m_phaseIncrement[86] = ((25 << 16) + 3893);
       m_phaseIncrement[87] = ((26 << 16) + 36015);
       //E7
       m_phaseIncrement[88] = ((28 << 16) + 8402);
       //F7
       m_phaseIncrement[89] = ((29 << 16) + 52477);
       m_phaseIncrement[90] = ((31 << 16) + 37546);
       //G7
       m_phaseIncrement[91] = ((33 << 16) + 29506);
       m_phaseIncrement[92] = ((35 << 16) + 28793);
       //A7
       m_phaseIncrement[93] = ((37 << 16) + 35826);
       m_phaseIncrement[94] = ((39 << 16) + 51079);
       //B7
       m_phaseIncrement[95] = ((42 << 16) + 9479);
       //C8
       m_phaseIncrement[96] = ((44 << 16) + 42642);
       m_phaseIncrement[97] = ((47 << 16) + 20041);
       //D8
       m_phaseIncrement[98] = ((50 << 16) + 7787);
       m_phaseIncrement[99] = ((53 << 16) + 6494);
       //E8
       m_phaseIncrement[100] = ((56 << 16) + 16805);
       //F8
       m_phaseIncrement[101] = ((59 << 16) + 39433);
       m_phaseIncrement[102] = ((63 << 16) + 9556);
       //G8
       m_phaseIncrement[103] = ((66 << 16) + 59013);
       m_phaseIncrement[104] = ((70 << 16) + 57587);
       //A8
       m_phaseIncrement[105] = ((75 << 16) + 6116);
       m_phaseIncrement[106] = ((79 << 16) + 36609);
       //B8
       m_phaseIncrement[107] = ((84 << 16) + 18972);
       //C9
       m_phaseIncrement[108] = ((89 << 16) + 19748);
       m_phaseIncrement[109] = ((94 << 16) + 40083);
       //D9
       m_phaseIncrement[110] = ((100 << 16) + 15560);
       m_phaseIncrement[111] = ((106 << 16) + 12974);
       //E9
       m_phaseIncrement[112] = ((112 << 16) + 33610);
       //F9
       m_phaseIncrement[113] = ((119 << 16) + 13316);
       m_phaseIncrement[114] = ((126 << 16) + 19098);
       //G9
       m_phaseIncrement[115] = ((133 << 16) + 52505);
       m_phaseIncrement[116] = ((141 << 16) + 49639);
       //A9
       m_phaseIncrement[117] = ((150 << 16) + 12233);
       m_phaseIncrement[118] = ((159 << 16) + 7682);
       //B9
       m_phaseIncrement[119] = ((168 << 16) + 37930);
       //C10
       m_phaseIncrement[120] = ((178 << 16) + 39496);

       // For 24kHz sample rate, we divide by 2 the
       // phase increment.
       if ( true == is24kHz )
       {
          for (int i=0; i < LAST_MIDI_NOTE + 1 ; i++)
          {
             m_phaseIncrement [i] = m_phaseIncrement [i] >> 1;
          }

       }

   }



};

#endif



