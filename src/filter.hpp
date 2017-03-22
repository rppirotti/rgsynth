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

#ifndef FILTER_H
#define FILTER_H


////////////////////////////////////////////////////////////////////////////
/// @brief Type for audio channel mode selection
////////////////////////////////////////////////////////////////////////////
class BaseFilter
{
public:

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   typedef enum
   {
      eLOW_PASS,
      eHIGH_PASS,
      ePASS_BAND,
      eREJECT_BAND
   } EnFilterType;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   BaseFilter()
   : m_filterType ( eLOW_PASS )
   , m_cutoffMultiplier ( 0x10000 )
   , m_resonanceMultiplier ( 0x10000 )
   { }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   virtual ~BaseFilter () { }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   virtual int32 process         ( int32  input    ) = 0 ;
   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   virtual void  SetCutoff       ( uint32 cutoff   ) = 0 ;
   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   virtual void  SetResonance    ( uint32 resonance) = 0 ;
   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   virtual void  SetFilterType         ( EnFilterType filterType    ) { m_filterType          = filterType;          }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   virtual void  SetCutoffMultiplier   ( uint32 cutoffMultiplier    ) { m_cutoffMultiplier    = cutoffMultiplier;    }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   virtual void  SetResonanceMultiplier( uint32 resonanceMultiplier ) { m_resonanceMultiplier = resonanceMultiplier; }

protected:
   EnFilterType   m_filterType;
   uint32         m_cutoffMultiplier;
   uint32         m_resonanceMultiplier;

};



#define FRAC_WIDTH 16

//   b1 = (*in + b0) * p - b1 * f;
#define COEF_CALC(store, cfA, tmpA) { \
      int64 temp = ((int64)cfA + (int64)tmpA) * (int64)p; \
      temp = ((int64)temp)-((int64)((int64)store * (int64)f)); \
      store = temp >> 16; \
      }

static const int16   sMaxValue = 2047;
static const int16   sMinValue = -2047;


////////////////////////////////////////////////////////////////////////////
/// @brief Type for audio channel mode selection
////////////////////////////////////////////////////////////////////////////
class MoogFilter
{
public:
   MoogFilter()
   : q  ( 0 )
   , p  ( 0 )
   , f  ( 0 )
   , b0 ( 0 )
   , b1 ( 0 )
   , b2 ( 0 )
   , b3 ( 0 )
   , b4 ( 0 )
   , t1 ( 0 )
   , t2 ( 0 )
   { }


   void coefficients(float frequency, float resonance)
   {
      q = 1.0f - frequency;
      p = frequency + 0.8f * frequency * q;
      f = p + p - 1.0f;
      q = resonance * (1.0f + 0.5f * q * (1.0f - q + 5.6f * q * q));
   }

   void process(float *in, float *out)
   {
      *in -= q * b4;                          //feedback
      t1 = b1;
      b1 = (*in + b0) * p - b1 * f;
      t2 = b2;
      b2 = (b1 + t1) * p - b2 * f;
      t1 = b3;
      b3 = (b2 + t2) * p - b3 * f;
      b4 = (b3 + t1) * p - b4 * f;
      b4 = b4 - b4 * b4 * b4 * 0.166667f;    //clipping
      b0 = *in;
      *out = b4;
   }

private:
   float q, f, p;
   float t1, t2, b0, b1, b2, b3, b4;

};

///////////////////////////////////////////////////////////////////////////////
/// @brief    Fixed point implementation of a 24dB resonant low pass filter
/// @details: This code was adapted from http://musicdsp.org/archive.php?classid=3#25,
///           Moog VCF, variation 1, which points to
///           CSound source code, Stilson/Smith CCRMA paper., Paul Kellett version
///////////////////////////////////////////////////////////////////////////////
class IntFracMoogFilter :  public BaseFilter
{
public:

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   IntFracMoogFilter()
   : q  ( 0 )
   , f  ( 0 )
   , p  ( 0 )
   , b0 ( 0 )
   , b1 ( 0 )
   , b2 ( 0 )
   , b3 ( 0 )
   , b4 ( 0 )
   , t1 ( 0 )
   , t2 ( 0 )
   {
    
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void  SetCutoff ( uint32 cutoff )
   {
      m_cutoff    = cutoff;
      RefreshCoefficients( );
   }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void  SetResonance ( uint32 resonance )
   {
      if ( resonance < 256 )
      {
         m_resonance = 256;
         RefreshCoefficients( );
      }
      else
      {
         m_resonance = resonance;
         RefreshCoefficients( );
      }
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   void  SetCutoffMultiplier   ( uint32 cutoffMultiplier )
   {
      m_cutoffMultiplier    = cutoffMultiplier;
      SetCutoff ( (uint64)((uint64)m_cutoff * (uint64)m_cutoffMultiplier) >> 16 );
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void RefreshCoefficients( )
   {
      noInterrupts();
      //   q = 1.0f - frequency;
      q = (1 << FRAC_WIDTH) - m_cutoff;  // 16.16 = 16.16 - 16.16
      //   p = frequency + 0.8f * frequency * q;
      uint64 tempP = 0x0000CCCC * m_cutoff;  // 32.32 = 16.16. * 16.16   (i.e., only frac)
      tempP = (tempP >> 16) * q;  // 32.32 = 16.16. * 16.16 (i.e., only frac)
      p = m_cutoff + (tempP >> FRAC_WIDTH);  // 16.16 = 16.16 + 16.16
      //   f = p + p - 1.0f;
      f = p + p - (1 << FRAC_WIDTH);   // 16.16 = 16.16 + 16.16 - 16.16;
      //   q = resonance * (1.0f + 0.5f * q * (1.0f - q + 5.6f * q * q));
      uint64 tempQ  = (uint64)q * (uint64)q; // 32.32 = 16.16 // (i.e., only frac)  q * q
      uint64 temp64 = 0x0005999A * (tempQ >> 16);   // 32.32 = 16.16 * 16.16;       5.6f * q * q
      temp64 = (1 << FRAC_WIDTH) - q + (temp64 >> 16);   // 16.16 = 16.16 - 16.16 + 16.16      (1.0f - q + 5.6f * q * q)
      temp64 = temp64 * q; // 32.32 = 16.16 * 16.16                                 q * (1.0f - q + 5.6f * q * q)
      temp64 = (temp64 >> 16) * 0x00008000; // 32.32 = 16.16 * 16.16                0.5f * q * (1.0f - q + 5.6f * q * q)
      temp64 = (temp64 >> 16) + (1 << FRAC_WIDTH); // 16.16 = 16.16 + 16.16         (1.0f + 0.5f * q * (1.0f - q + 5.6f * q * q)
      temp64 = m_resonance * temp64;                                                  //resonance * (1.0f + 0.5f * q * (1.0f - q + 5.6f * q * q)
      q = temp64 >> 16;
      interrupts();
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline int32 process( int32 input )
   {
      // delta = 2047 - ( -2047) = 4094
      // 1 = 2047
      // -1 = -2047
      // 0 = 0
      // 2047 --> 1111 1111 1111 1111
      // -2047 -->

      // Normaliza input to fixed point -1.0...+1.0
      int32 inCopy = input << 5;

//      if (inCopy > 0 )
//      {
//         inCopy = (inCopy << 5) + (2^5 -1);
//      }
//      else if (inCopy < 0 )
//      {
//         inCopy = ((~inCopy + 1) << 5) + (2^5 -1);
//         inCopy = 0 - inCopy;
//      }

      //   *in -= q * b4;                          //feedback
      int64 tempIn = (int64)q * (int64)b4; // 32.32 = 16.16*16.16
      inCopy = inCopy - (tempIn >> 16);;  // 16.16 = 16.16 - 16.16
      //   t1 = b1;
      t1 = b1;

      //   b1 = (*in + b0) * p - b1 * f;
      //int64 tempB1 = (in + b0) * p; // 32.32 = 16.16 * 16.16
      //tempB1 = (tempB1) - ((int64)((int64)b1 * (int64)f));  // 32.32 = 32.32 - 32.32
      //b1 = tempB1 >> 16;
      COEF_CALC(b1, inCopy, b0);
      t2 = b2;
      COEF_CALC(b2, b1, t1);
      t1 = b3;
      COEF_CALC(b3, b2, t2);
      COEF_CALC(b4, b3, t1);
      // clipping
      int64 tempB4 = (int64)b4 * (int64)b4; // 32.32 = 16.16 * 16.16
      tempB4 = (tempB4 >> 16) * (int64)b4; // 32.32 = 16.16 * 16.16
      tempB4 = (tempB4 >> 16) * 0x00002AAB; // 32.32 = 16.16 * 16.16
      b4 = b4 - (tempB4 >> 16); // 16.16 = 16.16 - 16.16

      b0 = inCopy;
      return ( ((int64)b4 * (int64)sMaxValue) >> 16);
   }

private:
   int32 q, f, p;    // 16.16
   int32 t1, t2, b0, b1, b2, b3, b4;   // 16.16
   int32 m_cutoff, m_resonance;

};

///////////////////////////////////////////////////////////////////////////////
/// @brief
/// @details by Paul Kellett http://www.musicdsp.org/archive.php?classid=3#29
///////////////////////////////////////////////////////////////////////////////
class SimpleLowPassFilter :  public BaseFilter
{
public:

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   SimpleLowPassFilter ()
   : m_buf0 ( 0 )
   , m_buf1 ( 0 )
   , m_buf2 ( 0 )
   , m_buf3 ( 0 )
   , m_cutoff    ( 0x00008000 )
   , m_feedback  ( 0 )
   , m_resonance ( 0x00008000 )
   {
      UpdateFeedback();
   }


   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void  SetCutoff ( uint32 cutoff )
   {
      m_cutoff    = cutoff >= (65000) ? 65000 : cutoff;
      m_modulatedCutoff = m_cutoff;
      UpdateFeedback( );
   }
   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void  SetResonance ( uint32 resonance )
   {
      m_resonance = resonance;
      UpdateFeedback( );
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   void  SetCutoffMultiplier   ( uint32 cutoffMultiplier )
   {
      m_cutoffMultiplier    = cutoffMultiplier;
      m_modulatedCutoff = (uint64)((uint64)m_cutoff * (uint64)m_cutoffMultiplier) >> 16;
      UpdateFeedback();
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline int32 process ( int32 input )
   {
      // Convert input to fixed point -1.0...+1.0
      int32 inCopy = input;

      //if (inCopy > 0 )
      //{
      inCopy = (inCopy << 5);
      //}
      //else if (inCopy < 0 )
      //{
      //   inCopy = ((~inCopy + 1) << 3);
      //   inCopy = 0 - inCopy;
      //}

      int64 temp = ((int64)(m_feedback * (int64)(inCopy - m_buf1)) >> 16);
      m_buf0 += ((int64)m_modulatedCutoff * (int64)( inCopy - m_buf0 + temp )) >> 16;
      m_buf1 += ((int64)m_modulatedCutoff * ( m_buf0 - m_buf1 ) )>> 16;
      m_buf2 += ((int64)m_modulatedCutoff * ( m_buf1 - m_buf2 ) )>> 16;
      m_buf3 += ((int64)m_modulatedCutoff * ( m_buf2 - m_buf3 ) )>> 16;

      return (m_buf3 * sMaxValue) >> 16;
   }

private:

   void UpdateFeedback ()
   {
      noInterrupts();
      //m_feedback  = m_resonance + ( (m_resonance / ((uint64)(1 << 16) - m_cutoff ) ) << 16);
      m_feedback  = m_resonance + ( (m_resonance * ((uint32)(1 << 16) - m_modulatedCutoff ) ) >> 16 );
//      if (m_feedback >= 0x00020000)
//      {
//         m_feedback = 0x00020000;
//      }
      interrupts();
   }

   int32    m_cutoff;
   int32    m_modulatedCutoff;
   int64    m_resonance;
   int64    m_feedback;
   int64    m_buf0;
   int64    m_buf1;
   int64    m_buf2;
   int64    m_buf3;
};

#endif

