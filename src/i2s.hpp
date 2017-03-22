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

#ifndef I2S_H
#define I2S_H

#ifdef WIN32
#include <iostream>
#include <fstream>
#endif

#define  SSC_IRQ_PRIO   4


//////////////////////////////////////////////////////////////////////////////
/// @brief Class to use Arduino Due SSC port with an external I2S DAC
/// @details For now, only support I2S Master mode.
///////////////////////////////////////////////////////////////////////////////
class I2sJapaneseFormat
{
public:

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   typedef enum
   {
      eAUDIO_STEREO,
      eAUDIO_MONO_LEFT_ONLY,
      eAUDIO_MONO_RIGHT_ONLY
   } EnAudioMode;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for clock mode selection
   ////////////////////////////////////////////////////////////////////////////
   typedef enum
   {
      eCLOCK_MODE_EXTERNAL_CLOCK,
      eCLOCK_MODE_RX_TX_CLOCK,
      eCLOCK_MODE_INTERNAL_CLOCK
   } EnClockMode;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for channel information
   ////////////////////////////////////////////////////////////////////////////
   typedef enum
   {
      eLEFT_CHANNEL,
      eRIGHT_CHANNEL
   } EnChannel;

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Callback for an user function callback for TX READY interrupt
   ////////////////////////////////////////////////////////////////////////////
   typedef  void (*TxReadyCallback)(EnChannel channel);


   ////////////////////////////////////////////////////////////////////////////
   /// @brief Constructor
   /// @param bitsPerChannel  Number of bits per channel
   /// @param sampleRate  The sample rate to be used, in Hz
   /// @param numberOFChannels  Number of channels - 1 or 2
   ////////////////////////////////////////////////////////////////////////////
   I2sJapaneseFormat ( const char* moduleName)
   : m_pModuleName ( moduleName )
   , m_pDataOut    ( NULL )
   {
#if !defined(WIN32) //&& !defined(linux)
      /* Initialize the SSC module and work in loop mode. */
      pmc_enable_periph_clk(ID_SSC);
      ssc_reset(SSC);
#endif
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Configure SSC transmitter block
   /// @param audioMode  Select MONO or STEREO modes
   /// @param clockMode  Select type of clock operation
   /// @param txReadyCallback  A user function to be called on TX INT
   ////////////////////////////////////////////////////////////////////////////
   bool configureTransmitter ( uint8 bitsPerChannel, uint16 sampleRate, EnAudioMode audioMode, EnClockMode clockMode, TxReadyCallback txReadyCallback )
   {
      bool  success = true;

#if !defined(WIN32) //&& !defined(linux)

      clock_opt_t transmitterClockOptions;
      memset ((uint8*)&transmitterClockOptions, 0, sizeof(clock_opt_t));

      data_frame_opt_t  dataFrameOptions;
      memset ((uint8*)&dataFrameOptions, 0, sizeof(data_frame_opt_t));

      uint32   ul_mck = 84000000;
      uint32   numberOfChannels = ( eAUDIO_STEREO == audioMode ) ? 2 : 1;
      if ( 48000 == sampleRate)
      {
         ssc_set_clock_divider(SSC, bitsPerChannel * sampleRate * numberOfChannels, ul_mck);
      }
      else
      {
         //ssc_set_clock_divider(SSC, 1500000, ul_mck); //calibrated with subtractive
         ssc_set_clock_divider(SSC, 1470000, ul_mck); //calibrated with subtractive
      }


      // Configure Transmitter pints
      ConfigurePIO();

      dataFrameOptions.ul_datlen = bitsPerChannel - 1;    // Number of bits of the data
      dataFrameOptions.ul_msbf   = SSC_TFMR_MSBF;           // MSB bit first
      dataFrameOptions.ul_fsos   = SSC_TFMR_FSOS_TOGGLING;
      dataFrameOptions.ul_fsedge = SSC_TFMR_FSEDGE_POSITIVE;
      if ( eAUDIO_STEREO == audioMode )
      {
         dataFrameOptions.ul_datnb  = 1;     // Number of data words - 1
      }
      else
      {
         dataFrameOptions.ul_datnb  = 0;     // Number of data words - 1
      }

      switch ( clockMode )
      {
         case eCLOCK_MODE_EXTERNAL_CLOCK:
            success = false;
            break;

         case eCLOCK_MODE_INTERNAL_CLOCK:
            transmitterClockOptions.ul_cks = SSC_TCMR_CKS_MCK;          // Clock = internal master clock
            transmitterClockOptions.ul_cko = SSC_TCMR_CKO_CONTINUOUS;   // Mode = continuous
            transmitterClockOptions.ul_cki = 0;                         // Clock inversion = no
            transmitterClockOptions.ul_ckg = SSC_TCMR_CKG_NONE;         // Gating

            // Check below options
            /* The delay is defined by I2S protocol. */
            transmitterClockOptions.ul_start_sel = SSC_TCMR_START_CONTINUOUS;
            transmitterClockOptions.ul_sttdly = 1;
            transmitterClockOptions.ul_period = bitsPerChannel;
            break;

         case eCLOCK_MODE_RX_TX_CLOCK:
            success = false;
            break;
      }
      ssc_set_transmitter(SSC, &transmitterClockOptions, &dataFrameOptions);

      if ( txReadyCallback )
      {
         m_txReadyCallback = txReadyCallback;
      }
      else
      {
         success = false;
      }

      m_pDataOut = (uint32*)ssc_get_tx_access(SSC);

#endif

      return success;
   }

   ////////////////////////////////////////////////////////////////////////////
   ///
   ////////////////////////////////////////////////////////////////////////////
   inline void Write ( int16 data )
   {
#if !defined(WIN32) //&& !defined(linux)
      if ( m_pDataOut != NULL )
      {
         // TDA1543A uses 24-bit left justified format.
         *m_pDataOut = data << 8;
      }
#else
      std::ofstream fOutCh01, fOutCh02;
      static bool isSplitChannel = false;

      fOutCh01.open (std::string(m_pModuleName) + std::string("Ch01"), std::ofstream::app);
      fOutCh02.open (std::string(m_pModuleName) + std::string("Ch02"), std::ofstream::app);

      if ( true == isSplitChannel )
      {
         isSplitChannel = false;
         fOutCh01 << data << std::endl;
      }
      else
      {
         isSplitChannel = true;
         fOutCh02 << data << std::endl;
      }
      fOutCh01.close();
      fOutCh02.close();
#endif
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Enable and start SSC peripheral
   ////////////////////////////////////////////////////////////////////////////
   void  Start()
   {
#if !defined(WIN32) //&& !defined(linux)
      /* Enable SSC interrupt line from the core */
      NVIC_DisableIRQ(SSC_IRQn);
      NVIC_ClearPendingIRQ(SSC_IRQn);
      NVIC_SetPriority(SSC_IRQn, SSC_IRQ_PRIO);
      NVIC_EnableIRQ(SSC_IRQn);

      ssc_enable_tx(SSC);
      ssc_enable_interrupt(SSC, SSC_IER_TXRDY);
#endif
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief Type for audio channel mode selection
   ////////////////////////////////////////////////////////////////////////////
   static void HandlerInterrupt()
   {
#if !defined(WIN32) //&& !defined(linux)
      // Save STATUS register
      uint32 status = ssc_get_status(SSC);

      // Check TX READY interrupt
      if ( SSC_RC_YES == ssc_is_tx_ready(SSC) )
      {
         if ( status & SSC_IER_TXSYN )
         {
            m_txReadyCallback(eLEFT_CHANNEL);
         }
         else
         {
            m_txReadyCallback(eRIGHT_CHANNEL);
         }
      }
#endif
   }

private:

   ////////////////////////////////////////////////////////////////////////////
   ///
   ////////////////////////////////////////////////////////////////////////////
   void ConfigurePIO ()
   {
#if !defined(WIN32) //&& !defined(linux)
      uint32 regValue = 0;

      // Configure PA16 as SSC TD
      // Disable PIO controller for the pin
      REG_PIOA_PDR = PIO_PA16B_TD;
      // Disable interrupt on the pin
      REG_PIOA_IDR = PIO_PA16B_TD;
      // Set peripheral B
      regValue = REG_PIOA_ABSR;
      REG_PIOA_ABSR = regValue | (PIO_PA16B_TD);

      // Configure PA16 as SSC TD
      // Disable PIO controller for the pin
      REG_PIOA_PDR = PIO_PA15B_TF;
      // Disable interrupt on the pin
      REG_PIOA_IDR = PIO_PA15B_TF;
      // Set peripheral B
      regValue = REG_PIOA_ABSR;
      REG_PIOA_ABSR = regValue | (PIO_PA15B_TF);

      // Configure PA16 as SSC TD
      // Disable PIO controller for the pin
      REG_PIOA_PDR = PIO_PA14B_TK;
      // Disable interrupt on the pin
      REG_PIOA_IDR = PIO_PA14B_TK;
      // Set peripheral B
      regValue = REG_PIOA_ABSR;
      REG_PIOA_ABSR = regValue | (PIO_PA14B_TK);
#endif
   }

   static TxReadyCallback   m_txReadyCallback;
   uint32*           m_pDataOut;
   const char*       m_pModuleName;
};

///////////////////////////////////////////////////////////////////////////////
/// @brief SCC interrupt handler
/// @details This function is the SSC interrupt handler - Arduino compiler
///          automatically call this function on the SSC int vector
///////////////////////////////////////////////////////////////////////////////
void SSC_Handler (void)
{
   I2sJapaneseFormat::HandlerInterrupt();
}

I2sJapaneseFormat::TxReadyCallback   I2sJapaneseFormat::m_txReadyCallback = 0;

#endif











