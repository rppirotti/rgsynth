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

#ifndef NOTE_CONTROLLER_H
#define NOTE_CONTROLLER_H


class NoteController
{
public:

   typedef  enum
   {
      eMOOG_MODE = 1,
      eARP_MODE  = 2,
      eLAST      = 3
   } EnNotePriorityMode;


   static const int8 sm_NoPressedNote    = -1;
   static const int8 sm_maxNumberOfNotes = 127;

   ////////////////////////////////////////////////////////////////////////////
   /// @ctor
   ////////////////////////////////////////////////////////////////////////////
   NoteController( int8 lowestSupportedNote, int8 highestSupportedNote )
#ifdef KEYBED
   : m_notePriorityMode ( eMOOG_MODE )
#else
   : m_notePriorityMode ( eLAST )
#endif
   , m_trigger ( false )
   , m_gate    ( false )
   , m_noteOn1 ( sm_NoPressedNote )
   , m_noteOn2 ( sm_NoPressedNote )
   , m_noteOn3 ( sm_NoPressedNote )
   , m_noteOn4 ( sm_NoPressedNote )
   , m_currentNote     ( sm_NoPressedNote )
   , m_lastPressedNote ( sm_NoPressedNote )
   , m_numberOfNotesOn ( 0 )
   , m_lowestSupportedNote  ( lowestSupportedNote )
   , m_highestSupportedNote ( highestSupportedNote )
   {
      memset (&m_notesOn[0], sm_NoPressedNote, sizeof(m_notesOn));
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   int8 operator () ( int8 note ) const
   {
      return m_notesOn [ note ];
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void ScanKeybed  ()
   {
#if ( !defined(WIN32) ) //&& !defined(linux) )
      uint8 notes[120] = { 0 };
      memset (&notes[0], sm_NoPressedNote, sizeof(notes));
      uint8 index = 0;
      uint8 note  = 0;
      for ( int i = 1; i <= (1 << 6); i = i << 1)
      {
         digitalWrite (16, i & (1 << 0) );
         digitalWrite (15, i & (1 << 1) );
         digitalWrite (14, i & (1 << 2) );
         digitalWrite (2,  i & (1 << 3) );
         digitalWrite (3,  i & (1 << 4) );
         digitalWrite (4,  i & (1 << 5) );
         digitalWrite (5,  i & (1 << 6) );

         delayMicroseconds(1);

         for (int j=0; j<8; j++ )
         {
            note = 36 + (index * 8) + j;
            notes [ note ] = ( (uint8)digitalRead ( 6 + j )  << j ) ? note : sm_NoPressedNote;
         }
         index++;
      }

      // Copy only the valid notes according to the keybed we have
      noInterrupts();
      if ( eLAST == m_notePriorityMode )
      {
         for (int i = m_lowestSupportedNote; i <= m_highestSupportedNote; i++ )
         {
            if ( m_notesOn [ i ] != notes [ i ] )
            {
               if ( sm_NoPressedNote == notes [ i ] )
               {
                  HandleNoteOff (0, i);
               }
               else
               {
                  HandleNoteOn  (0, i);
               }
            }
         }
      }
      else
      {
         memcpy (&m_notesOn[36], &notes[36], 49);
      }
      interrupts();
#endif
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void HandleNoteOn ( int8 channel, int8 note )
   {
      if ( note >= m_lowestSupportedNote && note <= m_highestSupportedNote )
      {
         m_notesOn [ note ] = note;
         m_noteOn4 = m_noteOn3;
         m_noteOn3 = m_noteOn2;
         m_noteOn2 = m_noteOn1;
         m_noteOn1 = m_currentNote;
         m_lastPressedNote = note;
      }
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void HandleNoteOff ( int8 channel, int8 note )
   {
      if ( note >= m_lowestSupportedNote && note <= m_highestSupportedNote )
      {
         m_notesOn [ note ] = sm_NoPressedNote;

         if ( note == m_currentNote )
         {
            m_lastPressedNote = m_noteOn1;
            m_noteOn1 = m_noteOn2;
            m_noteOn2 = m_noteOn3;
            m_noteOn3 = m_noteOn4;
            m_noteOn4 = sm_NoPressedNote;
         } else if ( note == m_noteOn1 )
         {
            m_noteOn1 = m_noteOn2;
            m_noteOn2 = m_noteOn3;
            m_noteOn3 = m_noteOn4;
            m_noteOn4 = sm_NoPressedNote;
         } else if ( note == m_noteOn2 )
         {
            m_noteOn2 = m_noteOn3;
            m_noteOn3 = m_noteOn4;
            m_noteOn4 = sm_NoPressedNote;
         } else if ( note == m_noteOn3 )
         {
            m_noteOn3 = m_noteOn4;
            m_noteOn4 = sm_NoPressedNote;
         } else if ( note == m_noteOn4 )
         {
            m_noteOn4 = sm_NoPressedNote;
         }
      }
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void ProcessNotesVirtualAnalog ()
   {
      uint8  i       = 0;
      int8   loopInc = 0;
      if ( eMOOG_MODE == m_notePriorityMode )
      {
         i = sm_maxNumberOfNotes;
         loopInc = -1;
      }
      else if ( eARP_MODE == m_notePriorityMode )
      {
         i = 0;
         loopInc = 1;
      }
      else
      {
         if ( m_currentNote != m_lastPressedNote )
         {
            m_currentNote = m_lastPressedNote;
            if ( sm_NoPressedNote != m_currentNote )
            {
               SetTriggerOn();
            }
         }
      }

      if ( eMOOG_MODE == m_notePriorityMode || eARP_MODE == m_notePriorityMode )
      {
         int8 maxNumberOfLoops = sm_maxNumberOfNotes;
         m_numberOfNotesOn = 0;

         for (; i <= maxNumberOfLoops; i += loopInc)
         {
            if ( m_notesOn[i] != sm_NoPressedNote )
            {
               if (m_currentNote != m_notesOn[i] && 0 == m_numberOfNotesOn)
               {
                  m_currentNote = m_notesOn[i];
                  SetTriggerOn();
                  SetGateOn();
               }
               m_numberOfNotesOn++;
            }
         }
         if (0 == m_numberOfNotesOn)
         {
            m_currentNote = sm_NoPressedNote;
            SetGateOff();
         }

      }
      else
      {
         if ( sm_NoPressedNote != m_currentNote )
         {
            m_numberOfNotesOn = 1;
            SetGateOn();
         }
         else
         {
            m_numberOfNotesOn = 0;
            SetGateOff();
         }
      }
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   void SetNotePriorityMode ( EnNotePriorityMode notePriorityMode )
   {
      m_notePriorityMode = notePriorityMode;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   bool  getTrigger ()
   {
      bool returnValue = m_trigger;
      m_trigger = false;
      return returnValue;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   bool  getGate ()
   {
      return m_gate;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   int8 getCurrentNote() const
   {
      return m_currentNote;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   int8 getHighestSupportedNote() const
   {
      return m_highestSupportedNote;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   int8 getLowestSupportedNote() const
   {
      return m_lowestSupportedNote;
   }


private:

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void  SetTriggerOn ()
   {
      m_trigger = true;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void  SetGateOn ()
   {
      m_gate    = true;
   }

   ////////////////////////////////////////////////////////////////////////////
   /// @brief
   ////////////////////////////////////////////////////////////////////////////
   inline void  SetGateOff ()
   {
      m_gate    = false;
   }

   int8                 m_notesOn [ sm_maxNumberOfNotes + 1 ];
   bool                 m_trigger;
   bool                 m_gate;
   EnNotePriorityMode   m_notePriorityMode;
   int8                 m_noteOn1;
   int8                 m_noteOn2;
   int8                 m_noteOn3;
   int8                 m_noteOn4;
   int8                 m_currentNote;
   int8                 m_lastPressedNote;
   uint8                m_numberOfNotesOn;
   int8                 m_lowestSupportedNote;
   int8                 m_highestSupportedNote;
};



#endif




