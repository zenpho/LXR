/*
 * MidiParser.c
 *
 *  Created on: 02.04.2012
 * ------------------------------------------------------------------------------------------------------------------------
 *  Copyright 2013 Julian Schmidt
 *  Julian@sonic-potions.com
 * ------------------------------------------------------------------------------------------------------------------------
 *  This file is part of the Sonic Potions LXR drumsynth firmware.
 * ------------------------------------------------------------------------------------------------------------------------
 *  Redistribution and use of the LXR code or any derivative works are permitted
 *  provided that the following conditions are met:
 *
 *       - The code may not be sold, nor may it be used in a commercial product or activity.
 *
 *       - Redistributions that are modified from the original source must include the complete
 *         source code, including the source code for all components used by a binary built
 *         from the modified sources. However, as a special exception, the source code distributed
 *         need not include anything that is normally distributed (in either source or binary form)
 *         with the major components (compiler, kernel, and so on) of the operating system on which
 *         the executable runs, unless that component itself accompanies the executable.
 *
 *       - Redistributions must reproduce the above copyright notice, this list of conditions and the
 *         following disclaimer in the documentation and/or other materials provided with the distribution.
 * ------------------------------------------------------------------------------------------------------------------------
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ------------------------------------------------------------------------------------------------------------------------
 */


#include "MidiParser.h"
#include "MidiNoteNumbers.h"
#include "DrumVoice.h"
#include "Snare.h"
#include "config.h"
#include "HiHat.h"
#include "CymbalVoice.h"
#include "Uart.h"
#include "sequencer.h"
#include "clockSync.h"
#include "mixer.h"
#include "valueShaper.h"
#include "modulationNode.h"
#include "frontPanelParser.h"
#include "usb_manager.h"
#define MORPH_CC        0xac
#define BANK_CHANGE_CC  0xad
#define PARAM_CC        0xae
#define PARAM_CC2       0xaf
#define VOICE_CC			0xb4
   #define BANK_1 0x01
   #define BANK_2 0x02
   #define BANK_3 0x04
   #define BANK_4 0x08
   #define BANK_5 0x10
   #define BANK_6 0x20
   #define BANK_7 0x40
   #define BANK_GLOBAL 0x7F
// banks 1-6 plus global stack to allow for multiple voices stacked on the same
// MIDI channel to respond to the same bank change command

// above BANK_GLOBAL it doesn't matter - we reset the command anyway
#define MORPH_OP 0x81
// there is space in here to add more long operations - pattern change
// must have the highest priority
#define PATTERN_CHANGE_OP 0xAF
#define NULL_OP 0x00

uint8_t midiParser_originalCcValues[0xff];

MidiMsg midi_midiCache[256];
MidiMsg midi_midiKit[256];
uint8_t midi_midiCacheAvailable[256];
uint8_t midi_midiLfoCache[6];
uint8_t midi_kitLfoCache[6];
uint8_t midi_midiLfoCacheAvailable[6];
uint8_t midi_midiVeloCache[6];
uint8_t midi_kitVeloCache[6];
uint8_t midi_midiVeloCacheAvailable[6];

uint8_t midi_envPosition[6];

uint8_t midi_unused;

// this will be set to some value if we are ignoring all mtc messages until the next 0 message
static uint8_t midiParser_mtcIgnore=1;
static uint32_t volatile midiParser_lastMtcReceived=0x0;
static uint8_t midiParser_mtcIsRunning=0;

void midi_clearCache()
{
   uint16_t i;
   for (i=0;i<256;i++)
   {
      midi_midiCacheAvailable[i]=0;
   }
   for(i=0;i<6;i++)
   {
      midi_midiLfoCache[i]=0;
      midi_midiLfoCacheAvailable[i]=0;
      midi_midiVeloCache[i]=0;
      midi_midiVeloCacheAvailable[i]=0;
   }
}

static union {
   uint8_t value;
   struct {
      unsigned usb2midi:1;
      unsigned usb2usb:1;   // not used
      unsigned midi2midi:1;
      unsigned midi2usb:1;
      unsigned :4;
   } route;
} midiParser_routing = {0};

uint8_t midiParser_txRxFilter = 0xFF;

enum State
{
MIDI_STATUS,  		// waiting for status byte
MIDI_DATA1,  		// waiting for data byte1
MIDI_DATA2,  		// waiting for data byte2
SYSEX_DATA,  		// read sysex data byte
IGNORE				// set when unknown status byte received, stays in ignore mode until next known status byte
};

// 2^(1/12) factor for 1 semitone
#define SEMITONE_UP 1.0594630943592952645618252949463f
#define SEMITONE_DOWN 0.94387431268169349664191315666753f

#define NUM_LFO 6
uint8_t midiParser_selectedLfoVoice[NUM_LFO] = {0,0,0,0,0,0};

#if 0
// -- AS for debugging
void midiDebugSend(uint8_t b1, uint8_t b2)
{
uart_sendMidiByte(0xF2);
uart_sendMidiByte(b1&0x7F);
uart_sendMidiByte(b2&0x7F);
}
#endif

//----------------------------------------------------------
#if 0
inline uint16_t calcSlopeEgTime(uint8_t data2)
{
float val = (data2+1)/128.f;
return data2>0?val*val*data2*128:1;
}
#endif
//-----------------------------------------------------------
static inline float calcPitchModAmount(uint8_t data2)
{
   const float val = data2/127.f;
   return val*val*PITCH_AMOUNT_FACTOR;
}

//-----------------------------------------------------------
// vars
//-----------------------------------------------------------
uint8_t midi_MidiChannels[8];	// the currently selected midi channel for each voice (element 7 is global channel)

//--AS note overrides for each voice
uint8_t midi_NoteOverride[7];
//uint8_t midi_mode; //--AS not used anymore
MidiMsg midiMsg_tmp;				// buffer message where the incoming data is stored
// these two are used only when building up a midi message
//static uint8_t msgLength;					// number of following data bytes expected for current status
static uint8_t parserState = IGNORE;	// state of the parser state machine. Set to what it's expecting next
									// we set it to ignore initially so that any random data we get before
									// a valid msg header is ignored

//-----------------------------------------------------------
//takes a midi value from 0 to 127 and return +/- 50 cent detune factor
float midiParser_calcDetune(uint8_t value)
{
//linear interpolation between 1(no change) and semitone up/down)
   float frac = (value/127.f -0.5f);
   float cent = 1;
   if(cent>=0)
   {
      cent += frac*(SEMITONE_UP - 1);
   }
   else
   {
      cent += frac*(SEMITONE_UP - 1);
   }
   return cent;
}
//-----------------------------------------------------------

/** handle all incoming CCs and invoke action*/
void midiParser_LXRccHandler(MidiMsg msg, uint8_t updateOriginalValue)
{
   if(msg.status==MIDI_CC)
   {
      const uint16_t paramNr = msg.data1-1;
      if(updateOriginalValue) {
         midiParser_originalCcValues[msg.data1] = msg.data2;
      }
   
      switch(msg.data1)
      {      
         case VOL_SLOPE1:
            slopeEg2_setSlope(&voiceArray[0].oscVolEg,msg.data2);
            break;
         case PITCH_SLOPE1:
            DecayEg_setSlope(&voiceArray[0].oscPitchEg,msg.data2);
            break;
      
         case VOL_SLOPE2:
            slopeEg2_setSlope(&voiceArray[1].oscVolEg,msg.data2);
            break;
         case PITCH_SLOPE2:
            DecayEg_setSlope(&voiceArray[1].oscPitchEg,msg.data2);
            break;
      
         case VOL_SLOPE3:
            slopeEg2_setSlope(&voiceArray[2].oscVolEg,msg.data2);
            break;
         case PITCH_SLOPE3:
            DecayEg_setSlope(&voiceArray[2].oscPitchEg,msg.data2);
            break;
      
         case PITCH_SLOPE4:
            DecayEg_setSlope(&snareVoice.oscPitchEg,msg.data2);
            break;
         case VOL_SLOPE6:
            slopeEg2_setSlope(&hatVoice.oscVolEg,msg.data2);
            break;
      
         case FILTER_FREQ_DRUM1:
         case FILTER_FREQ_DRUM2:
         case FILTER_FREQ_DRUM3:
            {
               const float f = msg.data2/127.f;
            //exponential full range freq
               SVF_directSetFilterValue(&voiceArray[msg.data1-FILTER_FREQ_DRUM1].filter,valueShaperF2F(f,FILTER_SHAPER) );
            }
            break;
         case RESO_DRUM1:
         case RESO_DRUM2:
         case RESO_DRUM3:
            SVF_setReso(&voiceArray[msg.data1-RESO_DRUM1].filter, msg.data2/127.f);
            break;
      
         case F_OSC1_COARSE:
            {
            //clear upper nibble
               voiceArray[0].osc.midiFreq &= 0x00ff;
            //set upper nibble
               voiceArray[0].osc.midiFreq |= msg.data2 << 8;
               osc_recalcFreq(&voiceArray[0].osc);
            }
            break;
         case F_OSC2_COARSE:
            {
            //clear upper nibble
               voiceArray[1].osc.midiFreq &= 0x00ff;
            //set upper nibble
               voiceArray[1].osc.midiFreq |= msg.data2 << 8;
               osc_recalcFreq(&voiceArray[1].osc);
            }
            break;
         case F_OSC3_COARSE:
            {
            //clear upper nibble
               voiceArray[2].osc.midiFreq &= 0x00ff;
            //set upper nibble
               voiceArray[2].osc.midiFreq |= msg.data2 << 8;
               osc_recalcFreq(&voiceArray[2].osc);
            }
            break;
         case F_OSC4_COARSE:
            {
            //clear upper nibble
               snareVoice.osc.midiFreq &= 0x00ff;
            //set upper nibble
               snareVoice.osc.midiFreq |= msg.data2 << 8;
               osc_recalcFreq(&snareVoice.osc);
            }
            break;
         case F_OSC5_COARSE:
            {
            //clear upper nibble
               cymbalVoice.osc.midiFreq &= 0x00ff;
            //set upper nibble
               cymbalVoice.osc.midiFreq |= msg.data2 << 8;
               osc_recalcFreq(&cymbalVoice.osc);
            }
            break;
      
         case F_OSC1_FINE:
            {
            //clear lower nibble
               voiceArray[0].osc.midiFreq &= 0xff00;
            //set lower nibble
               voiceArray[0].osc.midiFreq |= msg.data2;
               osc_recalcFreq(&voiceArray[0].osc);
            }
            break;
         case F_OSC2_FINE:
            {
            //clear lower nibble
               voiceArray[1].osc.midiFreq &= 0xff00;
            //set lower nibble
               voiceArray[1].osc.midiFreq |= msg.data2;
               osc_recalcFreq(&voiceArray[1].osc);
            }
            break;
         case F_OSC3_FINE:
            {
            //clear lower nibble
               voiceArray[2].osc.midiFreq &= 0xff00;
            //set lower nibble
               voiceArray[2].osc.midiFreq |= msg.data2;
               osc_recalcFreq(&voiceArray[2].osc);
            }
            break;
         case F_OSC4_FINE:
            {
            //clear lower nibble
               snareVoice.osc.midiFreq &= 0xff00;
            //set lower nibble
               snareVoice.osc.midiFreq |= msg.data2;
               osc_recalcFreq(&snareVoice.osc);
            }
            break;
      
         case F_OSC5_FINE:
            {
            //clear lower nibble
               cymbalVoice.osc.midiFreq &= 0xff00;
            //set lower nibble
               cymbalVoice.osc.midiFreq |= msg.data2;
               osc_recalcFreq(&cymbalVoice.osc);
            }
            break;
      
         case F_OSC6_FINE:
            {
            //clear lower nibble
               hatVoice.osc.midiFreq &= 0xff00;
            //set lower nibble
               hatVoice.osc.midiFreq |= msg.data2;
               osc_recalcFreq(&hatVoice.osc);
            }
            break;
      
         case OSC_WAVE_DRUM1:
            voiceArray[0].osc.waveform = msg.data2;
            break;
      
         case MOD_WAVE_DRUM1:
            voiceArray[0].modOsc.waveform = msg.data2;
            break;
      
         case OSC_WAVE_DRUM2:
            voiceArray[1].osc.waveform = msg.data2;
            break;
      
         case MOD_WAVE_DRUM2:
            voiceArray[1].modOsc.waveform = msg.data2;
            break;
      
         case OSC_WAVE_DRUM3:
            voiceArray[2].osc.waveform = msg.data2;
            break;
      
         case MOD_WAVE_DRUM3:
            voiceArray[2].modOsc.waveform = msg.data2;
            break;
      
         case OSC_WAVE_SNARE:
            snareVoice.osc.waveform = msg.data2;
            break;
      
      
         case OSC1_DIST:
         #if USE_FILTER_DRIVE
         voiceArray[0].filter.drive = 0.5f + (msg.data2/127.f) *6;
         #else
            setDistortionShape(&voiceArray[0].distortion,msg.data2);
         #endif
            break;
         case OSC2_DIST:
         #if USE_FILTER_DRIVE
         voiceArray[1].filter.drive = 0.5f + (msg.data2/127.f)*6;
         #else
            setDistortionShape(&voiceArray[1].distortion,msg.data2);
         #endif
            break;
         case OSC3_DIST:
         #if USE_FILTER_DRIVE
         voiceArray[3].filter.drive = 0.5f + (msg.data2/127.f)*6;
         #else
            setDistortionShape(&voiceArray[2].distortion,msg.data2);
         #endif
            break;
      
         // "velo" is misleading, these are AEG controls
         case VELOA1:
            slopeEg2_setAttack(&voiceArray[0].oscVolEg,msg.data2,AMP_EG_SYNC);
            break;
      
         case VELOD1:
            {
               slopeEg2_setDecay(&voiceArray[0].oscVolEg,msg.data2,AMP_EG_SYNC);
            }
            break;
      
         case PITCHD1:
            {
               DecayEg_setDecay(&voiceArray[0].oscPitchEg,msg.data2);
            }
            break;
      
         case MODAMNT1:
            voiceArray[0].egPitchModAmount = calcPitchModAmount(msg.data2);
            break;
      
         case FMAMNT1:
            voiceArray[0].fmModAmount = msg.data2/127.f;
            break;
      
         case FMDTN1:
         //clear upper nibble
            voiceArray[0].modOsc.midiFreq &= 0x00ff;
         //set upper nibble
            voiceArray[0].modOsc.midiFreq |= msg.data2 << 8;
            osc_recalcFreq(&voiceArray[0].modOsc);
            break;
      
         case VOL1:
            voiceArray[0].vol = msg.data2/127.f;
            break;
      
         case PAN1:
            setPan(0,msg.data2);
            break;
      
         // "velo" is misleading, these are AEG controls
         case VELOA2:
            {
               slopeEg2_setAttack(&voiceArray[1].oscVolEg,msg.data2,AMP_EG_SYNC);
            }
            break;
      
         case VELOD2:
            {
               slopeEg2_setDecay(&voiceArray[1].oscVolEg,msg.data2,AMP_EG_SYNC);
            }
            break;
      
         case PITCHD2:
            {
               DecayEg_setDecay(&voiceArray[1].oscPitchEg,msg.data2);
            }
            break;
      
         case MODAMNT2:
            voiceArray[1].egPitchModAmount = calcPitchModAmount(msg.data2);
            break;
      
         case FMAMNT2:
            voiceArray[1].fmModAmount = msg.data2/127.f;
            break;
      
         case FMDTN2:
         //clear upper nibble
            voiceArray[1].modOsc.midiFreq &= 0x00ff;
         //set upper nibble
            voiceArray[1].modOsc.midiFreq |= msg.data2 << 8;
            osc_recalcFreq(&voiceArray[1].modOsc);
            break;
      
         case VOL2:
            voiceArray[1].vol = msg.data2/127.f;
            break;
      
         case PAN2:
            setPan(1,msg.data2);
            break;
      
         // "velo" is misleading, these are AEG controls
         case VELOA3:
            {
               slopeEg2_setAttack(&voiceArray[2].oscVolEg,msg.data2,AMP_EG_SYNC);
            }
            break;
      
         case VELOD3:
            {
               slopeEg2_setDecay(&voiceArray[2].oscVolEg,msg.data2,AMP_EG_SYNC);
            }
            break;
      
         case PITCHD3:
            {
               DecayEg_setDecay(&voiceArray[2].oscPitchEg,msg.data2);
            }
            break;
      
         case MODAMNT3:
            voiceArray[2].egPitchModAmount = calcPitchModAmount(msg.data2);
            break;
      
         case FMAMNT3:
            voiceArray[2].fmModAmount = msg.data2/127.f;
            break;
      
         case FMDTN3:
         	//clear upper nibble
            voiceArray[2].modOsc.midiFreq &= 0x00ff;
         	//set upper nibble
            voiceArray[2].modOsc.midiFreq |= msg.data2 << 8;
            osc_recalcFreq(&voiceArray[2].modOsc);
            break;
      
         case VOL3:
            voiceArray[2].vol = msg.data2/127.f;
            break;
      
         case PAN3:
            setPan(2,msg.data2);
            break;
      
      	//snare
         case VOL4:
            snareVoice.vol = msg.data2/127.f;
            break;
      
         case PAN4:
            Snare_setPan(msg.data2);
         
            break;
      
         case SNARE_NOISE_F:
            snareVoice.noiseOsc.freq = msg.data2/127.f*22000;
         	//TODO respond to midi note
            break;
          
         // "velo" is misleading, these are AEG controls
         case VELOA4:
            {
               slopeEg2_setAttack(&snareVoice.oscVolEg,msg.data2,false);
            }
            break;
         case VELOD4:
            {
               slopeEg2_setDecay(&snareVoice.oscVolEg,msg.data2,false);
            }
         
            break;
         case PITCHD4:
            {
               DecayEg_setDecay(&snareVoice.oscPitchEg,msg.data2);
            }
         
            break;
         case MODAMNT4:
            snareVoice.egPitchModAmount = calcPitchModAmount(msg.data2);
            break;
         case SNARE_FILTER_F:
            {
            #if USE_PEAK
            		peak_setFreq(&snareVoice.filter, msg.data2/127.f*20000.f);
            #else
               const float f = msg.data2/127.f;
            		//exponential full range freq
               SVF_directSetFilterValue(&snareVoice.filter,valueShaperF2F(f,FILTER_SHAPER) );
            #endif
            }
            break;
      
         case SNARE_RESO:
         #if USE_PEAK
         	peak_setGain(&snareVoice.filter, msg.data2/127.f);
         #else
            SVF_setReso(&snareVoice.filter, msg.data2/127.f);
         #endif
            break;
      
         case SNARE_MIX:
            snareVoice.mix = msg.data2/127.f;
            break;
      
      	//snare 2
         case VOL5:
            cymbalVoice.vol = msg.data2/127.f;
            break;
      
         case PAN5:
            Cymbal_setPan(msg.data2);
         
            break;
      
         // "velo" is misleading, these are AEG controls
         case VELOA5:
            {
               slopeEg2_setAttack(&cymbalVoice.oscVolEg,msg.data2,false);
            }
         
            break;
         case VELOD5:
            {
               slopeEg2_setDecay(&cymbalVoice.oscVolEg,msg.data2,false);
            }
            break;
      
      
         case CYM_WAVE1:
            cymbalVoice.osc.waveform = msg.data2;
            break;
      
         case CYM_WAVE2:
            cymbalVoice.modOsc.waveform = msg.data2;
            break;
      
         case CYM_WAVE3:
            cymbalVoice.modOsc2.waveform = msg.data2;
            break;
      
         case CYM_MOD_OSC_F1:
         	//clear upper nibble
            cymbalVoice.modOsc.midiFreq &= 0x00ff;
         	//set upper nibble
            cymbalVoice.modOsc.midiFreq |= msg.data2 << 8;
            osc_recalcFreq(&cymbalVoice.modOsc);
         
            break;
         case CYM_MOD_OSC_F2:
         	//clear upper nibble
            cymbalVoice.modOsc2.midiFreq &= 0x00ff;
         	//set upper nibble
            cymbalVoice.modOsc2.midiFreq |= msg.data2 << 8;
            osc_recalcFreq(&cymbalVoice.modOsc2);
            break;
         case CYM_MOD_OSC_GAIN1:
            cymbalVoice.fmModAmount1 = msg.data2/127.f;
            break;
         case CYM_MOD_OSC_GAIN2:
            cymbalVoice.fmModAmount2 = msg.data2/127.f;
            break;
      
         case CYM_FIL_FREQ:
            {
               const float f = msg.data2/127.f;
            //exponential full range freq
               SVF_directSetFilterValue(&cymbalVoice.filter,valueShaperF2F(f,FILTER_SHAPER) );
            }
            break;
      
         case CYM_RESO:
            SVF_setReso(&cymbalVoice.filter, msg.data2/127.f);
            break;
      
         case CYM_REPEAT:
            cymbalVoice.oscVolEg.repeat = msg.data2;
            break;
         case CYM_SLOPE:
            slopeEg2_setSlope(&cymbalVoice.oscVolEg,msg.data2);
            break;
      
      	// hat
         case WAVE1_HH:
            hatVoice.osc.waveform = msg.data2;
            break;
         case WAVE2_HH:
            hatVoice.modOsc.waveform = msg.data2;
            break;
         case WAVE3_HH:
            hatVoice.modOsc2.waveform = msg.data2;
            break;
      
         // "velo" is misleading, these are AEG controls
         // D6=closedhh, D6_OPEN=openhh
         case VELOD6:
            hatVoice.decayClosed = slopeEg2_calcDecay(msg.data2);
            break;
      
         case VELOD6_OPEN:
            hatVoice.decayOpen = slopeEg2_calcDecay(msg.data2);
            break;
      
         case HAT_FILTER_F:
            {
               const float f = msg.data2/127.f;
            //exponential full range freq
               SVF_directSetFilterValue(&hatVoice.filter,valueShaperF2F(f,FILTER_SHAPER) );
            }
            break;
      
         case HAT_RESO:
            SVF_setReso(&hatVoice.filter, msg.data2/127.f);
            break;
      
         case VOL6:
            hatVoice.vol = msg.data2/127.f;
            break;
      
         case PAN6:
            HiHat_setPan(msg.data2);
            break;
      
         case F_OSC6_COARSE:
            {
            //clear upper nibble
               hatVoice.osc.midiFreq &= 0x00ff;
            //set upper nibble
               hatVoice.osc.midiFreq |= msg.data2 << 8;
               osc_recalcFreq(&hatVoice.osc);
            }
            break;
      
         case MOD_OSC_F1:
         	//clear upper nibble
            hatVoice.modOsc.midiFreq &= 0x00ff;
         	//set upper nibble
            hatVoice.modOsc.midiFreq |= msg.data2 << 8;
            osc_recalcFreq(&hatVoice.modOsc);
            break;
      
         case MOD_OSC_F2:
         	//clear upper nibble
            hatVoice.modOsc2.midiFreq &= 0x00ff;
         	//set upper nibble
            hatVoice.modOsc2.midiFreq |= msg.data2 << 8;
            osc_recalcFreq(&hatVoice.modOsc2);
            break;
      
         case MOD_OSC_GAIN1:
            hatVoice.fmModAmount1 = msg.data2/127.f;
            break;
      
         case MOD_OSC_GAIN2:
            hatVoice.fmModAmount2 = msg.data2/127.f;
            break;
      
         case VELOA6:
            slopeEg2_setAttack(&hatVoice.oscVolEg,msg.data2,false);
            break;
      
         case REPEAT1:
            snareVoice.oscVolEg.repeat = msg.data2;
            break;
      
      
         case EG_SNARE1_SLOPE:
            slopeEg2_setSlope(&snareVoice.oscVolEg,msg.data2);
            break;
      
         case SNARE_DISTORTION:
            setDistortionShape(&snareVoice.distortion,msg.data2);
            break;
      
         case CYMBAL_DISTORTION:
            setDistortionShape(&cymbalVoice.distortion,msg.data2);
            break;
      
         case HAT_DISTORTION:
            setDistortionShape(&hatVoice.distortion,msg.data2);
            break;
      
         case FREQ_LFO1:
         case FREQ_LFO2:
         case FREQ_LFO3:
            lfo_setFreq(&voiceArray[msg.data1-FREQ_LFO1].lfo,msg.data2);
            break;
         case FREQ_LFO4:
            lfo_setFreq(&snareVoice.lfo,msg.data2);
            break;
         case FREQ_LFO5:
            lfo_setFreq(&cymbalVoice.lfo,msg.data2);
            break;
         case FREQ_LFO6:
            lfo_setFreq(&hatVoice.lfo,msg.data2);
            break;
      
         case AMOUNT_LFO1:
         case AMOUNT_LFO2:
         case AMOUNT_LFO3:
            voiceArray[msg.data1-AMOUNT_LFO1].lfo.modTarget.amount = msg.data2/127.f;
            break;
         case AMOUNT_LFO4:
            snareVoice.lfo.modTarget.amount = msg.data2/127.f;
            break;
         case AMOUNT_LFO5:
            cymbalVoice.lfo.modTarget.amount = msg.data2/127.f;
            break;
         case AMOUNT_LFO6:
            hatVoice.lfo.modTarget.amount = msg.data2/127.f;
            break;
      
         case VOICE_DECIMATION1:
         case VOICE_DECIMATION2:
         case VOICE_DECIMATION3:
         case VOICE_DECIMATION4:
         case VOICE_DECIMATION5:
         case VOICE_DECIMATION6:
         case VOICE_DECIMATION_ALL:
            mixer_decimation_rate[msg.data1-VOICE_DECIMATION1] = valueShaperI2F(msg.data2,-0.7f);
            break;
      
         default:
            break;
      }
      modNode_originalValueChanged(paramNr);
   } //msg.status == MIDI_CC
   
   else if(msg.status==MIDI_CC_2) //MIDI_CC_2 for parameters above 127 // ? FRONT_CC_LFO_TARGET
   {
      const uint16_t paramNr = msg.data1+1 + 127;
   
      if(updateOriginalValue) {
         midiParser_originalCcValues[paramNr] = msg.data2;
      }
      switch(msg.data1)
      {
      
         case CC2_TRANS1_WAVE:
            transient_setWaveform(&voiceArray[0].transGen, msg.data2);
            break;
      
         case CC2_TRANS1_VOL:
            voiceArray[0].transGen.volume = msg.data2/127.f;
            break;
      
         case CC2_TRANS1_FREQ:
            // range about  0.25 to 4 => 1/4 to 1*4
            voiceArray[0].transGen.pitch = 1.f + ((msg.data2/33.9f)-0.75f) ;
            break;
      
         case CC2_TRANS2_WAVE:
            transient_setWaveform(&voiceArray[1].transGen, msg.data2);
            break;
      
         case CC2_TRANS2_VOL:
            voiceArray[1].transGen.volume = msg.data2/127.f;
            break;
      
         case CC2_TRANS2_FREQ:
            // range about  0.25 to 4 => 1/4 to 1*4
            voiceArray[1].transGen.pitch = 1.f + ((msg.data2/33.9f)-0.75f) ;
            break;
      
         case CC2_TRANS3_WAVE:
            transient_setWaveform(&voiceArray[2].transGen, msg.data2);
            break;
      
         case CC2_TRANS3_VOL:
            voiceArray[2].transGen.volume = msg.data2/127.f;
            break;
      
         case CC2_TRANS3_FREQ:
            // range about  0.25 to 4 => 1/4 to 1*4
            voiceArray[2].transGen.pitch = 1.f + ((msg.data2/33.9f)-0.75f) ;
            break;
      
         case CC2_TRANS4_WAVE:
            transient_setWaveform(&snareVoice.transGen, msg.data2);
            break;
      
         case CC2_TRANS4_VOL:
            snareVoice.transGen.volume = msg.data2/127.f;
            break;
         case CC2_TRANS4_FREQ:
            // range about  0.25 to 4 => 1/4 to 1*4
            snareVoice.transGen.pitch = 1.f + ((msg.data2/33.9f)-0.75f) ;
            break;
      
         case CC2_TRANS5_WAVE:
            transient_setWaveform(&cymbalVoice.transGen, msg.data2);
            break;
      
         case CC2_TRANS5_VOL:
            cymbalVoice.transGen.volume = msg.data2/127.f;
            break;
         case CC2_TRANS5_FREQ:
            // range about  0.25 to 4 => 1/4 to 1*4
            cymbalVoice.transGen.pitch = 1.f + ((msg.data2/33.9f)-0.75f) ;
            break;
      
         case CC2_TRANS6_WAVE:
            transient_setWaveform(&hatVoice.transGen, msg.data2);
            break;
      
         case CC2_TRANS6_VOL:
            hatVoice.transGen.volume = msg.data2/127.f;
            break;
         case CC2_TRANS6_FREQ:
            // range about  0.25 to 4 => 1/4 to 1*4
            hatVoice.transGen.pitch = 1.f + ((msg.data2/33.9f)-0.75f) ;
            break;
      
         case CC2_FILTER_TYPE_1:
         case CC2_FILTER_TYPE_2:
         case CC2_FILTER_TYPE_3:
            // +1 because 0 is filter off which results in silence
            voiceArray[msg.data1-CC2_FILTER_TYPE_1].filterType = msg.data2+1;
            break;
         case CC2_FILTER_TYPE_4:
            snareVoice.filterType = msg.data2 + 1;
            break;
         case CC2_FILTER_TYPE_5:
         //cymbal filter
            cymbalVoice.filterType = msg.data2+1;
            break;
         case CC2_FILTER_TYPE_6:
         //Hihat filter
            hatVoice.filterType = msg.data2+1;
            break;
      
         case CC2_FILTER_DRIVE_1:
         case CC2_FILTER_DRIVE_2:
         case CC2_FILTER_DRIVE_3:
         #if UNIT_GAIN_DRIVE
         voiceArray[msg.data1-CC2_FILTER_DRIVE_1].filter.drive = (msg.data2/127.f);
         #else
            SVF_setDrive(&voiceArray[msg.data1-CC2_FILTER_DRIVE_1].filter,msg.data2);
         #endif
            break;
         case CC2_FILTER_DRIVE_4:
         #if UNIT_GAIN_DRIVE
         snareVoice.filter.drive = (msg.data2/127.f);
         #else
            SVF_setDrive(&snareVoice.filter, msg.data2);
         #endif
         
            break;
         case CC2_FILTER_DRIVE_5:
         #if UNIT_GAIN_DRIVE
         cymbalVoice.filter.drive = (msg.data2/127.f);
         #else
            SVF_setDrive(&cymbalVoice.filter, msg.data2);
         #endif
         
            break;
         case CC2_FILTER_DRIVE_6:
         #if UNIT_GAIN_DRIVE
         hatVoice.filter.drive = (msg.data2/127.f);
         #else
            SVF_setDrive(&hatVoice.filter, msg.data2);
         #endif
         
            break;
      
         case CC2_MIX_MOD_1:
         case CC2_MIX_MOD_2:
         case CC2_MIX_MOD_3:
            voiceArray[msg.data1-CC2_MIX_MOD_1].mixOscs = msg.data2;
            break;
      
         case CC2_VOLUME_MOD_ON_OFF1:
         case CC2_VOLUME_MOD_ON_OFF2:
         case CC2_VOLUME_MOD_ON_OFF3:
            voiceArray[msg.data1-CC2_VOLUME_MOD_ON_OFF1].volumeMod = msg.data2;
            break;
         case CC2_VOLUME_MOD_ON_OFF4:
            snareVoice.volumeMod = msg.data2;
            break;
         case CC2_VOLUME_MOD_ON_OFF5:
            cymbalVoice.volumeMod = msg.data2;
            break;
         case CC2_VOLUME_MOD_ON_OFF6:
            hatVoice.volumeMod = msg.data2;
            break;
      
         case CC2_VELO_MOD_AMT_1:
         case CC2_VELO_MOD_AMT_2:
         case CC2_VELO_MOD_AMT_3:
         case CC2_VELO_MOD_AMT_4:
         case CC2_VELO_MOD_AMT_5:
         case CC2_VELO_MOD_AMT_6:
            velocityModulators[msg.data1-CC2_VELO_MOD_AMT_1].amount = msg.data2/127.f;
            break;
      
         case CC2_WAVE_LFO1:
         case CC2_WAVE_LFO2:
         case CC2_WAVE_LFO3:
            voiceArray[msg.data1-CC2_WAVE_LFO1].lfo.waveform = msg.data2;
            break;
         case CC2_WAVE_LFO4:
            snareVoice.lfo.waveform = msg.data2;
            break;
         case CC2_WAVE_LFO5:
            cymbalVoice.lfo.waveform = msg.data2;
            break;
         case CC2_WAVE_LFO6:
            hatVoice.lfo.waveform = msg.data2;
            break;
      
         case CC2_RETRIGGER_LFO1:
         case CC2_RETRIGGER_LFO2:
         case CC2_RETRIGGER_LFO3:
            voiceArray[msg.data1-CC2_RETRIGGER_LFO1].lfo.retrigger = msg.data2;
            break;
         case CC2_RETRIGGER_LFO4:
            snareVoice.lfo.retrigger = msg.data2;
            break;
         case CC2_RETRIGGER_LFO5:
            cymbalVoice.lfo.retrigger = msg.data2;
            break;
         case CC2_RETRIGGER_LFO6:
            hatVoice.lfo.retrigger = msg.data2;
            break;
      
         case CC2_SYNC_LFO1:
         case CC2_SYNC_LFO2:
         case CC2_SYNC_LFO3:
            lfo_setSync(&voiceArray[msg.data1-CC2_SYNC_LFO1].lfo, msg.data2);
            break;
         case CC2_SYNC_LFO4:
            lfo_setSync(&snareVoice.lfo, msg.data2);
            break;
         case CC2_SYNC_LFO5:
            lfo_setSync(&cymbalVoice.lfo, msg.data2);
            break;
         case CC2_SYNC_LFO6:
            lfo_setSync(&hatVoice.lfo, msg.data2);
            break;
      
         case CC2_VOICE_LFO1:
         case CC2_VOICE_LFO2:
         case CC2_VOICE_LFO3:
         case CC2_VOICE_LFO4:
         case CC2_VOICE_LFO5:
         case CC2_VOICE_LFO6:
            break;
      
         // see MIDImodTargetHandler()
         case CC2_TARGET_LFO1:
         case CC2_TARGET_LFO2:
         case CC2_TARGET_LFO3:
         case CC2_TARGET_LFO4:
         case CC2_TARGET_LFO5:
         case CC2_TARGET_LFO6:
            break;
      
         case CC2_OFFSET_LFO1:
         case CC2_OFFSET_LFO2:
         case CC2_OFFSET_LFO3:
            voiceArray[msg.data1-CC2_OFFSET_LFO1].lfo.phaseOffset = msg.data2/127.f * 0xffffffff;
            break;
         case CC2_OFFSET_LFO4:
            snareVoice.lfo.phaseOffset = msg.data2/127.f * 0xffffffff;
            break;
         case CC2_OFFSET_LFO5:
            cymbalVoice.lfo.phaseOffset = msg.data2/127.f * 0xffffffff;
            break;
         case CC2_OFFSET_LFO6:
            hatVoice.lfo.phaseOffset = msg.data2/127.f * 0xffffffff;
            break;
      
         case CC2_AUDIO_OUT1:
         case CC2_AUDIO_OUT2:
         case CC2_AUDIO_OUT3:
         case CC2_AUDIO_OUT4:
         case CC2_AUDIO_OUT5:
         case CC2_AUDIO_OUT6:
            mixer_audioRouting[msg.data1-CC2_AUDIO_OUT1] = msg.data2;
            break;
      
         case CC2_ENVELOPE_POSITION_1:
         case CC2_ENVELOPE_POSITION_2:
         case CC2_ENVELOPE_POSITION_3:
         case CC2_ENVELOPE_POSITION_4:
         case CC2_ENVELOPE_POSITION_5:
         case CC2_ENVELOPE_POSITION_6:
         //--AS set the note override for the voice. 0 means use the note value, anything else means
         // that the voice will always play with fixed pitch
            midi_envPosition[msg.data1-CC2_ENVELOPE_POSITION_1] = msg.data2;
            drumVoice_setEnvelope(msg.data1-CC2_ENVELOPE_POSITION_1,msg.data2);
            break;
      
         case CC2_MUTE_1:
         case CC2_MUTE_2:
         case CC2_MUTE_3:
         case CC2_MUTE_4:
         case CC2_MUTE_5:
         case CC2_MUTE_6:
         case CC2_MUTE_7:
            {
               const uint8_t voiceNr = msg.data1 - CC2_MUTE_1;
               if(msg.data2 == 0)
               {
                  seq_setMute(voiceNr,0);
               }
               else
               {
                  seq_setMute(voiceNr,1);
               }
            
            }
            break;
         
         case CC2_MAC1_DST1:       // bc: handled with separate message like LFO, VELO dest's
         case CC2_MAC1_DST2:       // this happens in frontPanelParser
         case CC2_MAC2_DST1:       //? also MIDImodTargetHandler()
         case CC2_MAC2_DST2:
            break;
          
         case CC2_MAC1_DST1_AMT:       // bc: change perf macro destination amounts
            if (msg.data2)
               macroModulators[0].amount = ((msg.data2+1)/64.f)-1;
            else
               macroModulators[0].amount = -1;
            modNode_updateValue(&macroModulators[0],macroModulators[0].lastVal);
            break;
         case CC2_MAC1_DST2_AMT:
            if (msg.data2)
               macroModulators[1].amount = ((msg.data2+1)/64.f)-1;
            else
               macroModulators[1].amount = -1;
            modNode_updateValue(&macroModulators[1],macroModulators[1].lastVal);
            break;
         case CC2_MAC2_DST1_AMT:
            if (msg.data2)
               macroModulators[2].amount = ((msg.data2+1)/64.f)-1;
            else
               macroModulators[2].amount = -1;
            modNode_updateValue(&macroModulators[2],macroModulators[2].lastVal);
            break;
         case CC2_MAC2_DST2_AMT:
            if (msg.data2)
               macroModulators[3].amount = ((msg.data2+1)/64.f)-1;
            else
               macroModulators[3].amount = -1;
            modNode_updateValue(&macroModulators[3],macroModulators[3].lastVal);
            break;            
         default:
            break;
      }
      modNode_originalValueChanged(paramNr);
   }
}


//-----------------------------------------------------------
// this will check whether mtc is running, and if so will
// check to see whether we need to stop the sequencer due to
// lack of recent mtc activity. It will also reset our running indicator
// if the sequencer has stopped for some other reason
void midiParser_checkMtc()
{
   if(!midiParser_mtcIsRunning)
      return;

   if(!seq_isRunning()) {
   // inform mtc that his services are no longer needed
      midiParser_mtcIsRunning=0;
      return;
   }

// at a 24fps framerate (the lowest) we should receive a completed message (we receive one every 2 frames)
// every 83 ms. our tick counter is .25 ms resolution
   if(systick_ticks-midiParser_lastMtcReceived > 100 * 4) { // overestimate, just in case something untoward should happen
   // too much time has elapsed since our last message. mtc has gone away.
      midiParser_mtcIsRunning=0;
   
      uart_sendFrontpanelByte(FRONT_SEQ_CC);
      uart_sendFrontpanelByte(FRONT_SEQ_RUN_STOP);
      uart_sendFrontpanelByte(0);
   // stop the sequencer
      seq_setRunning(0);
   }
}

//------------------------------------------------------
// this fn assumes a valid voice is sent
// do_rec - whether we want to allow it to be recorded and echoed - we will only record
//          to the active track for notes received from the global channel
static void midiParser_noteOn(uint8_t voice, uint8_t note, uint8_t vel, uint8_t do_rec)
{

   if(seq_isTrackMuted(voice))
      return;

// --AS check the midi mode for the voice. If it's "any", trigger that note.
// if it's some other note, trigger the drum voice only if it matches, and in this case
// trigger the default note

   if(midi_NoteOverride[voice] != 0) {
   // looking for specific note
      if(note==midi_NoteOverride[voice])
         note=SEQ_DEFAULT_NOTE; // match found. Play default note
      else
         return; // note does not match. Do nothing
   }
   
   // -bc- don't respond to note-offs, only record
   if (vel)
   {
      voiceControl_noteOn(voice,note,vel);
   }

//Recording Mode - record the note to sequencer and echo to channel of that voice
// (we'd want to hear what's being recorded)
   if(do_rec) {
   
      
   
   // record note if rec is on
      seq_addNote(voice,vel, note);
   
   //if a note is on for that channel send note-off first
   // -bc- is that really necessary? I think seq can deal with overlapping notes
   //seq_midiNoteOff(chan);
   //send the new note to midi/usb out
   // --AS todo a user played note will end up being turned off if a pattern switch happens.
   //           to fix this we'd have to differentiate between a user played note and a
   //           note triggered from sequencer, and also recognize note off on user played note and send it.
      if(midi_MidiChannels[voice])
      {
         const uint8_t chan=midi_MidiChannels[voice]-1;
         seq_sendMidiNoteOn(chan, note, vel);
      }
   }



}
//------------------------------------------------------
static void midiParser_noteOff(uint8_t voice, uint8_t note, uint8_t vel, uint8_t do_rec)
{
   vel=0;
   midiParser_noteOn(voice, note, vel, do_rec);
}

int16_t midiParser_MIDItoLXRparamNr(uint8_t MIDIparamNr, uint8_t LXRvoiceNr)
/* map from midi cc per voice channel to range LXR paramNr
  returns I_DUNNO if not found
 
  use with midiParser_LXRccHandler() with MIDI_CC for params<128
  and with MIDI_CC_2 for params>=128
 
  input "MIDIparamNr" range 0 to 127
  input "LXRvoiceNr" range 0 to 6 (0=BD .. 4=CY, 5=CHH, 6=OHH, 7=GLOBAL)
 
  see also frontParser_LXRtoMIDIparamNr() and frontParser_LXRtoMIDIchanlNr()
*/
{
  // special case for voice 6 hh open cc 72
  if( LXRvoiceNr == 6 && MIDIparamNr == ENV_RELEASE )
  {
    return VELOD6_OPEN; // unique decay time for voice 6 open hh
  }
  else if( LXRvoiceNr == 6 )
  {
    LXRvoiceNr = 5; // all other voice 6 params redirect to voice 5 hh
  }

  // special cases for voice 7 i.e. GLOBAL
  if( LXRvoiceNr == 7 && MIDIparamNr == EFFECT_1 )
  {
    return VOICE_DECIMATION_ALL; // cc 12
  }
  
  // general cases - morph is handled directly in midiParser_MIDIccHandler()
  switch( MIDIparamNr )
  {
      case  CHANNEL_VOL:
        if( LXRvoiceNr == 0 ) return VOL1;
        if( LXRvoiceNr == 1 ) return VOL2;
        if( LXRvoiceNr == 2 ) return VOL3;
        if( LXRvoiceNr == 3 ) return VOL4; //snr
        if( LXRvoiceNr == 4 ) return VOL5; //cym+clap
        if( LXRvoiceNr == 5 ) return VOL6; //hh
      
      case  UNDEF_9:
        if( LXRvoiceNr == 0 ) return OSC_WAVE_DRUM1;
        if( LXRvoiceNr == 1 ) return OSC_WAVE_DRUM2;
        if( LXRvoiceNr == 2 ) return OSC_WAVE_DRUM3;
        if( LXRvoiceNr == 3 ) return OSC_WAVE_SNARE;
        if( LXRvoiceNr == 4 ) return CYM_WAVE1; //+clap
        if( LXRvoiceNr == 5 ) return WAVE1_HH;
      case  PAN:
        if( LXRvoiceNr == 0 ) return PAN1;
        if( LXRvoiceNr == 1 ) return PAN2;
        if( LXRvoiceNr == 2 ) return PAN3;
        if( LXRvoiceNr == 3 ) return PAN4; //snr
        if( LXRvoiceNr == 4 ) return PAN5; //cym+clap
        if( LXRvoiceNr == 5 ) return PAN6; //hh
      
      case  EFFECT_1: // 12
        if( LXRvoiceNr == 0 ) return VOICE_DECIMATION1;
        if( LXRvoiceNr == 1 ) return VOICE_DECIMATION2;
        if( LXRvoiceNr == 2 ) return VOICE_DECIMATION3;
        if( LXRvoiceNr == 3 ) return VOICE_DECIMATION4; //snr
        if( LXRvoiceNr == 4 ) return VOICE_DECIMATION5; //cym+clap
        if( LXRvoiceNr == 5 ) return VOICE_DECIMATION6; //hh
      case  EFFECT_2: // 13
        if( LXRvoiceNr == 0 ) return OSC1_DIST;
        if( LXRvoiceNr == 1 ) return OSC2_DIST;
        if( LXRvoiceNr == 2 ) return OSC3_DIST;
        if( LXRvoiceNr == 3 ) return SNARE_DISTORTION;
        if( LXRvoiceNr == 4 ) return CYMBAL_DISTORTION; //+clap
        if( LXRvoiceNr == 5 ) return HAT_DISTORTION;
      case  UNDEF_14:
        if( LXRvoiceNr == 0 ) return F_OSC1_COARSE;
        if( LXRvoiceNr == 1 ) return F_OSC2_COARSE;
        if( LXRvoiceNr == 2 ) return F_OSC3_COARSE;
        if( LXRvoiceNr == 3 ) return F_OSC4_COARSE; //snr
        if( LXRvoiceNr == 4 ) return F_OSC5_COARSE; //cym+clap
        if( LXRvoiceNr == 5 ) return F_OSC6_COARSE; //hh
      case  UNDEF_14_LSB:
        if( LXRvoiceNr == 0 ) return F_OSC1_FINE;
        if( LXRvoiceNr == 1 ) return F_OSC2_FINE;
        if( LXRvoiceNr == 2 ) return F_OSC3_FINE;
        if( LXRvoiceNr == 3 ) return F_OSC4_FINE; //snr
        if( LXRvoiceNr == 4 ) return F_OSC5_FINE; //cym+clap
        if( LXRvoiceNr == 5 ) return F_OSC6_FINE; //hh
      case  GEN_CONTROLLER_16:
        if( LXRvoiceNr == 0 ) return FILTER_FREQ_DRUM1;
        if( LXRvoiceNr == 1 ) return FILTER_FREQ_DRUM2;
        if( LXRvoiceNr == 2 ) return FILTER_FREQ_DRUM3;
        if( LXRvoiceNr == 3 ) return SNARE_FILTER_F;
        if( LXRvoiceNr == 4 ) return CYM_FIL_FREQ; //+clap
        if( LXRvoiceNr == 5 ) return HAT_FILTER_F;
      case  GEN_CONTROLLER_17:
        if( LXRvoiceNr == 0 ) return RESO_DRUM1;
        if( LXRvoiceNr == 1 ) return RESO_DRUM2;
        if( LXRvoiceNr == 2 ) return RESO_DRUM3;
        if( LXRvoiceNr == 3 ) return SNARE_RESO;
        if( LXRvoiceNr == 4 ) return CYM_RESO; //+clap
        if( LXRvoiceNr == 5 ) return HAT_RESO;
      case  GEN_CONTROLLER_18:
        if( LXRvoiceNr == 0 ) return 128+CC2_FILTER_DRIVE_1;
        if( LXRvoiceNr == 1 ) return 128+CC2_FILTER_DRIVE_2;
        if( LXRvoiceNr == 2 ) return 128+CC2_FILTER_DRIVE_3;
        if( LXRvoiceNr == 3 ) return 128+CC2_FILTER_DRIVE_4;
        if( LXRvoiceNr == 4 ) return 128+CC2_FILTER_DRIVE_5;
        if( LXRvoiceNr == 5 ) return 128+CC2_FILTER_DRIVE_6;
      case  GEN_CONTROLLER_19:
        if( LXRvoiceNr == 0 ) return 128+CC2_FILTER_TYPE_1;
        if( LXRvoiceNr == 1 ) return 128+CC2_FILTER_TYPE_2;
        if( LXRvoiceNr == 2 ) return 128+CC2_FILTER_TYPE_3;
        if( LXRvoiceNr == 3 ) return 128+CC2_FILTER_TYPE_4;
        if( LXRvoiceNr == 4 ) return 128+CC2_FILTER_TYPE_5;
        if( LXRvoiceNr == 5 ) return 128+CC2_FILTER_TYPE_6;
      case  UNDEF_20:
        if( LXRvoiceNr == 3 ) return SNARE_MIX;
      case  UNDEF_21:
        if( LXRvoiceNr == 3 ) return SNARE_NOISE_F;
      case  UNDEF_22:
        if( LXRvoiceNr == 0 ) return 128+CC2_MIX_MOD_1;
        if( LXRvoiceNr == 1 ) return 128+CC2_MIX_MOD_2;
        if( LXRvoiceNr == 2 ) return 128+CC2_MIX_MOD_3;
      case  UNDEF_23:
        if( LXRvoiceNr == 0 ) return 128+CC2_VOLUME_MOD_ON_OFF1;
        if( LXRvoiceNr == 1 ) return 128+CC2_VOLUME_MOD_ON_OFF2;
        if( LXRvoiceNr == 2 ) return 128+CC2_VOLUME_MOD_ON_OFF3;
        if( LXRvoiceNr == 3 ) return 128+CC2_VOLUME_MOD_ON_OFF4;
        if( LXRvoiceNr == 4 ) return 128+CC2_VOLUME_MOD_ON_OFF5;
        if( LXRvoiceNr == 5 ) return 128+CC2_VOLUME_MOD_ON_OFF6;
      case  UNDEF_24:
        if( LXRvoiceNr == 0 ) return 128+CC2_VELO_MOD_AMT_1;
        if( LXRvoiceNr == 1 ) return 128+CC2_VELO_MOD_AMT_2;
        if( LXRvoiceNr == 2 ) return 128+CC2_VELO_MOD_AMT_3;
        if( LXRvoiceNr == 3 ) return 128+CC2_VELO_MOD_AMT_4;
        if( LXRvoiceNr == 4 ) return 128+CC2_VELO_MOD_AMT_5;
        if( LXRvoiceNr == 5 ) return 128+CC2_VELO_MOD_AMT_6;
      
      case  UNDEF_27:
        if( LXRvoiceNr == 0 ) return 128+CC2_TRANS1_WAVE;
        if( LXRvoiceNr == 1 ) return 128+CC2_TRANS2_WAVE;
        if( LXRvoiceNr == 2 ) return 128+CC2_TRANS3_WAVE;
        if( LXRvoiceNr == 3 ) return 128+CC2_TRANS4_WAVE;
        if( LXRvoiceNr == 4 ) return 128+CC2_TRANS5_WAVE;
        if( LXRvoiceNr == 5 ) return 128+CC2_TRANS6_WAVE;
      case  UNDEF_28:
        if( LXRvoiceNr == 0 ) return 128+CC2_TRANS1_FREQ;
        if( LXRvoiceNr == 1 ) return 128+CC2_TRANS2_FREQ;
        if( LXRvoiceNr == 2 ) return 128+CC2_TRANS3_FREQ;
        if( LXRvoiceNr == 3 ) return 128+CC2_TRANS4_FREQ;
        if( LXRvoiceNr == 4 ) return 128+CC2_TRANS5_FREQ;
        if( LXRvoiceNr == 5 ) return 128+CC2_TRANS6_FREQ;
      case  UNDEF_29:
        if( LXRvoiceNr == 0 ) return 128+CC2_TRANS1_VOL;
        if( LXRvoiceNr == 1 ) return 128+CC2_TRANS2_VOL;
        if( LXRvoiceNr == 2 ) return 128+CC2_TRANS3_VOL;
        if( LXRvoiceNr == 3 ) return 128+CC2_TRANS4_VOL;
        if( LXRvoiceNr == 4 ) return 128+CC2_TRANS5_VOL;
        if( LXRvoiceNr == 5 ) return 128+CC2_TRANS6_VOL;
      
      case  SOUND_VAR: // 70
        if( LXRvoiceNr == 3 ) return REPEAT1; //voice4=snr
        if( LXRvoiceNr == 4 ) return CYM_REPEAT; //+clap
      case  ENV_ATTACK: // 73
        if( LXRvoiceNr == 0 ) return VELOA1;
        if( LXRvoiceNr == 1 ) return VELOA2;
        if( LXRvoiceNr == 2 ) return VELOA3;
        if( LXRvoiceNr == 3 ) return VELOA4; //snr
        if( LXRvoiceNr == 4 ) return VELOA5; //cym+clap
        if( LXRvoiceNr == 5 ) return VELOA6; //hh
      case  ENV_DECAY: // 75
        if( LXRvoiceNr == 0 ) return VELOD1;
        if( LXRvoiceNr == 1 ) return VELOD2;
        if( LXRvoiceNr == 2 ) return VELOD3;
        if( LXRvoiceNr == 3 ) return VELOD4; //snr
        if( LXRvoiceNr == 4 ) return VELOD5; //cym+clap
        if( LXRvoiceNr == 5 ) return VELOD6; //hh
      case  UNDEF_87: // was (bad idea!) sound_bright cc74
        if( LXRvoiceNr == 0 ) return VOL_SLOPE1;
        if( LXRvoiceNr == 1 ) return VOL_SLOPE2;
        if( LXRvoiceNr == 2 ) return VOL_SLOPE3;
        if( LXRvoiceNr == 3 ) return EG_SNARE1_SLOPE;
        if( LXRvoiceNr == 4 ) return CYM_SLOPE; //+clap
        if( LXRvoiceNr == 5 ) return VOL_SLOPE6; //hh

      case  SOUND_VIB_RATE: // 76
        if( LXRvoiceNr == 0 ) return FREQ_LFO1;
        if( LXRvoiceNr == 1 ) return FREQ_LFO2;
        if( LXRvoiceNr == 2 ) return FREQ_LFO3;
        if( LXRvoiceNr == 3 ) return FREQ_LFO4;
        if( LXRvoiceNr == 4 ) return FREQ_LFO5;
        if( LXRvoiceNr == 5 ) return FREQ_LFO6;
      case  SOUND_VIB_DEPTH: // 77
        if( LXRvoiceNr == 0 ) return AMOUNT_LFO1;
        if( LXRvoiceNr == 1 ) return AMOUNT_LFO2;
        if( LXRvoiceNr == 2 ) return AMOUNT_LFO3;
        if( LXRvoiceNr == 3 ) return AMOUNT_LFO4;
        if( LXRvoiceNr == 4 ) return AMOUNT_LFO5;
        if( LXRvoiceNr == 5 ) return AMOUNT_LFO6;
      case  SOUND_VIB_DELAY: // 78
        if( LXRvoiceNr == 0 ) return 128+CC2_OFFSET_LFO1;
        if( LXRvoiceNr == 1 ) return 128+CC2_OFFSET_LFO2;
        if( LXRvoiceNr == 2 ) return 128+CC2_OFFSET_LFO3;
        if( LXRvoiceNr == 3 ) return 128+CC2_OFFSET_LFO4;
        if( LXRvoiceNr == 4 ) return 128+CC2_OFFSET_LFO5;
        if( LXRvoiceNr == 5 ) return 128+CC2_OFFSET_LFO6;
      case  SOUND_UNDEF: // 79
        if( LXRvoiceNr == 0 ) return 128+CC2_WAVE_LFO1;
        if( LXRvoiceNr == 1 ) return 128+CC2_WAVE_LFO2;
        if( LXRvoiceNr == 2 ) return 128+CC2_WAVE_LFO3;
        if( LXRvoiceNr == 3 ) return 128+CC2_WAVE_LFO4;
        if( LXRvoiceNr == 4 ) return 128+CC2_WAVE_LFO5;
        if( LXRvoiceNr == 5 ) return 128+CC2_WAVE_LFO6;
    
      case  GEN_CONTROLLER_82:
        if( LXRvoiceNr == 0 ) return 128+CC2_RETRIGGER_LFO1;
        if( LXRvoiceNr == 1 ) return 128+CC2_RETRIGGER_LFO2;
        if( LXRvoiceNr == 2 ) return 128+CC2_RETRIGGER_LFO3;
        if( LXRvoiceNr == 3 ) return 128+CC2_RETRIGGER_LFO4;
        if( LXRvoiceNr == 4 ) return 128+CC2_RETRIGGER_LFO5;
        if( LXRvoiceNr == 5 ) return 128+CC2_RETRIGGER_LFO6;
      case  GEN_CONTROLLER_83:
        if( LXRvoiceNr == 0 ) return 128+CC2_SYNC_LFO1;
        if( LXRvoiceNr == 1 ) return 128+CC2_SYNC_LFO2;
        if( LXRvoiceNr == 2 ) return 128+CC2_SYNC_LFO3;
        if( LXRvoiceNr == 3 ) return 128+CC2_SYNC_LFO4;
        if( LXRvoiceNr == 4 ) return 128+CC2_SYNC_LFO5;
        if( LXRvoiceNr == 5 ) return 128+CC2_SYNC_LFO6;
      case  PORT_CONTROL: // 84
        if( LXRvoiceNr == 0 ) return PITCHD1; // mod env decay
        if( LXRvoiceNr == 1 ) return PITCHD2;
        if( LXRvoiceNr == 2 ) return PITCHD3;
        if( LXRvoiceNr == 3 ) return PITCHD4;
      case  UNDEF_85:
        if( LXRvoiceNr == 0 ) return MODAMNT1; // mod env decay
        if( LXRvoiceNr == 1 ) return MODAMNT2;
        if( LXRvoiceNr == 2 ) return MODAMNT3;
        if( LXRvoiceNr == 3 ) return MODAMNT4;
      case  UNDEF_86:
        if( LXRvoiceNr == 0 ) return PITCH_SLOPE1; // mod env decay
        if( LXRvoiceNr == 1 ) return PITCH_SLOPE2;
        if( LXRvoiceNr == 2 ) return PITCH_SLOPE3;
        if( LXRvoiceNr == 3 ) return PITCH_SLOPE4;
      
      case  UNDEF_89:
        if( LXRvoiceNr == 0 ) return 128+CC2_AUDIO_OUT1;
        if( LXRvoiceNr == 1 ) return 128+CC2_AUDIO_OUT2;
        if( LXRvoiceNr == 2 ) return 128+CC2_AUDIO_OUT3;
        if( LXRvoiceNr == 3 ) return 128+CC2_AUDIO_OUT4;
        if( LXRvoiceNr == 4 ) return 128+CC2_AUDIO_OUT5;
        if( LXRvoiceNr == 5 ) return 128+CC2_AUDIO_OUT6;
      
      case  UNDEF_90:
        if( LXRvoiceNr == 0 ) return 128+CC2_ENVELOPE_POSITION_1;
        if( LXRvoiceNr == 1 ) return 128+CC2_ENVELOPE_POSITION_2;
        if( LXRvoiceNr == 2 ) return 128+CC2_ENVELOPE_POSITION_3;
        if( LXRvoiceNr == 3 ) return 128+CC2_ENVELOPE_POSITION_4;
        if( LXRvoiceNr == 4 ) return 128+CC2_ENVELOPE_POSITION_5;
        if( LXRvoiceNr == 5 ) return 128+CC2_ENVELOPE_POSITION_6;
      
      case  UNDEF_102:
        if( LXRvoiceNr == 0 ) return MOD_WAVE_DRUM1;
        if( LXRvoiceNr == 1 ) return MOD_WAVE_DRUM2;
        if( LXRvoiceNr == 2 ) return MOD_WAVE_DRUM3;
      case  UNDEF_103:
        if( LXRvoiceNr == 0 ) return FMDTN1;
        if( LXRvoiceNr == 1 ) return FMDTN2;
        if( LXRvoiceNr == 2 ) return FMDTN3;
      case UNDEF_104:
        if( LXRvoiceNr == 0 ) return FMAMNT1;
        if( LXRvoiceNr == 1 ) return FMAMNT2;
        if( LXRvoiceNr == 2 ) return FMAMNT3;
      case  UNDEF_105:
        if( LXRvoiceNr == 4 ) return CYM_WAVE2; // +clap
        if( LXRvoiceNr == 5 ) return WAVE2_HH;
      case  UNDEF_106:
        if( LXRvoiceNr == 4 ) return CYM_MOD_OSC_GAIN1; //+clap
        if( LXRvoiceNr == 5 ) return MOD_OSC_GAIN1; //hh
      case  UNDEF_107:
        if( LXRvoiceNr == 4 ) return CYM_MOD_OSC_F1; //+clap
      case  UNDEF_108:
        if( LXRvoiceNr == 4 ) return CYM_WAVE3; //+clap
        if( LXRvoiceNr == 5 ) return WAVE3_HH;
      case  UNDEF_109:
        if( LXRvoiceNr == 4 ) return CYM_MOD_OSC_GAIN2; //+clap
        if( LXRvoiceNr == 5 ) return MOD_OSC_GAIN2; //hh
      case  UNDEF_110:
        if( LXRvoiceNr == 4 ) return CYM_MOD_OSC_F2; //+clap
        if( LXRvoiceNr == 5 ) return MOD_OSC_F2; //hh
  }

  return I_DUNNO;
}

void midiParser_MIDImodTargetHandler(MidiMsg msg)
{
  if((msg.status&0xb0)!=MIDI_CC) return;

  uint8_t MIDIchannl = (msg.status&0x0f)+1;
  uint8_t LXRvoiceNr = 15; // default no match
  if(MIDIchannl==midi_MidiChannels[0]) LXRvoiceNr=0; // DRUM1 voice 0
  if(MIDIchannl==midi_MidiChannels[1]) LXRvoiceNr=1; // DRUM2 voice 1
  if(MIDIchannl==midi_MidiChannels[2]) LXRvoiceNr=2; // DRUM3 voice 2
  if(MIDIchannl==midi_MidiChannels[3]) LXRvoiceNr=3; // SNARE voice 3
  if(MIDIchannl==midi_MidiChannels[4]) LXRvoiceNr=4; // CYMBAL voice 4
  if(MIDIchannl==midi_MidiChannels[5]) LXRvoiceNr=5; // CLOSED HAT voice 5
  if(MIDIchannl==midi_MidiChannels[6]) LXRvoiceNr=6; // OPEN HAT voice 6
  if(MIDIchannl==midi_MidiChannels[7]) LXRvoiceNr=7; // GLOBAL voice 7
  if(LXRvoiceNr==15) return; // abort if no match

  // velocity node targets
  uint8_t value = msg.data2&0x7f;
  switch( msg.data1  )
  {
    case  UNDEF_25: // data2 param<127
      modNode_setDestination(&velocityModulators[LXRvoiceNr], value);
      msg.data1 = (uint8_t)( (LXRvoiceNr&0x3f)<<1 );
      msg.data2 = value&0x7f;
      msg.status = FRONT_CC_VELO_TARGET;
      break;
    case  UNDEF_26: // data2 param>=128
      modNode_setDestination(&velocityModulators[LXRvoiceNr], value+128);
      msg.data1 = (uint8_t)( ((LXRvoiceNr&0x3f)<<1) + 1 );
      msg.data2 = value&0x7f;
      msg.status = FRONT_CC_VELO_TARGET;
      break;
  }
  
  // inform frontpanel of param changes
  if(msg.status==FRONT_CC_VELO_TARGET)
  {
    uart_sendFrontpanelByte(FRONT_CC_VELO_TARGET);
    uart_sendFrontpanelByte(msg.data1);
    uart_sendFrontpanelByte(msg.data2);
  }
}

/* handle per-channel per-voice msg to frontpanel MIDI_CC or MIDI_CC_2 msg
   input "msg" is e.g. from midiParser rx from ext port
*/
void midiParser_MIDIccHandler(MidiMsg msg)
{
  if((msg.status&0xb0)!=MIDI_CC) return;

  uint8_t MIDIchannl = (msg.status&0x0f)+1;
  uint8_t LXRvoiceNr = 15; // default no match
  if(MIDIchannl==midi_MidiChannels[0]) LXRvoiceNr=0; // DRUM1 voice 0
  if(MIDIchannl==midi_MidiChannels[1]) LXRvoiceNr=1; // DRUM2 voice 1
  if(MIDIchannl==midi_MidiChannels[2]) LXRvoiceNr=2; // DRUM3 voice 2
  if(MIDIchannl==midi_MidiChannels[3]) LXRvoiceNr=3; // SNARE voice 3
  if(MIDIchannl==midi_MidiChannels[4]) LXRvoiceNr=4; // CYMBAL voice 4
  if(MIDIchannl==midi_MidiChannels[5]) LXRvoiceNr=5; // CLOSED HAT voice 5
  if(MIDIchannl==midi_MidiChannels[6]) LXRvoiceNr=6; // OPEN HAT voice 6
  if(MIDIchannl==midi_MidiChannels[7]) LXRvoiceNr=7; // GLOBAL voice 7
  if(LXRvoiceNr==15) return; // abort if no match

  int16_t LXRparamNr = midiParser_MIDItoLXRparamNr(msg.data1, LXRvoiceNr);
  if(LXRparamNr==I_DUNNO) return; // abort if no match
  
  if(LXRparamNr<128)
  {
    msg.status = MIDI_CC;
    msg.data1  = LXRparamNr;
  }
  else // paramNr >= 128
  {
    msg.status = MIDI_CC_2;
    msg.data1  = LXRparamNr-128;
  }
  
  // inform frontpanel of param changes
  if(msg.status==MIDI_CC) //params<127
  {
      midiParser_LXRccHandler(msg,1); // update dsp
      uart_sendFrontpanelByte(PARAM_CC);  // rx 0xb0, tx 0xae
      uart_sendFrontpanelByte(msg.data1-1); // MIDI_CC is offset
      uart_sendFrontpanelByte(msg.data2);
  }
  if(msg.status==MIDI_CC_2) //paramNr>=128
  {
    midiParser_LXRccHandler(msg,1); // update dsp
    uart_sendFrontpanelByte(PARAM_CC2); // rx 0xb1, tx 0xaf
    uart_sendFrontpanelByte(msg.data1); // MIDI_CC_2 is not offset
    uart_sendFrontpanelByte(msg.data2);
  }
}

//-----------------------------------------------------------
/** Parse incoming midi messages and invoke corresponding action
 */
void midiParser_parseMidiMessage(MidiMsg msg)
{

// route message if needed
   if(midiParser_routing.value) {
      if(msg.bits.source==midiSourceUSB) {
         if(midiParser_routing.route.usb2midi){
         // route to midi out port
            uart_sendMidi(msg);
         }
      } 
      else if(msg.bits.source==midiSourceMIDI) {
         if(midiParser_routing.route.midi2midi) {
         // route to midi out port
            uart_sendMidi(msg);
         }
         if(midiParser_routing.route.midi2usb) {
         // route to usb out port
            usb_sendMidi(msg);
         }
      }
   }

   if(msg.bits.sysxbyte)
      return; // no further action needed. we don't interpret sysex data right now

// --AS FILT filter messages here. Filter inline below to be more optimal
// we are interested in the low nibble since we are Rx here


   if((msg.status & 0xF0) == 0XF0) {
   // BC: !!!NB!!! for midi jack input, system realtime messages 
   // are dealt with at the top of midiParser_parseUartData(),
   // to avoid conflicts with channel-specific messages. These may
   // not need repeating here, or may only exist for USB messages.
   
   // non-channel specific messages (system messages)
      switch(msg.status) {
         case MIDI_CLOCK:
            if((midiParser_txRxFilter & 0x02) && seq_getExtSync())
               seq_sync();
            break;
      
         case MIDI_START:
         case MIDI_CONTINUE:
            if((midiParser_txRxFilter & 0x02) && seq_getExtSync())
               sync_midiStartStop(1);
            break;
      
         case MIDI_STOP:
            if((midiParser_txRxFilter & 0x02) && seq_getExtSync())
               sync_midiStartStop(0);
            break;
      
         case MIDI_MTC_QFRAME:
         /* --AS Strategy:
         * If we get through all 8 of the mtc messages with a value of 0, it means that
         * the song has just started playing. That is the only time we will start the
         * sequencer. so we will not start it if they start the tape recorder playing half way
         * through the song. Also, if the sequencer is running, or we are in external sync mode
         * we will ignore mtc messages as well
         */
            if ((midiParser_txRxFilter & 0x02) == 0)
               break; // ignore filtered
         
         // keep track of when we got the last mtc. only do this for the 0 msg to save time
            if((msg.data1 & 0x70)==0) {
               midiParser_lastMtcReceived=systick_ticks;
            }
         
            if(seq_isRunning()) {
               break; //already running, so we don't care to figure out where we are
            }
            if(seq_getExtSync())
               break; // bypass the lot. we are using external midi clock sync
         
         // figure out whether we are at 0:0:0:0
            if((msg.data1 & 0x7F)==0) { // this is the first mtc message of the set AND it's value is 0
               midiParser_mtcIgnore=0; // reset our level of ignorance
            } 
            else if(midiParser_mtcIgnore) { // not the first msg and we are ignoring
               break; // get out of here fast
            } 
            else if((msg.data1 & 0x70)!=0x70) { // messages 0 to 6 and we are not ignoring yet
               if((msg.data1 & 0x0F) !=0) {
                  midiParser_mtcIgnore=1;
                  break; // get out fast
               }
            } 
            else { // message 7 and we are not ignoring yet
               if((msg.data1 & 0x01)==0) { // hour high nibble is 0
               // well, we got all the way thru all 8 messages with 0, so the song has just begun
               // tell the front that we've started running on our own
                  uart_sendFrontpanelByte(FRONT_SEQ_CC);
                  uart_sendFrontpanelByte(FRONT_SEQ_RUN_STOP);
                  uart_sendFrontpanelByte(1);
                  midiParser_mtcIgnore=1; // in case we happen to miss a 0 message. probably wouldn't happen, but...
                  midiParser_mtcIsRunning=1;
                  midiParser_lastMtcReceived=systick_ticks; // also might not be needed, but...
               // start the sequencer
                  seq_setRunning(1);
               }
            }
         
            break;
         default:
            break;
      }
   
   }
   else if((msg.status&0xB0) == 0xB0) // cc on any channel
   {
      // filter set to rx cc?
      //if(midiParser_txRxFilter & 0x04)
      {
        midiParser_MIDImodTargetHandler(msg);
        midiParser_MIDIccHandler(msg);
      }
     
      // deal with bank and morph separately, avoid mainboard overwhelming front
//      if( (msg.data1==BANK)||(msg.data1==MOD_WHEEL) ) //cc0 or cc1
//      {
//        uint8_t voiceBitFlags=0; // stripe voices across a byte
//        if (chanonly==midi_MidiChannels[7]) // global channel affects all voices
//           voiceBitFlags=0x3F;
//        else
//        {
//           if (chanonly==midi_MidiChannels[0])
//              voiceBitFlags|=BANK_1;
//           if (chanonly==midi_MidiChannels[1])
//              voiceBitFlags|=BANK_2;
//           if (chanonly==midi_MidiChannels[2])
//              voiceBitFlags|=BANK_3;
//           if (chanonly==midi_MidiChannels[3])
//              voiceBitFlags|=BANK_4;
//           if (chanonly==midi_MidiChannels[4])
//              voiceBitFlags|=BANK_5;
//           if ( (chanonly==midi_MidiChannels[5])||(chanonly==midi_MidiChannels[6]) )
//           {
//              voiceBitFlags|=BANK_6;
//              voiceBitFlags|=BANK_7;
//           }
//        }
//
//        if (voiceBitFlags!=0) // tx request to frontpanel?
//        {
//           if(msg.data1==BANK)      uart_sendFrontpanelByte(BANK_CHANGE_CC); // 0xad
//           if(msg.data1==MOD_WHEEL) uart_sendFrontpanelByte(MORPH_CC);       // 0xac
//           uart_sendFrontpanelByte(voiceBitFlags);
//           uart_sendFrontpanelByte(msg.data2);
//        }
//      }
//   }
//   else if((msg.status&0xf0)==ALL_SOUND_OFF) /*120*/
//   {
//      if(msg.data2 == 0) seq_setMute(msg.data1-TRACK1_SOUND_OFF,0);
//      else               seq_setMute(msg.data1-TRACK1_SOUND_OFF,1);
//   }
//   else if((msg.status&0xf0)==RESET_ALL_CONTROLLERS) /*121*/
//   {
//      seq_newVoiceAvailable=0x7f;
//      uart_sendFrontpanelSysExByte(PATCH_RESET);
   }
   else // note and progchange
   {
      const uint8_t msgonly =msg.status & 0xF0;
      const uint8_t chanonly=(msg.status&0x0F)+1;
   
      if((msgonly & 0xE0) == 0x80) {
      // note on or note off message (one of these two only)
         if(midiParser_txRxFilter & 0x01) {
            int8_t v;
         // --AS if a note message comes in on global channel, then send that note to
         // the voice that is currently active on the front.
            if(midi_MidiChannels[7]==chanonly) {
            
               // -bc- first, check to see if active track is set to 'any' - use chromatic mode if it is
               if( (msgonly==NOTE_ON/* && msg.data2*/) && !midi_NoteOverride[frontParser_activeTrack] ) {
                  midiParser_noteOn(frontParser_activeTrack, msg.data1, msg.data2, 1);
               } 
               // current active track is not set to 'any' - user wants to assign voices to global notes
               else if (msgonly==NOTE_ON){
                  for(v=0;v<7;v++){
                     if (midi_NoteOverride[v]==msg.data1){
                        midiParser_noteOn(v, msg.data1, msg.data2, 1);
                     }
                  }
               
               }
               else if( (msgonly==NOTE_OFF) && !midi_NoteOverride[frontParser_activeTrack] ) {
                  midiParser_noteOff(frontParser_activeTrack, msg.data1, msg.data2, 1);
               } 
               // current active track is not set to 'any' - user wants to assign voices to global notes
               else if (msgonly==NOTE_OFF){
                  for(v=0;v<7;v++){
                     if (midi_NoteOverride[v]==msg.data1){
                        midiParser_noteOff(v, msg.data1, msg.data2, 1);
                     }
                  }
               
               }
            }
           
            // additionally, check each voice channel to see if it cares about this message
            for(v=0;v<7;v++) {
               if(midi_MidiChannels[v]==chanonly) { // if channel match and we haven't sent it already for the voice
                  if(msgonly==NOTE_ON) {
                     if(v==frontParser_activeTrack)
                        midiParser_noteOn(v, msg.data1, msg.data2, 1);
                     else
                        midiParser_noteOn(v, msg.data1, msg.data2, 0);
                     //Also used in sequencer trigger note function
                  } 
                  else if (msgonly==NOTE_OFF)
                  { 
                     if(v==frontParser_activeTrack)
                        midiParser_noteOff(v, msg.data1, msg.data2, 1);
                     else
                        midiParser_noteOff(v, msg.data1, msg.data2, 0);
                  }
               } // if channel matches
            } // for each voice
         } // check midi filter
         
      } 
      else if(msgonly==PROG_CHANGE) 
      {
         
      // --AS respond to prog change and change patterns. This responds only when global channel matches the PC message's channel.
         //send the ack message to tell the front that a new pattern starts playing
         if(midiParser_txRxFilter & 0x08)
         {
            if(chanonly == midi_MidiChannels[7])
            {
               if(msg.data1<16)
               {
                  seq_setNextPattern(msg.data1&0x07,0x7f);
                  if(msg.data1>7)
                  {
                     seq_newVoiceAvailable=0x7f;
                  }
               }   
            }
            uint8_t i;
            for(i=0;i<NUM_TRACKS;i++) // set individual track patterns with PC on that channel
            {
               if(chanonly == midi_MidiChannels[i])
               {
                  seq_setNextPattern(msg.data1&0x07,i);
                  seq_newVoiceAvailable&=(0x01<<i);
               }
            }   
         }
      }
   }// channel specific msg
}

//-----------------------------------------------------------

void midiParser_handleStatusByte(unsigned char data)
{
// we received a channel voice/mode byte. set the status as appropriate
   switch(data&0xF0) {
   // 2 databyte messages
      case NOTE_OFF:
      case NOTE_ON:
      case MIDI_CC:
      case MIDI_PITCH_WHEEL:
      case MIDI_AT:
         midiMsg_tmp.status = data;	// store the new status byte
         parserState = MIDI_DATA1;	//status received, next should be data byte 1
         midiMsg_tmp.bits.length=2;// status is followed by 2 data bytes
         break;
   
   // 1 databyte messages
      case PROG_CHANGE:
      case CHANNEL_PRESSURE:
         midiMsg_tmp.status = data;	// store the new status byte
         parserState = MIDI_DATA1;	//status received, next should be data byte 1
         midiMsg_tmp.bits.length=1;// status is followed by 1 data bytes
         break;
   
   // messages we don't care about right now, and don't know how to handle or passthru (Are there any?).
      default:
         parserState = IGNORE;	// throw away any data bytes until next message
         midiMsg_tmp.bits.length=0;
      
         break;
   }
}
//-----------------------------------------------------------
// This will build up the midi message and hand it off to
// parseMidiMessage when it's complete
void midiParser_parseUartData(unsigned char data)
{

   if(data&0x80) { // High bit is set -  its either a status or a system message.
   // regardless of current state, we blindly start a new message without questioning it
      if((data&0xf8)==0xf8) // data is system realtime - deal with here to avoid data conflicts
      {
         switch(data)
         {      
            case MIDI_START:
            case MIDI_CONTINUE:
               if((midiParser_txRxFilter & 0x02) && seq_getExtSync())
                  sync_midiStartStop(1);
               break;
               
            case MIDI_STOP:
               if((midiParser_txRxFilter & 0x02) && seq_getExtSync())
                  sync_midiStartStop(0);
               break;
               
            case MIDI_CLOCK:
            // passthru clock and other realtime messages. start/stop are transmitted
            // by the sequencer, we don't need to duplicate them.
               if((midiParser_txRxFilter & 0x02) && seq_getExtSync())
                  seq_sync();
                  
            default:
            
               if(midiParser_routing.value) {
                  MidiMsg rtMsg;
                  rtMsg.status=data;
                  rtMsg.data1=0x00;
                  rtMsg.data2=0x00;
                  if(midiParser_routing.route.midi2midi) {
                  // route to midi out port
                     uart_sendMidi(rtMsg);
                  }
                  if(midiParser_routing.route.midi2usb) {
                  // route to usb out port
                     usb_sendMidi(rtMsg);
                  }
               
               }
               break;
         }
         // route message if needed
         return; // don't do anything else - leave the parser as it was. there is no followup data.
      }
      midiMsg_tmp.bits.sysxbyte=0;
      if( (data&0xf0) == 0xf0) { // system message
         midiMsg_tmp.status = data;
         switch(data) {
            case SYSEX_START: // get into sysex receive mode. any more bytes received until this status changes are considered
            		  // to be sysex data. we still need to parse this sysex start, in case we are routing it
               parserState = SYSEX_DATA;
               midiMsg_tmp.bits.length=0;
               goto parseMsg; // we will still parse it in case we are doing a passthru
            case SYSEX_END:	  // get out of sysex mode
               if(parserState==SYSEX_DATA) {
                  parserState=MIDI_STATUS;
                  midiMsg_tmp.bits.length=0;
                  goto parseMsg; // we will still parse it in case we are doing a passthru
               } 
               else {
               // spurious sysex end msg received. ignore it
               }
               break; //
         // 1 byte payload messages
            case MIDI_SONG_SEL:		// passthru only
            case MIDI_MTC_QFRAME: 	// mtc chunk
               parserState = MIDI_DATA1; 	// we expect the nugget of mtc frame info
               midiMsg_tmp.bits.length=1;// we expect 1 data byte
               break;
         
         // 0 byte payload messages (we will assume that any system message
         // other than those above has 0 byte payload)
            default:
               midiMsg_tmp.bits.length=0;
               goto parseMsg;
         }
      } 
      else { //status byte (channel specific message containing a channel number)
         midiParser_handleStatusByte(data);
      }
   } 
   else { // high bit is not set - it's a data byte
   
      switch(parserState)	{
         case MIDI_STATUS: // we are expecting status msg, but got data, so running status may be in effect
            if(midiMsg_tmp.bits.length) {
               midiMsg_tmp.data1 = data;
               if(midiMsg_tmp.bits.length==2)
                  parserState=MIDI_DATA2;
               else {
                  midiMsg_tmp.data2=0;
                  goto parseMsg;
               }
            } 
            else {
               break; // last msg had 0 payload, so wtf is this? just ignore it
            }
            break;
      
         case MIDI_DATA1:
            midiMsg_tmp.data1 = data;
            if(midiMsg_tmp.bits.length==2) {
            // we need another byte before we can do anything meaningful
               parserState = MIDI_DATA2;
            } 
            else { // it must be 1
               goto parseMsg;
            }
            break;
         case MIDI_DATA2:
            midiMsg_tmp.data2 = data;
            goto parseMsg; // message complete
         case SYSEX_DATA: // we are in sysex mode
            midiMsg_tmp.bits.sysxbyte=1;
            midiMsg_tmp.status = data; // status will contain the sysex byte
            midiMsg_tmp.bits.length=0;
            goto parseMsg;
         default: //we are expecting no data byte, but we got one.
         // ignore it
            break;
      } // switch parserState
   } // if high bit is set

   return;

parseMsg:
// we get here if we have proudly received a message that we want to do something with
   if(parserState != SYSEX_DATA) // we are still in sysex receive mode
      parserState = MIDI_STATUS; // next byte should be a new message, or we don't care about it
   midiMsg_tmp.bits.source=midiSourceMIDI;
   midiParser_parseMidiMessage(midiMsg_tmp);

}

// 0 - Off - nothing to nothing
// 1 - U2M - usb in to midi out
// 2 - M2M - midi in to midi out
// 3 - A2M - usb in and midi in to midi out
// 4 - M2U - midi in to usb out
// 5 - M2A - midi in to usb out and midi out

void midiParser_setRouting(uint8_t value)
{
   midiParser_routing.value=0;

   switch(value) {
      case 1:
         midiParser_routing.route.usb2midi=1;
         break;
      case 2:
         midiParser_routing.route.midi2midi=1;
         break;
      case 3:
         midiParser_routing.route.usb2midi=1;
         midiParser_routing.route.midi2midi=1;
         break;
      case 4:
         midiParser_routing.route.midi2usb=1;
         break;
      case 5:
         midiParser_routing.route.midi2midi=1;
         midiParser_routing.route.midi2usb=1;
         break;
      default:
      case 0:
      
         break;
   }

}

void midiParser_setFilter(uint8_t is_tx, uint8_t value)
{

   if(is_tx) // set the high nibble to value
      midiParser_txRxFilter = (value << 4) | (midiParser_txRxFilter & 0x0F);
   else // set the low nibble to value
      midiParser_txRxFilter = (value & 0x0F) | (midiParser_txRxFilter & 0xF0);

}
