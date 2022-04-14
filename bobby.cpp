/*
    BSD 3-Clause License

    Copyright (c) 2022, Jacob Ulmert
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * File: bobby.cpp
 *
 * Sample player
 *
 */

#include "bobby.hpp"
#include "notemap.h"
#include "samplebank.h"

static Bobby bobby;

void OSC_INIT(uint32_t platform, uint32_t api)
{
  (void)platform;
  (void)api;

  initSamples();
}

void OSC_CYCLE(const user_osc_param_t * const params, int32_t *yn, const uint32_t frames)
{
  Bobby::State &s = bobby.state;
  Bobby::Params &p = bobby.params;
  Bobby::Voice *voices = &bobby.voices[0];

  dsp::BiQuad &hpf = bobby.hpf;
  {
    if (s.triggerSample) {
      s.triggerSample = false;

      const float r = 1.0;
      uint16_t map = notemap[(params->pitch)>>8];
      uint16_t i = N_SAMPLES;
      while (i != 0) {
        i--;
        if ((map & (1 << i))) { 
          Sample *sample = &samples[i];

          Bobby::Voice &voice = voices[sample->voiceIdx];
          
          voice.sampleIdx = sample->offset;
          voice.sampleIdxEnd = sample->len + voice.sampleIdx;

          voice.sampleLen = sample->len;
          voice.sampleLoopIdx = sample->loopIdx;
          voice.sampleCnt = 0;
          voice.isPlaying = true;

          voice.gain = *voice.gainGroup;

          if (map & TRIG_SLOWER_PLAYBACK_SPEED) {
            voice.sampleStep = ((sample->rate) * (r - *voice.tuneGroup)) / (k_samplerate * 2);
          } else if (map & TRIG_FASTER_PLAYBACK_SPEED) {
            voice.sampleStep = ((sample->rate) * (r - *voice.tuneGroup)) / (k_samplerate * 0.5);
          } else {
            voice.sampleStep = ((sample->rate) * (r - *voice.tuneGroup)) / k_samplerate;
          }
              
          uint8_t repeats = 0;
          if (i > 0) {
            if (osc_white() > 0) {
              if (p.repeats > 0) {
                repeats += p.repeats;
                voice.sampleStep *= 0.5;
              }
            } else if (p.randomPitch) {
              voice.sampleStep *= 1.5;
            } 
          }
        
          voice.repeat = repeats;
          repeats++;
          voice.repeatGainDec = voice.gain / (float)repeats;
          voice.repeatDur = ((1.f / voice.sampleStep) * voice.sampleLen) / (repeats);
          if (voice.repeatDur < frames) {
            voice.repeatDur = frames;
          }

          if (voice.sampleLoopIdx == 0) {
            voice.decayDur = (1.f / voice.sampleStep) * voice.sampleLen;
            if (voice.repeat > 1 && voice.decayDur > voice.repeatDur) {
              voice.decayDur = voice.repeatDur;
            }          
          } else {
            voice.decayDur = (1.f / voice.sampleStep) * voice.sampleLen * 2.f;
          }
          voice.sampleIdxStart = voice.sampleIdx;
        }
      }
    }
  }
  
  q31_t * __restrict y = (q31_t *)yn;
  const q31_t * y_e = y + frames;

  uint8_t i = MAX_VOICES;
  while (i != 0) {
    i--;
    Bobby::Voice &voice = voices[i];
    if (voice.isPlaying) {
      if (voice.sampleCnt >= voice.decayDur) {
        voice.isPlaying = false;
      } else if (voice.sampleIdx < 0 || voice.sampleIdx >= voice.sampleIdxEnd) {
        if (voice.sampleLoopIdx != 0) {
          voice.sampleIdx = voice.sampleLoopIdx + (voice.sampleIdx - (float)voice.sampleIdxEnd);
        } else {
          voice.isPlaying = false;
        }
      }
    }
    if (voice.repeat != 0 && voice.sampleCnt >= voice.repeatDur) {
      voice.sampleIdx = voice.sampleIdxStart;
      voice.sampleCnt = voice.sampleCnt - voice.repeatDur;
      voice.gain -= voice.repeatGainDec;
      voice.isPlaying = true;   
      voice.repeat--;
    } 
  }

  Bobby::Voice &voice0 = voices[0];  
  Bobby::Voice &voice1 = voices[1];  
  Bobby::Voice &voice2 = voices[2];  
  Bobby::Voice &voice3 = voices[3];  
  Bobby::Voice &voice8bit = voices[4];

  const bool voice0_isPlaying = voice0.isPlaying;
  const bool voice1_isPlaying = voice1.isPlaying;
  const bool voice2_isPlaying = voice2.isPlaying;
  const bool voice3_isPlaying = voice3.isPlaying;
  const bool voice4_isPlaying = voice8bit.isPlaying;

  uint16_t sampleIdx;
  float fr;
  float sig;

  for (; y != y_e; ) {
// 0
    sig = 0.f;
    if (voice0_isPlaying) {
      sampleIdx = (uint16_t)voice0.sampleIdx;
      fr = (voice0.sampleIdx - sampleIdx);
      const float sample1 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
      sampleIdx++;
      const float sample2 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
      sig += ((((sample1 * (1.0 - fr) + sample2 * fr) - DENOMINATOR_12BIT))) * voice0.gain;
      voice0.sampleIdx += voice0.sampleStep;
      voice0.sampleCnt++;
    }
// 1  
    if (voice1_isPlaying) {
      sampleIdx = (uint16_t)voice1.sampleIdx;
      fr = (voice1.sampleIdx - sampleIdx);
      const float sample1 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
      sampleIdx++;
      const float sample2 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
      sig += (((sample1 * (1.0 - fr) + sample2 * fr) - DENOMINATOR_12BIT))
              * ((1.f - ((float)voice1.sampleCnt / (float)voice1.decayDur))) * voice1.gain;
      voice1.sampleIdx += voice1.sampleStep;
      voice1.sampleCnt++;
    }
// 3   
    if (voice3_isPlaying) {
      sampleIdx = (uint16_t)voice3.sampleIdx;
      fr = (voice3.sampleIdx - sampleIdx);
      const float sample1 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
      sampleIdx++;
      const float sample2 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
       sig += (((sample1 * (1.0 - fr) + sample2 * fr) - DENOMINATOR_12BIT))
              * ((1.f - ((float)voice3.sampleCnt / (float)voice3.decayDur))) * voice3.gain;
      voice3.sampleIdx += voice3.sampleStep;
      voice3.sampleCnt++;
    }
    sig = sig / DENOMINATOR_12BIT; 

// 2 
    if (voice2_isPlaying) {
      sampleIdx = (uint16_t)voice2.sampleIdx;
      fr = (voice2.sampleIdx - sampleIdx);
      const float sample1 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
      sampleIdx++;
      const float sample2 = ((uint8_t)sampleBank[sampleIdx] | ((((sampleBankUpper[(sampleIdx >> 1)] >> ((sampleIdx & 1) << 2)) & 0xf)) << 8));
      sig += hpf.process_fo(((((sample1 * (1.0 - fr) + sample2 * fr) - DENOMINATOR_12BIT)))
              * ((1.f - ((float)voice2.sampleCnt / (float)voice2.decayDur))) / DENOMINATOR_12BIT) * voice2.gain;
      voice2.sampleIdx += voice2.sampleStep;
      voice2.sampleCnt++;
    }
// 4   
    if (voice4_isPlaying) {
      sampleIdx = (uint16_t)voice8bit.sampleIdx;
      fr = (voice8bit.sampleIdx - sampleIdx);
      const float sample = ((((int8_t)sampleBank[sampleIdx] * (1.0 - fr)) + ((int8_t)sampleBank[sampleIdx + 1] * fr)) / DENOMINATOR_8BIT);
      sig += (sample * (1.f - ((float)voice8bit.sampleCnt / (float)voice8bit.decayDur))) * voice8bit.gain;
      voice8bit.sampleIdx += voice8bit.sampleStep;
      voice8bit.sampleCnt++;
    }
    
    *(y++) = f32_to_q31(sig);
  }
}

void OSC_NOTEON(const user_osc_param_t * const params) 
{
  bobby.state.triggerSample = true;
}

void OSC_NOTEOFF(const user_osc_param_t * const params)
{
}

void OSC_PARAM(uint16_t index, uint16_t value)
{ 
  Bobby::Params &p = bobby.params;

  switch (index) {
    case k_user_osc_param_id1: 
      p.tuneGroup_b = value / 100.f;
      if (p.tuneGroup_b < 0.1) {
        p.tuneGroup_b = 0.1;
      }
      break;
  
    case k_user_osc_param_id2:
      p.tuneGroup_a = value / 100.f;
      if (p.tuneGroup_a < 0.1) {
        p.tuneGroup_a = 0.1;
      }
      break;

    case k_user_osc_param_id3:
      p.repeats = (value != 0) ? 8:0;
      break;

    case k_user_osc_param_id4:
      p.randomPitch = (value != 0) ? 1:0;
      break;    
    
    case k_user_osc_param_shape:
      p.gainGroup_b = param_val_to_f32(value);
      break;

    case k_user_osc_param_shiftshape:
      p.gainGroup_a = param_val_to_f32(value);
      break;

    default:
      break;
  }
}