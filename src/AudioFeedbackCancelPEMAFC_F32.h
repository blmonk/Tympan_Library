
/*
   AudioFeedbackCancelPEMAFC_F32

   Created: Bryan Monk, April 2026
   Purpose: PEMAFC adaptive feedback using NLMS update. Supports two modes:

       MODE F (PEMAFC-f):
         Whitening is done by a fixed first-order prediction error filter:
           A(z) = 1 - alpha * z^{-1}
         where alpha is fixed.

       MODE A (PEMAFC-a):
         The prediction error filter is formed by an AR model of order (ar_order).
         coeffs re-estimated every ar_block_len samples from the error signal with the 
         Levinson-Durbin algorithm.

   NOTE: setting a nonzero delay doesn't seem to work as-is

   MIT License.  Use at your own risk.
*/

#ifndef _AudioFeedbackCancelPEMAFC_F32_h
#define _AudioFeedbackCancelPEMAFC_F32_h

#include <Arduino.h>
#include <arm_math.h>
#include <string.h>
#include "AudioStream_F32.h"
#include "AudioLoopBack_F32.h"
#include <cfloat>
#include <cmath>

// Compile-time limits
#ifndef MAX_AFC_PEMAFC_FILT_LEN
#define MAX_AFC_PEMAFC_FILT_LEN   1024
#endif

#ifndef MAX_AFC_PEMAFC_AR_ORDER
#define MAX_AFC_PEMAFC_AR_ORDER   32
#endif

#ifndef MAX_AFC_PEMAFC_BLOCK_LEN
#define MAX_AFC_PEMAFC_BLOCK_LEN  512
#endif

#ifndef MAX_AFC_PEMAFC_AUDIO_BLOCK
#define MAX_AFC_PEMAFC_AUDIO_BLOCK 128
#endif


// Mode selector
enum class PEMAFCMode : uint8_t {
    F = 0,  // Fixed first-order AR model
    A = 1   // Adaptive AR model 
};


class settings_AFC_PEMAFC {
  public:
    PEMAFCMode mode;        // F or A
    float mu;               // NLMS step size
    float delta;              // NLMS regularization parameter
    int   afl;              // Adaptive filter length
    float alpha;            // Fixed 1st-order AR coeff (mode F only)
    int   ar_order;         // AR model order (mode A only)
    int   ar_block_len;     // Loudspeaker samples between AR updates (mode A)
    float ar_reg;           // Autocorrelation regularization (mode A)
    int   delay_samples;    // Extra delay samples beyond the inherent one-block lag (>= 0)
};


class AudioFeedbackCancelPEMAFC_F32
    : public AudioStream_F32, public AudioLoopBackInterface_F32
{
  public:

    AudioFeedbackCancelPEMAFC_F32(void)
        : AudioStream_F32(1, inputQueueArray_f32)
    {
        audio_block_samples = MAX_AFC_PEMAFC_AUDIO_BLOCK;
        setDefaultValues();
        initializeStates();
    }

    AudioFeedbackCancelPEMAFC_F32(const AudioSettings_F32 &settings)
        : AudioStream_F32(1, inputQueueArray_f32)
    {
        audio_block_samples = settings.audio_block_samples;
        setDefaultValues();
        initializeStates();
    }


    // Default/static settings helpers
    static settings_AFC_PEMAFC getDefaultSettings(float sample_rate_Hz,
                                                   int   block_samples)
    {
        settings_AFC_PEMAFC s;
        s.mode          = PEMAFCMode::F;
        s.mu            = 0.008f;
        s.delta           = 1e-6f;
        s.afl           = (int)(42.0f * (sample_rate_Hz / 24000.f));
        s.alpha         = 0.8f;
        s.ar_order      = 8;
        s.ar_block_len  = (int)(320.0f * (sample_rate_Hz / 24000.f));
        s.ar_reg        = 1e-6f;
        s.delay_samples = 0;
        return s;
    }

    virtual void setDefaultValues(void) {
        mode          = PEMAFCMode::F;
        mu            = 0.008f;
        delta           = 1e-6f;
        afl           = 100;
        alpha         = 0.8f;
        ar_order      = 8;
        ar_block_len  = 320;
        ar_reg        = 1e-6f;
        delay_samples = 0;
    }


    // Parameter setters / getters
    virtual settings_AFC_PEMAFC setParams(settings_AFC_PEMAFC &s) {
        setMu(s.mu);  setEps(s.delta);  setAFL(s.afl);
        setAlpha(s.alpha);
        setAROrder(s.ar_order);
        setARBlockLen(s.ar_block_len);
        setARReg(s.ar_reg);
        setDelaySamples(s.delay_samples);
        setMode(s.mode);   // call last so ar_coeffs are set correctly
        return getSettings();
    }

    virtual settings_AFC_PEMAFC getSettings(void) const {
        settings_AFC_PEMAFC s;
        s.mode = mode;  s.mu = mu;  s.delta = delta;  s.afl = afl;
        s.alpha = alpha;
        s.ar_order = ar_order;  s.ar_block_len = ar_block_len;
        s.ar_reg = ar_reg;
        s.delay_samples = delay_samples;
        return s;
    }

    /// Switch between mode F and mode A.
    /// In mode F: sets ar_coeffs[0] = -alpha, zeroes the rest, order=1.
    /// In mode A: resets ar_coeffs to zero (will be estimated).
    virtual void setMode(PEMAFCMode newMode) {
        mode = newMode;
        if (mode == PEMAFCMode::F) {
            ar_coeffs[0] = -alpha;
            for (int i = 1; i < MAX_AFC_PEMAFC_AR_ORDER; i++) ar_coeffs[i] = 0.0f;
        } else {
            for (int i = 0; i < MAX_AFC_PEMAFC_AR_ORDER; i++) ar_coeffs[i] = 0.0f;
            ar_count = 0;
        }
    }
    virtual PEMAFCMode getMode(void)  const { return mode; }
    virtual bool       isModeF(void)  const { return mode == PEMAFCMode::F; }
    virtual bool       isModeA(void)  const { return mode == PEMAFCMode::A; }

    virtual float setMu(float _mu)       { return mu  = max(0.0f, _mu); }
    virtual float getMu(void)  const     { return mu; }
    virtual float setEps(float _delta)     { return delta = max(1e-30f, _delta); }
    virtual float getEps(void) const     { return delta; }

    virtual int setAFL(int _afl) {
        afl = min(max(_afl, 1), MAX_AFC_PEMAFC_FILT_LEN);
        for (int i = afl; i < MAX_AFC_PEMAFC_FILT_LEN; i++) efbp[i] = 0.0f;
        return afl;
    }
    virtual int getAFL(void) const { return afl; }

    // AR coefficient in F mode (ar_coeffs[0]).
    virtual float setAlpha(float _alpha) {
        alpha = min(max(_alpha, 0.0f), 0.9999f);
        if (mode == PEMAFCMode::F) ar_coeffs[0] = -alpha;
        return alpha;
    }
    virtual float getAlpha(void) const { return alpha; }

    /// AR model order for mode A.
    virtual int setAROrder(int _order) {
        ar_order = min(max(_order, 1), MAX_AFC_PEMAFC_AR_ORDER);
        // Zero out coefficients beyond new order so stale values don't
        // contribute if order is later increased again.
        for (int i = ar_order; i < MAX_AFC_PEMAFC_AR_ORDER; i++) ar_coeffs[i] = 0.0f;
        return ar_order;
    }
    virtual int getAROrder(void) const { return ar_order; }

    /// Samples between AR updates (mode A).
    virtual int setARBlockLen(int _len) {
        ar_block_len = min(max(_len, 2 * audio_block_samples),
                           MAX_AFC_PEMAFC_BLOCK_LEN);
        return ar_block_len;
    }
    virtual int getARBlockLen(void) const { return ar_block_len; }

    /// Autocorrelation regularization (mode A).
    virtual float setARReg(float _reg)  { return ar_reg = max(0.0f, _reg); }
    virtual float getARReg(void) const  { return ar_reg; }

    /// Extra delay samples beyond the inherent one-block lag.
    /// Shifts the ring buffer offset so efbp[0] aligns with the start of
    /// the acoustic path rather than wasting taps on the delay.
    virtual int setDelaySamples(int _delay) {
        const int max_delay = MAX_RING
                              - MAX_AFC_PEMAFC_FILT_LEN
                              - MAX_AFC_PEMAFC_AR_ORDER
                              - MAX_AFC_PEMAFC_AUDIO_BLOCK
                              + 1;
        delay_samples = min(max(_delay, 0), max_delay);
        return delay_samples;
    }
    virtual int getDelaySamples(void) const { return delay_samples; }

    virtual bool enable(void)             { return enabled = true; }
    virtual bool enable(bool _enabled)    { return enabled = _enabled; }
    virtual void setEnable(bool _enabled) { enabled = _enabled; }
    virtual bool getEnable(void) const    { return enabled; }


    // State management
    virtual void initializeStates(void) {
        for (int i = 0; i < MAX_AFC_PEMAFC_FILT_LEN;  i++) efbp[i]         = 0.0f;
        for (int i = 0; i < MAX_RING;                  i++) u_ring[i]    = 0.0f;
        for (int i = 0; i < MAX_AFC_PEMAFC_BLOCK_LEN;  i++) ar_buf[i]      = 0.0f;
        for (int i = 0; i < MAX_AFC_PEMAFC_AR_ORDER;   i++) prev_y_buf[i] = 0.0f;

        ar_buf_idx   = 0;
        ar_count     = 0;
        // Restore ar_coeffs consistent with current mode
        setMode(mode);
    }


    // AudioStream_F32 interface
    virtual void update(void);


    // Core DSP routine
    virtual void cha_afc(float32_t *y_buf, float32_t *e, int cs);


    // AudioLoopBackInterface_F32
    virtual void receiveLoopBackAudio(audio_block_f32_t *in_block) {
        newest_ring_audio_block_id = in_block->id;
        receiveLoopBackAudio(in_block->data, in_block->length);
    }
    virtual void receiveLoopBackAudio(float *u, int cs);


    // Static helpers
    static void levinsonDurbin(const float32_t *r, int order, float32_t *coeffs);


    // Diagnostics
    virtual void printFeedbackImpulse(void) {
        Serial.println("AudioFeedbackCancelPEMAFC_F32: estimated feedback IR:");
        for (int i = 0; i < afl; i++) {
            Serial.print(efbp[i], 5);
            Serial.print(", ");
        }
        Serial.println();
    }

    virtual void printARCoeffs(void) {
        int p = effectiveAROrder();
        Serial.print("AudioFeedbackCancelPEMAFC_F32: AR coeffs (order=");
        Serial.print(p);
        Serial.print(", mode=");
        Serial.print(mode == PEMAFCMode::F ? "F" : "A");
        Serial.println("):");
        for (int i = 0; i < p; i++) {
            Serial.print("  a["); Serial.print(i);
            Serial.print("] = "); Serial.println(ar_coeffs[i], 6);
        }
    }

    virtual void printAlgorithmInfo(void) {
        Serial.println("AudioFeedbackCancelPEMAFC_F32: parameter values...");
        Serial.println("    mode          = " + String(mode == PEMAFCMode::F ? "F" : "A"));
        Serial.println("    mu            = " + String(mu,    6));
        Serial.println("    delta           = " + String(delta,   8));
        Serial.println("    afl           = " + String(afl));
        Serial.println("    alpha         = " + String(alpha, 6) + " (mode F)");
        Serial.println("    ar_order      = " + String(ar_order)     + " (mode A)");
        Serial.println("    ar_block_len  = " + String(ar_block_len) + " (mode A)");
        Serial.println("    ar_reg        = " + String(ar_reg, 8)    + " (mode A)");
        Serial.println("    eff. order    = " + String(effectiveAROrder()));
        Serial.println("    delay_samples = " + String(delay_samples));
    }


  protected:

    int effectiveAROrder(void) const {
        return (mode == PEMAFCMode::F) ? 1 : ar_order;
    }

    static const int MAX_RING = 2 * MAX_AFC_PEMAFC_FILT_LEN;

    // AudioStream bookkeeping
    audio_block_f32_t *inputQueueArray_f32[1];
    bool enabled = true;
    int  audio_block_samples;

    PEMAFCMode mode;

    // Shared parameters
    float mu, delta;
    int   afl;

    // Mode F parameter
    float alpha;

    // Mode A parameters
    int   ar_order;
    int   ar_block_len;
    float ar_reg;

    // Delay compensation
    int   delay_samples;

    float32_t ar_coeffs[MAX_AFC_PEMAFC_AR_ORDER];

    // Adaptive filter (estimated feedback impulse response)
    float32_t efbp[MAX_AFC_PEMAFC_FILT_LEN];

    // Ring buffer (raw speaker history, newest at [0])
    float32_t u_ring[MAX_RING];

    // AR estimation circular buffer (mode A)
    float32_t ar_buf[MAX_AFC_PEMAFC_BLOCK_LEN];
    int       ar_buf_idx;
    int       ar_count;

    // Working buffers for AR estimation and whitening scratch
    // Must be MAX_AFC_PEMAFC_FILT_LEN (not BLOCK_LEN) so it can also serve as
    // scratch in the CMSIS-optimized whitening loop inside cha_afc.
    float32_t ar_work_buf[MAX_AFC_PEMAFC_FILT_LEN];
    float32_t ar_r[MAX_AFC_PEMAFC_AR_ORDER + 1];

    // Inter-block history
    float32_t prev_y_buf[MAX_AFC_PEMAFC_AR_ORDER];  // newest at [0]

    // Whitened speaker regressor buffer
    float32_t u_f[MAX_AFC_PEMAFC_FILT_LEN];

    unsigned long newest_ring_audio_block_id = 999999;

    void estimateARCoeffs(void);

};  // class AudioFeedbackCancelPEMAFC_F32


#endif  // _AudioFeedbackCancelPEMAFC_F32_h