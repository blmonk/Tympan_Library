/*
*   AudioPassThru_FeedbackPEMAFC
*
*   Created: Bryan Monk, April 2026
*   Purpose: Demo for Audio pass-through with adaptive feedback cancellation using
*      the PEMAFC algorithm.
* 
*   Highpass filter is for DC blocking
*
*   Serial Monitor:
*      Send 'h' for the help menu. Most settings can be updated on the fly
*      including switching between fixed and adaptive modes.
*
*   Uses the Tympan EarpieceShield.  The front and rear
*      microphones of the right earpiece are averaged as the input.  Output goes to both earpiece
*      receivers and the Tympan headphone jack.
*
*   MIT License.  Use at your own risk.
*/

#include <Tympan_Library.h>

const float sample_rate_Hz      = 24000.0f;
const int   audio_block_samples = 16;
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

// Audio objects
Tympan                        myTympan(TympanRev::E, audio_settings);
EarpieceShield                earpieceShield(TympanRev::E, AICShieldRev::A);
AudioInputI2SQuad_F32         i2s_in(audio_settings);
AudioMixer4_F32               mic_mixer(audio_settings);      // average front + rear mics
AudioFilterBiquad_F32         hp_filter(audio_settings);      // DC-blocking highpass
AudioFeedbackCancelPEMAFC_F32 afc(audio_settings);            // generalized PEMAFC
AudioEffectGain_F32           gain1(audio_settings);          // digital gain
AudioOutputI2SQuad_F32        i2s_out(audio_settings);
AudioLoopBack_F32             afc_loopback(audio_settings);   // closes the AFC loop

// Audio connections
AudioConnection_F32  patchCord10(i2s_in,   EarpieceShield::PDM_LEFT_FRONT, mic_mixer, 0);
AudioConnection_F32  patchCord12(i2s_in,   EarpieceShield::PDM_LEFT_REAR,  mic_mixer, 1);
AudioConnection_F32  patchCord11(mic_mixer,      0, hp_filter,      0);
AudioConnection_F32  patchCord13(hp_filter,      0, afc,            0);
AudioConnection_F32  patchCord20(afc,            0, gain1,          0);
AudioConnection_F32  patchCord30(gain1, 0, i2s_out, EarpieceShield::OUTPUT_LEFT_EARPIECE);
AudioConnection_F32  patchCord31(gain1, 0, i2s_out, EarpieceShield::OUTPUT_RIGHT_EARPIECE);
AudioConnection_F32  patchCord32(gain1, 0, i2s_out, EarpieceShield::OUTPUT_LEFT_TYMPAN);
AudioConnection_F32  patchCord33(gain1, 0, i2s_out, EarpieceShield::OUTPUT_RIGHT_TYMPAN);
// Loopback (from post-gain signal)
AudioConnection_F32  patchCord40(gain1,          0, afc_loopback,   0);

// Serial
#include "SerialManager.h"
SerialManager serialManager;

// AFC setup
settings_AFC_PEMAFC afc_settings = AudioFeedbackCancelPEMAFC_F32::getDefaultSettings(
    sample_rate_Hz, audio_block_samples);

void setupAFC(void) {
    afc_settings.mode          = PEMAFCMode::A;
    afc_settings.mu            = 0.001f;
    afc_settings.delta         = 1e-3f;
    afc_settings.afl           = 256;
    afc_settings.alpha         = 0.9f;
    afc_settings.ar_order      = 12;
    afc_settings.ar_block_len  = 320;
    afc_settings.ar_reg        = 1e-5f;
    afc_settings.delay_samples = 0;

    afc.setParams(afc_settings);
    afc.setEnable(true);
}

// Globals
const float input_gain_dB            = 20.0f;
float       vol_knob_gain_dB         = 0.0f;
bool        enable_printCPUandMemory = false;
const float hp_cutoff_Hz             = 50.0f;
bool        enable_hp_filter         = true;

void setup() {
    myTympan.beginBothSerial(); delay(1000);
    myTympan.println("AudioPassThru_FeedbackPEMAFC: Starting setup()...");

    AudioMemory_F32(20, audio_settings);

    afc_loopback.setTarget(&afc);

    mic_mixer.gain(0, 0.5f);
    mic_mixer.gain(1, 0.5f);

    myTympan.enable();
    earpieceShield.enable();

    myTympan.enableDigitalMicInputs(true);
    earpieceShield.enableDigitalMicInputs(true);

    myTympan.volume_dB(0);
    earpieceShield.volume_dB(0);
    myTympan.setInputGain_dB(input_gain_dB);

    setHPFilterEnable(enable_hp_filter);
    setupAFC();

    servicePotentiometer(millis(), 0);

    myTympan.println("Setup complete.");
    serialManager.printHelp();
}

void loop() {
    while (Serial.available()) serialManager.respondToByte((char)Serial.read());

    servicePotentiometer(millis(), 100);

    if (enable_printCPUandMemory)
        myTympan.printCPUandMemory(millis(), 3000);

    myTympan.serviceLEDs(millis());
}

// Potentiometer servicing
void servicePotentiometer(unsigned long curTime_millis,
                          unsigned long updatePeriod_millis)
{
    static unsigned long lastUpdate_millis = 0;
    static float prev_val = -1.0f;

    if (curTime_millis < lastUpdate_millis) lastUpdate_millis = 0;
    if ((curTime_millis - lastUpdate_millis) <= updatePeriod_millis) return;

    float val = float(myTympan.readPotentiometer()) / 1023.0f;
    val = (1.0f / 9.0f) * (float)((int)(9.0f * val + 0.5f));

    if (abs(val - prev_val) > 0.05f) {
        prev_val = val;
        const float min_gain_dB = -20.0f, max_gain_dB = 40.0f;
        setVolKnobGain_dB(min_gain_dB + (max_gain_dB - min_gain_dB) * val);
    }
    lastUpdate_millis = curTime_millis;
}

// Gain helpers
float incrementKnobGain(float increment_dB) {
    return setVolKnobGain_dB(vol_knob_gain_dB + increment_dB);
}

float setVolKnobGain_dB(float gain_dB) {
    vol_knob_gain_dB = gain1.setGain_dB(gain_dB);
    printGainSettings();
    return vol_knob_gain_dB;
}

void printGainSettings(void) {
    Serial.print("Gains:");
    Serial.print("  Input (dB) = ");   Serial.print(input_gain_dB);
    Serial.print(", Digital (dB) = "); Serial.println(vol_knob_gain_dB);
}

// Highpass filter helper
void setHPFilterEnable(bool enable) {
    enable_hp_filter = enable;
    if (enable) {
        hp_filter.setHighpass(0, hp_cutoff_Hz, 0.707f);
    } else {
        float passthrough[] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        hp_filter.setCoefficients(0, passthrough);
    }
    Serial.println("HP filter (" + String(hp_cutoff_Hz, 0) + " Hz): " +
                   String(enable ? "ENABLED" : "DISABLED"));
}
