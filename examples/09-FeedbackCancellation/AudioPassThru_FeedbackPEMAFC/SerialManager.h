
#ifndef _SerialManager_h
#define _SerialManager_h

#include <Tympan_Library.h>
#include "AudioFeedbackCancelPEMAFC_F32.h"

// Externals provided by the main .ino file
extern Tympan                        myTympan;
extern AudioFeedbackCancelPEMAFC_F32 afc;
extern bool    enable_printCPUandMemory;
extern bool    enable_hp_filter;
extern float   vol_knob_gain_dB;
extern float   incrementKnobGain(float);
extern float   setVolKnobGain_dB(float);
extern void    printGainSettings(void);
extern void    setHPFilterEnable(bool);


class SerialManager : public SerialManagerBase {
  public:
    void printHelp(void);
    bool processCharacter(char c);

    // Override respondToByte to intercept newlines before the base class
    // discards them (SerialManagerBase::respondToByte strips \r and \n).
    virtual void respondToByte(char c) override {
        if (c == '\r' || c == '\n') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                lineLen = 0;
                processLine();
            }
            return;
        }
        SerialManagerBase::respondToByte(c);
    }

    float gainIncrement_dB  = 0.1f;
    float muFactor          = 2.0f;
    float alphaStep         = 0.05f;
    float deltaStep_factor  = sqrt(10.0f);

  private:
    char lineBuf[64];
    int  lineLen = 0;

    void processLine(void);
};


void SerialManager::printHelp(void) {
    Serial.println();
    Serial.println("SerialManager Help - AudioPassThru_FeedbackPEMAFC");
    Serial.println("  h / ?  : Print this help");
    Serial.println("  g      : Print gain settings");
    Serial.println("  c / C  : Enable / Disable CPU & memory printing");
    Serial.println("  k / K  : Increase / Decrease digital gain by "
                   + String(gainIncrement_dB, 1) + " dB");
    Serial.println("  gain <value>  : Set digital gain in dB directly"
                   " e.g. 'gain 10.5'");
    Serial.println("  f / F  : Enable / Disable the DC-blocking highpass filter"
                   " (currently " + String(enable_hp_filter) + ")");
    Serial.println("--- AFC ---");
    Serial.println("  a / A  : Enable / Disable AFC (currently "
                   + String(afc.getEnable()) + ")");
    Serial.println("  v      : Toggle AFC mode (currently mode "
                   + String(afc.isModeF() ? "F" : "A") + ")");
    Serial.println("  m / M  : Double / Halve AFC mu"
                   " (currently " + String(afc.getMu(), 6) + ")");
    Serial.println("  p / P  : Increase / Decrease alpha by "
                   + String(alphaStep, 2)
                   + " (mode F only; currently " + String(afc.getAlpha(), 4) + ")");
    Serial.println("  alpha <value> : Set alpha directly (mode F only)"
                   " e.g. 'alpha 0.9'");
    Serial.println("  e / E  : Increase / Decrease delta by x"
                   + String(deltaStep_factor, 2)
                   + " (currently " + String(afc.getEps(), 8) + ")");
    Serial.println("  x / X  : Increase / Decrease AFC filter length by 5"
                   " (currently " + String(afc.getAFL()) + ")");
    Serial.println("  afl <value>   : Set AFC filter length directly (max 1024)"
                   " e.g. 'afl 256'");
    Serial.println("  o / O  : Increase / Decrease AR order by 1"
                   " (mode A only; currently " + String(afc.getAROrder()) + ")");
    Serial.println("  b / B  : Double / Halve AR block length"
                   " (mode A only; currently " + String(afc.getARBlockLen()) + ")");
    Serial.println("  arlen <value> : Set AR block length directly (mode A only)"
                   " e.g. 'arlen 320'");
    Serial.println("  arreg <value> : Set AR regularization directly (mode A only)"
                   " e.g. 'arreg 1e-5'");
    Serial.println("  delay <value> : Set feedback path delay in samples"
                   " e.g. 'delay 32'");
    Serial.println("  i      : Print AFC parameter summary");
    Serial.println("  z      : Re-initialize AFC states (resets adaptive filter)");
    Serial.println("  w      : Print estimated feedback impulse response");
    Serial.println("  r      : Print current AR coefficients (mode A)");
    Serial.println();
}


bool SerialManager::processCharacter(char c) {
    if (c == 8 || c == 127) {
        if (lineLen > 0) lineLen--;
        return true;
    }
    if (lineLen < 63) lineBuf[lineLen++] = c;
    return true;
}

void SerialManager::processLine(void) {
    float  old_val, new_val;
    char   c = lineBuf[0];

    // Named commands (multi-character)
    if (strncmp(lineBuf, "afl ", 4) == 0) {
        int val = atoi(lineBuf + 4);
        Serial.print("Received: setting AFL to ");
        Serial.println(afc.setAFL(val));
        return;
    }
    if (strncmp(lineBuf, "gain ", 5) == 0) {
        float val = atof(lineBuf + 5);
        setVolKnobGain_dB(val);
        return;
    }
    if (strncmp(lineBuf, "alpha ", 6) == 0) {
        float val = atof(lineBuf + 6);
        Serial.print("Received: setting alpha to ");
        Serial.println(afc.setAlpha(val), 4);
        return;
    }
    if (strncmp(lineBuf, "arlen ", 6) == 0) {
        int val = atoi(lineBuf + 6);
        Serial.print("Received: setting AR block length to ");
        Serial.println(afc.setARBlockLen(val));
        return;
    }
    if (strncmp(lineBuf, "arreg ", 6) == 0) {
        float val = atof(lineBuf + 6);
        Serial.print("Received: setting AR reg to ");
        Serial.println(afc.setARReg(val), 8);
        return;
    }
    if (strncmp(lineBuf, "delay ", 6) == 0) {
        int val = atoi(lineBuf + 6);
        Serial.print("Received: setting delay_samples to ");
        Serial.println(afc.setDelaySamples(val));
        return;
    }

    // Single-character commands
    if (lineBuf[1] != '\0') {
        Serial.print("Unknown command: ");
        Serial.println(lineBuf);
        return;
    }

    switch (c) {

        // general
        case 'h': case '?':
            printHelp();
            break;

        case 'g': case 'G':
            printGainSettings();
            break;

        case 'c':
            Serial.println("Received: enabling CPU/memory printing.");
            enable_printCPUandMemory = true;
            break;
        case 'C':
            Serial.println("Received: disabling CPU/memory printing.");
            enable_printCPUandMemory = false;
            break;

        // digital gain
        case 'k':
            incrementKnobGain(gainIncrement_dB);
            break;
        case 'K':
            incrementKnobGain(-gainIncrement_dB);
            break;

        // highpass filter
        case 'f':
            setHPFilterEnable(true);
            break;
        case 'F':
            setHPFilterEnable(false);
            break;

        // AFC enable/disable
        case 'a':
            Serial.println("Received: enabling AFC.");
            afc.setEnable(true);
            break;
        case 'A':
            Serial.println("Received: disabling AFC.");
            afc.setEnable(false);
            break;

        // mode toggle
        case 'v': {
            PEMAFCMode newMode = afc.isModeF() ? PEMAFCMode::A : PEMAFCMode::F;
            afc.setMode(newMode);
            Serial.print("Received: AFC mode switched to ");
            Serial.println(afc.isModeF() ? "F (fixed pre-emphasis)" : "A (adaptive AR)");
            break;
        }

        // mu
        case 'm':
            Serial.print("Received: doubling mu to ");
            Serial.println(afc.setMu(afc.getMu() * muFactor), 6);
            break;
        case 'M':
            Serial.print("Received: halving mu to ");
            Serial.println(afc.setMu(afc.getMu() / muFactor), 6);
            break;

        // alpha
        case 'p':
            old_val = afc.getAlpha();  new_val = old_val + alphaStep;
            Serial.print("Received: increasing alpha to ");
            Serial.println(afc.setAlpha(new_val), 4);
            break;
        case 'P':
            old_val = afc.getAlpha();  new_val = old_val - alphaStep;
            Serial.print("Received: decreasing alpha to ");
            Serial.println(afc.setAlpha(new_val), 4);
            break;

        // delta
        case 'e':
            old_val = afc.getEps();  new_val = old_val * deltaStep_factor;
            Serial.print("Received: increasing delta to ");
            Serial.println(afc.setEps(new_val), 8);
            break;
        case 'E':
            old_val = afc.getEps();  new_val = old_val / deltaStep_factor;
            Serial.print("Received: decreasing delta to ");
            Serial.println(afc.setEps(new_val), 8);
            break;

        // filter length
        case 'x':
            old_val = (float)afc.getAFL();  new_val = old_val + 5.0f;
            Serial.print("Received: increasing AFL to ");
            Serial.println(afc.setAFL((int)new_val));
            break;
        case 'X':
            old_val = (float)afc.getAFL();  new_val = old_val - 5.0f;
            Serial.print("Received: decreasing AFL to ");
            Serial.println(afc.setAFL((int)new_val));
            break;

        // AR order
        case 'o':
            noInterrupts();
            Serial.print("Received: increasing AR order to ");
            Serial.println(afc.setAROrder(afc.getAROrder() + 1));
            interrupts();
            break;
        case 'O':
            noInterrupts();
            Serial.print("Received: decreasing AR order to ");
            Serial.println(afc.setAROrder(afc.getAROrder() - 1));
            interrupts();
            break;

        // AR block length
        case 'b':
            Serial.print("Received: doubling AR block length to ");
            Serial.println(afc.setARBlockLen(afc.getARBlockLen() * 2));
            break;
        case 'B':
            Serial.print("Received: halving AR block length to ");
            Serial.println(afc.setARBlockLen(afc.getARBlockLen() / 2));
            break;

        // diagnostics
        case 'i':
            afc.printAlgorithmInfo();
            break;

        case 'z':
            Serial.print("Received: re-initializing AFC states...");
            afc.setEnable(false);
            afc.initializeStates();
            afc.setEnable(true);
            Serial.println(" done.");
            break;

        case 'w':
            afc.printFeedbackImpulse();
            break;

        case 'r':
            afc.printARCoeffs();
            break;

        default:
            Serial.print("Unknown command: ");
            Serial.println(c);
            break;
    }
}

#endif  // _SerialManager_h
