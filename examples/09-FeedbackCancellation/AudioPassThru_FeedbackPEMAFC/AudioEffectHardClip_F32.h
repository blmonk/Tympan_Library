
#ifndef _AudioEffectHardClip_F32_h
#define _AudioEffectHardClip_F32_h

#include <Tympan_Library.h>

// Simple hard clipper.  Constrains every sample to [-threshold, +threshold].
// Uses in-place modification (receiveWritable) to avoid an extra block allocation.
class AudioEffectHardClip_F32 : public AudioStream_F32 {
  public:
    AudioEffectHardClip_F32(const AudioSettings_F32 &settings)
        : AudioStream_F32(1, inputQueueArray) {}

    void  setThreshold(float t)    { threshold = max(0.0f, t); }
    float getThreshold(void) const { return threshold; }

    virtual void update(void) {
        audio_block_f32_t *in_block = AudioStream_F32::receiveReadOnly_f32();
        if (!in_block) return;
        audio_block_f32_t *out_block = AudioStream_F32::allocate_f32();
        if (!out_block) {
            AudioStream_F32::release(in_block);
            return;
        }
        for (int i = 0; i < in_block->length; i++) {
            float32_t s = in_block->data[i];
            if      (s >  threshold) s =  threshold;
            else if (s < -threshold) s = -threshold;
            out_block->data[i] = s;
        }
        AudioStream_F32::transmit(out_block);
        AudioStream_F32::release(out_block);
        AudioStream_F32::release(in_block);
    }

  private:
    audio_block_f32_t *inputQueueArray[1];
    float threshold = 0.5f;
};

#endif  // _AudioEffectHardClip_F32_h
