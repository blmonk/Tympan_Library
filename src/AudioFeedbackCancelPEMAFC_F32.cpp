
#include "AudioFeedbackCancelPEMAFC_F32.h"


// update() – called once per audio block by the Teensy audio framework.
void AudioFeedbackCancelPEMAFC_F32::update(void)
{
    audio_block_f32_t *in_block = AudioStream_F32::receiveReadOnly_f32();
    if (!in_block) return;

    audio_block_f32_t *out_block = AudioStream_F32::allocate_f32();
    if (!out_block) {
        AudioStream_F32::release(in_block);
        return;
    }

    if (newest_ring_audio_block_id != 999999) {
        if ((in_block->id > 100) && (newest_ring_audio_block_id > 0)) {
            if ((in_block->id != 0) &&
                ((in_block->id - newest_ring_audio_block_id) > 1))
            {
                Serial.print("AudioFeedbackCancelPEMAFC_F32: falling behind? "
                             "in_block = ");
                Serial.print(in_block->id);
                Serial.print(", ring block = ");
                Serial.println(newest_ring_audio_block_id);
            }
        }
    }

    if (enabled) {
        cha_afc(in_block->data, out_block->data, in_block->length);
    } else {
        for (int i = 0; i < in_block->length; i++)
            // pass-thru if disabled
            out_block->data[i] = in_block->data[i];
    }

    AudioStream_F32::transmit(out_block);
    AudioStream_F32::release(out_block);
    AudioStream_F32::release(in_block);
}


// levinsonDurbin()
//   Solves the AR(order) Yule-Walker equations given the autocorrelation
//   signal r and returns the prediction-error filter coefficients
//   a[1..order] in coeffs[0..order-1].
void AudioFeedbackCancelPEMAFC_F32::levinsonDurbin(const float32_t *r,
                                                    int order,
                                                    float32_t *coeffs)
{
    float32_t a[MAX_AFC_PEMAFC_AR_ORDER + 1];
    float32_t a_prev[MAX_AFC_PEMAFC_AR_ORDER + 1];
    memset(a, 0, sizeof(float32_t) * (order + 1));
    a[0] = 1.0f;

    float32_t alpha = r[0];
    if (alpha <= 0.0f) {
        memset(coeffs, 0, sizeof(float32_t) * order);
        return;
    }

    for (int i = 1; i <= order; i++) {
        float32_t acc = r[i];
        for (int j = 1; j < i; j++) acc += a[j] * r[i - j];
        float32_t k = -acc / alpha;

        // Stability guard: |k| >= 1 means the AR model is unstable
        // so the previous estimate is kept.
        if (!isfinite(k) || fabsf(k) >= 0.999f) return;

        memcpy(a_prev, a, sizeof(float32_t) * (i + 1));
        a[i] = k;
        for (int j = 1; j < i; j++) {
            a[j] = a_prev[j] + k * a_prev[i - j];
        }

        alpha *= (1.0f - k * k);
        if (alpha <= 1e-12f) { alpha = 1e-12f; break; }
    }

    for (int i = 0; i < order; i++) coeffs[i] = a[i + 1];
}


// estimateARCoeffs()
//   Linearizes ar_buf, demeans, autocorrelates, regularizes,
//   then runs Levinson-Durbin to update ar_coeffs.
void AudioFeedbackCancelPEMAFC_F32::estimateARCoeffs(void)
{
    // Linearize circular ar_buf (oldest first)
    int start = (ar_buf_idx - ar_block_len + MAX_AFC_PEMAFC_BLOCK_LEN)
                % MAX_AFC_PEMAFC_BLOCK_LEN;
    for (int i = 0; i < ar_block_len; i++) {
        ar_work_buf[i] = ar_buf[(start + i) % MAX_AFC_PEMAFC_BLOCK_LEN];
    }

    // De-mean
    float32_t mean_val = 0.0f;
    for (int i = 0; i < ar_block_len; i++) mean_val += ar_work_buf[i];
    mean_val /= (float32_t)ar_block_len;
    for (int i = 0; i < ar_block_len; i++) ar_work_buf[i] -= mean_val;

    // Error autocorrelation r[0..ar_order]
    for (int k = 0; k <= ar_order; k++) {
        float32_t sum = 0.0f;
        for (int n = k; n < ar_block_len; n++) {
            sum += ar_work_buf[n] * ar_work_buf[n - k];
        }
        ar_r[k] = sum;
    }

    // Regularization
    ar_r[0] += ar_reg * ar_r[0] + ar_reg;

    // call Levinson-Durbin with r, order, and 
    levinsonDurbin(ar_r, ar_order, ar_coeffs);
}


// receiveLoopBackAudio() – delivers loudspeaker signal which we call u.
//   Slides u ring buffer and stores the new block reversed (newest at idx 0).
void AudioFeedbackCancelPEMAFC_F32::receiveLoopBackAudio(float *u, int cs)
{
    // Sanity check
    for (int i = 0; i < cs; i++) {
        if (!std::isfinite(u[i])) {
            initializeStates();
            return;
        }
    }

    // Slide u_ring to make room for cs new samples.
    // Must preserve afl + delay_samples + p - 1 samples so that cha_afc()
    // can reach offset_u[afl-1 + p] for the deepest whitening tap.
    int p = effectiveAROrder();
    int preserve_len = afl + delay_samples + p - 1;
    int Idst = preserve_len + cs - 1;
    for (int Isrc = preserve_len - 1; Isrc >= 0; Isrc--) {
        u_ring[Idst] = u_ring[Isrc];
        Idst--;
    }

    // Insert reversed raw data to ring buffer
    for (int i = 0; i < cs; i++) {
        u_ring[cs - 1 - i] = u[i];
    }
}


// cha_afc() – process one block of input samples.
void AudioFeedbackCancelPEMAFC_F32::cha_afc(float32_t *y_buf,
                                             float32_t *e,
                                             int cs)
{
    int p = effectiveAROrder();

    for (int i = 0; i < cs; i++) {

        float32_t *offset_u = u_ring + (cs - 1) - i + delay_samples;

        float32_t y = y_buf[i];   // raw input sample

        // Raw feedback estimate
        float32_t fb_est;
        arm_dot_prod_f32(offset_u, efbp, afl, &fb_est);

        // NaN/Inf guard: if efbp has been corrupted, reset and pass through
        if (!std::isfinite(fb_est)) {
            initializeStates();
            for (int j = i; j < cs; j++) e[j] = y_buf[j];
            return;
        }

        e[i] = y - fb_est;

        // Whiten mic signal with PE filter
        float32_t y_f = y;
        for (int j = 0; j < p; j++) {
            int past_idx = i - 1 - j;
            float32_t past = (past_idx >= 0) ? y_buf[past_idx] : prev_y_buf[-(past_idx) - 1];
            y_f += ar_coeffs[j] * past;
        }

        // Whiten u into u_f with PE filter
        // u_f[k] = offset_u[k] + sum_j( ar_coeffs[j] * offset_u[k+j+1] )
        arm_copy_f32(offset_u, u_f, afl);
        for (int j = 0; j < p; j++) {
            arm_scale_f32(offset_u + j + 1, ar_coeffs[j], ar_work_buf, afl);
            arm_add_f32(u_f, ar_work_buf, u_f, afl);
        }

        // Whitened feedback estimate and whitened error
        float32_t fb_est_f;
        arm_dot_prod_f32(u_f, efbp, afl, &fb_est_f);
        float32_t e_f = y_f - fb_est_f;

        // NLMS update
        float32_t norm_sq;
        arm_dot_prod_f32(u_f, u_f, afl, &norm_sq);
        norm_sq += delta;

        float32_t mu_mult = (mu / norm_sq) * e_f;
        arm_scale_f32(u_f, mu_mult, u_f, afl);
        arm_add_f32(efbp, u_f, efbp, afl);
    }

    // Update prev_y_buf with the most recent p input samples.
    if (p > cs) {
        for (int j = p - 1; j >= cs; j--)
            prev_y_buf[j] = prev_y_buf[j - cs];
    }
    int fill = min(p, cs);
    for (int j = 0; j < fill; j++)
        prev_y_buf[j] = y_buf[cs - 1 - j];

    // Mode A: load AR estimation buffer with error signal and re-compute coeffs if 
    // we've reached a full block length.
    if (mode == PEMAFCMode::A) {
        for (int i = 0; i < cs; i++) {
            ar_buf[ar_buf_idx] = e[i];
            ar_buf_idx = (ar_buf_idx + 1) % MAX_AFC_PEMAFC_BLOCK_LEN;
        }
        ar_count += cs;
        if (ar_count >= ar_block_len) {
            ar_count = 0;
            estimateARCoeffs();
        }
    }
}