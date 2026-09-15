# Online Heart Rate Algorithm Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the offline MATLAB heart rate estimation algorithm (LMS adaptive filtering + FFT fusion) to run online in real-time on STM32L452CEU6.

**Architecture:** Foreground/Background pattern - Timer ISR collects sensor data into 1-second float buffers; main loop shifts 8-second sliding windows and runs the HR algorithm pipeline every 1 second. All memory statically allocated. CMSIS-DSP library used for IIR filtering, FFT, LMS, and vector operations. Full 3-path computation (LMS-HF, LMS-ACC, Pure-FFT) retained for debugging comparison; estimated CPU usage <1% of 1-second budget.

**Tech Stack:** C11, ARM CMSIS-DSP (cortexM4lf), STM32 HAL, arm-none-eabi-gcc, CMake

### Design Decisions

1. **保留全部 3 条 LMS 路径**: 预估总计算量 ~9ms/秒, 仅占 Cortex-M4F@80MHz 的 ~1%, 实时性无压力. 后期确认某路径无价值再裁剪.
2. **运动阈值采用混合方案**: 硬编码默认值 (立即可用), 同时后台自动标定覆盖. 不存在"必须等待"问题.
3. **蓝牙输出 1Hz 心率结果包**: 约 21 字节/秒, 包含融合心率/运动标志/各路径BPM/信号质量/ACC运动强度/时间戳.

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Create | `Core/Inc/hr_algorithm.h` | Algorithm public API, config struct, state struct, macros |
| Create | `Core/Src/hr_algorithm.c` | Main algorithm orchestration: window management, motion detect, fusion |
| Create | `Core/Inc/hr_dsp.h` | DSP primitive declarations (FFT peaks, correlation, IIR wrapper) |
| Create | `Core/Src/hr_dsp.c` | DSP primitives: FFT peak extraction, delay/correlation search, IIR filter state management |
| Create | `Core/Inc/hr_lms.h` | LMS adaptive filter declarations |
| Create | `Core/Src/hr_lms.c` | Cascaded normalized LMS filter using `arm_lms_norm_f32` |
| Modify | `CMakeLists.txt` | Enable CMSIS-DSP includes and link `libarm_cortexM4lf_math.a` |
| Modify | `Core/Src/main.c` | Add float buffer collection in ISR path, foreground/background loop, algorithm call |
| Modify | `Core/Inc/main.h` | Add algorithm-related includes |

### Memory Budget (160 KB RAM total)

| Buffer | Size |
|--------|------|
| 8-sec raw windows (5 ch x 1000 floats) | 20 KB |
| 8-sec filtered windows (5 ch x 1000 floats) | 20 KB |
| 1-sec collection buffers (5 ch x 125 floats) | 2.5 KB |
| 1-sec IIR filtered temp (5 ch x 125 floats) | 2.5 KB |
| FFT working buffer (4096 + 4096 output) | 32 KB |
| Hamming window + scratch (1000 + 1000 floats) | 8 KB |
| IIR state (5 ch x 2 biquads x 4 state) | 160 B |
| LMS state (5 instances x 20 taps + state) | ~4 KB |
| HR history + misc | ~2 KB |
| **Total algorithm** | **~89 KB** |
| Stack + HAL + drivers | ~20 KB |
| **Remaining free** | **~51 KB** |

---

## Task 1: Enable CMSIS-DSP in Build System

**Files:**
- Modify: `CMakeLists.txt:116-131`

- [ ] **Step 1: Update CMakeLists.txt CMSIS-DSP section**

Replace the commented-out CMSIS-DSP block (lines 116-131) with active configuration for Cortex-M4F:

```cmake
# --------------------------------- CMSIS-DSP -------------------------------- #
include_directories(
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include/
)
link_directories(
    Drivers/CMSIS/DSP/Lib/GCC
)
add_compile_definitions(
    ARM_MATH_CM4
    __FPU_PRESENT=1
    ARM_MATH_MATRIX_CHECK
)
list(APPEND libs arm_cortexM4lf_math)
```

- [ ] **Step 2: Verify compilation**

Run: `cd build && cmake .. && make` (or equivalent build command)
Expected: Compiles with CMSIS-DSP linked. May have unused warnings — acceptable.

- [ ] **Step 3: Commit**

```bash
git add CMakeLists.txt
git commit -m "feat: enable CMSIS-DSP library for Cortex-M4F"
```

---

## Task 2: Create Algorithm Header Files

**Files:**
- Create: `Core/Inc/hr_algorithm.h`
- Create: `Core/Inc/hr_dsp.h`
- Create: `Core/Inc/hr_lms.h`

- [ ] **Step 1: Create `hr_algorithm.h`**

```c
#ifndef HR_ALGORITHM_H
#define HR_ALGORITHM_H

#include "arm_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 编译期常量
 * ============================================================ */
#define HR_FS               125     /* 采样率 125 Hz (硬件固定) */
#define HR_WIN_SEC          8       /* 窗口长度 8 秒 */
#define HR_STEP_SEC         1       /* 滑动步长 1 秒 */
#define HR_WIN_SAMPLES      (HR_FS * HR_WIN_SEC)    /* 1000 点 */
#define HR_STEP_SAMPLES     (HR_FS * HR_STEP_SEC)   /* 125 点 */
#define HR_NUM_CHANNELS     5       /* PPG, ACCx, ACCy, ACCz, HF */
#define HR_FFT_LEN          4096    /* FFT 点数 (实数 RFFT) */
#define HR_MAX_PEAKS        20      /* 最大候选峰值数 */
#define HR_MAX_ORDER        16      /* LMS 滤波器最大阶数 */
#define HR_LMS_CASCADE_HF   2       /* HF 路径级联数 */
#define HR_LMS_CASCADE_ACC  3       /* ACC 路径级联数 */
#define HR_HR_HISTORY_LEN   120     /* 心率历史 (120 秒 @1Hz 输出) */
#define HR_SMOOTH_WIN       5       /* 中值平滑窗口 */

/* 频率范围 (Hz) */
#define HR_FREQ_LOW         1.0f    /* 有效心率下限 60 BPM */
#define HR_FREQ_HIGH        4.0f    /* 有效心率上限 240 BPM */
#define HR_BANDPASS_LOW     0.5f    /* 带通滤波下限 */
#define HR_BANDPASS_HIGH    5.0f    /* 带通滤波上限 */

/* ============================================================
 * 算法参数结构体 (运行时可调)
 * ============================================================ */
typedef struct {
    float   LMS_Mu_Base;            /* LMS 基础步长, 默认 0.01 */
    float   Spec_Penalty_Width;     /* 频谱惩罚宽度 (Hz), 默认 0.2 */
    float   Spec_Penalty_Weight;    /* 频谱惩罚权重, 默认 0.2 */
    float   HR_Range_Hz;            /* 运动段心率搜索范围 (Hz), 默认 0.25 */
    float   HR_Range_Rest_Hz;       /* 静息段心率搜索范围 (Hz) */
    float   Slew_Limit_BPM;         /* 运动段心率变化率限制 (BPM/s) */
    float   Slew_Step_BPM;          /* 运动段心率步进 (BPM) */
    float   Slew_Limit_Rest_BPM;    /* 静息段心率变化率限制 (BPM/s) */
    float   Slew_Step_Rest_BPM;     /* 静息段心率步进 (BPM) */
    float   Motion_Th_Scale;        /* 运动阈值倍数, 默认 3.0 */
    float   Default_Motion_Th;      /* 默认运动阈值 (立即可用, 默认 200.0) */
    uint8_t Spec_Penalty_Enable;    /* 是否启用频谱惩罚 */
} HR_Config_t;

/* ============================================================
 * 算法状态结构体 (全局唯一实例, 静态分配)
 * ============================================================ */
typedef struct {
    /* --- 1 秒采集缓冲区 (前台 ISR 写入) --- */
    float   buf_1s_ppg[HR_STEP_SAMPLES];
    float   buf_1s_accx[HR_STEP_SAMPLES];
    float   buf_1s_accy[HR_STEP_SAMPLES];
    float   buf_1s_accz[HR_STEP_SAMPLES];
    float   buf_1s_hf[HR_STEP_SAMPLES];
    volatile uint16_t sample_idx;   /* 当前 1 秒内采样索引 */
    volatile uint8_t  flag_1s_ready;/* 1 秒数据就绪标志 */

    /* --- 8 秒滑动窗口 - 原始数据 (后台算法使用) --- */
    float   win_ppg[HR_WIN_SAMPLES];
    float   win_accx[HR_WIN_SAMPLES];
    float   win_accy[HR_WIN_SAMPLES];
    float   win_accz[HR_WIN_SAMPLES];
    float   win_hf[HR_WIN_SAMPLES];
    uint8_t win_filled;             /* 窗口是否已填满至少一次 */

    /* --- 8 秒滑动窗口 - IIR 滤波后数据 (仅追加新滤波数据 + 平移) --- */
    float   filt_ppg[HR_WIN_SAMPLES];
    float   filt_accx[HR_WIN_SAMPLES];
    float   filt_accy[HR_WIN_SAMPLES];
    float   filt_accz[HR_WIN_SAMPLES];
    float   filt_hf[HR_WIN_SAMPLES];

    /* --- IIR 带通滤波器状态 (5 通道 x 2 biquad 节, 流式) --- */
    arm_biquad_casd_df1_inst_f32 biquad_ppg;
    arm_biquad_casd_df1_inst_f32 biquad_accx;
    arm_biquad_casd_df1_inst_f32 biquad_accy;
    arm_biquad_casd_df1_inst_f32 biquad_accz;
    arm_biquad_casd_df1_inst_f32 biquad_hf;
    float   iir_state_ppg[8];      /* 2 sections x 4 state */
    float   iir_state_accx[8];
    float   iir_state_accy[8];
    float   iir_state_accz[8];
    float   iir_state_hf[8];
    float   iir_filt_1s_ppg[HR_STEP_SAMPLES];   /* 1 秒滤波临时输出 */
    float   iir_filt_1s_accx[HR_STEP_SAMPLES];
    float   iir_filt_1s_accy[HR_STEP_SAMPLES];
    float   iir_filt_1s_accz[HR_STEP_SAMPLES];
    float   iir_filt_1s_hf[HR_STEP_SAMPLES];

    /* --- FFT 工作缓冲区 --- */
    float   fft_input[HR_FFT_LEN];
    float   fft_output[HR_FFT_LEN]; /* RFFT 输出 (complex interleaved) */

    /* --- LMS 状态 --- */
    /* HF 路径: 2 级级联 */
    arm_lms_norm_instance_f32 lms_hf[HR_LMS_CASCADE_HF];
    float   lms_hf_coeffs[HR_LMS_CASCADE_HF][HR_MAX_ORDER];
    float   lms_hf_state[HR_LMS_CASCADE_HF][HR_MAX_ORDER + HR_WIN_SAMPLES - 1];
    float   lms_hf_err[HR_WIN_SAMPLES];
    float   lms_hf_tmp[HR_WIN_SAMPLES];
    float   lms_hf_tmp2[HR_WIN_SAMPLES]; /* 级联第二临时缓冲区 */

    /* ACC 路径: 3 级级联 */
    arm_lms_norm_instance_f32 lms_acc[HR_LMS_CASCADE_ACC];
    float   lms_acc_coeffs[HR_LMS_CASCADE_ACC][HR_MAX_ORDER];
    float   lms_acc_state[HR_LMS_CASCADE_ACC][HR_MAX_ORDER + HR_WIN_SAMPLES - 1];
    float   lms_acc_err[HR_WIN_SAMPLES];
    float   lms_acc_tmp[HR_WIN_SAMPLES];
    float   lms_acc_tmp2[HR_WIN_SAMPLES]; /* 级联第二临时缓冲区 */

    /* --- 运动检测 --- */
    float   motion_threshold;       /* ACC 幅值标准差阈值 */
    uint8_t motion_calibrated;      /* 校准完成标志 */
    float   acc_baseline_std;       /* 静息段 ACC 标准差基线 */
    uint16_t calib_count;           /* 校准窗口计数器 (0~HR_CALIB_WINDOWS) */
    float   calib_std_accum;        /* 校准期 std 累积器 */
    uint16_t calib_windows_done;    /* 已完成的校准窗口数 */

    /* --- Hamming 窗 (预计算, const) --- */
    float   hamming_win[HR_WIN_SAMPLES];

    /* --- 通用 scratch 缓冲区 (zscore 归一化等) --- */
    float   scratch_a[HR_WIN_SAMPLES];
    float   scratch_b[HR_WIN_SAMPLES];

    /* --- 心率输出 --- */
    float   hr_lms_hf;              /* 当前 LMS-HF 路径心率 (Hz) */
    float   hr_lms_acc;             /* 当前 LMS-ACC 路径心率 (Hz) */
    float   hr_fft;                 /* 当前 FFT 路径心率 (Hz) */
    float   hr_fused;               /* 融合心率 (Hz) */
    float   hr_bpm;                 /* 融合心率 (BPM) */
    uint8_t is_motion;              /* 当前运动标志 */

    /* --- 心率历史追踪 --- */
    float   hr_history_lms_hf[HR_HR_HISTORY_LEN];
    float   hr_history_lms_acc[HR_HR_HISTORY_LEN];
    float   hr_history_fft[HR_HR_HISTORY_LEN];
    uint16_t hr_history_idx;        /* 当前历史索引 */

    /* --- 峰值检测临时缓冲 --- */
    float   peak_freqs[HR_MAX_PEAKS];
    float   peak_amps[HR_MAX_PEAKS];
    uint16_t num_peaks;
} HR_State_t;

/* ============================================================
 * 公共 API
 * ============================================================ */

/**
 * @brief 初始化算法状态 (必须在 main 初始化阶段调用)
 * @param config: 算法参数, 传 NULL 使用默认值
 * @param state:  算法状态结构体指针
 */
void HR_Init(const HR_Config_t *config, HR_State_t *state);

/**
 * @brief 向 1 秒缓冲区追加一个采样点 (由 ISR 或 ISR 触发的采集逻辑调用)
 * @param state:  算法状态
 * @param ppg:    PPG 采样值 (float)
 * @param accx:   ACC X 采样值
 * @param accy:   ACC Y 采样值
 * @param accz:   ACC Z 采样值
 * @param hf:     HF/ADC 采样值
 */
void HR_PushSample(HR_State_t *state,
                   float ppg, float accx, float accy, float accz, float hf);

/**
 * @brief 执行一次完整的 8 秒窗口心率解算 (由主循环调用)
 * @param state:  算法状态
 * @return 融合心率值 (BPM), 0 表示窗口未填满或错误
 */
float HR_RunSolver(HR_State_t *state);

/**
 * @brief 获取默认配置参数
 * @param config: 输出配置
 */
void HR_GetDefaultConfig(HR_Config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGORITHM_H */
```

- [ ] **Step 2: Create `hr_dsp.h`**

```c
#ifndef HR_DSP_H
#define HR_DSP_H

#include "arm_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 预计算的 4 阶 Butterworth 带通滤波器系数 (0.5-5 Hz @ 125 Hz) */
/* 2 个 biquad 节, 每节 5 个系数 [b0, b1, b2, -a1, -a2] */
/* 外部通过 MATLAB/Octave 预计算, 运行时不可修改 */
extern const float HR_BPF_COEFFS[10];  /* 2 sections x 5 coeffs */

/**
 * @brief 对信号进行 FFT 并提取频谱峰值
 * @param signal:    输入信号 (时域, 长度 sig_len)
 * @param sig_len:   信号长度
 * @param fft_input: FFT 工作缓冲区 (>= HR_FFT_LEN)
 * @param fft_output:FFT 输出缓冲区 (>= HR_FFT_LEN)
 * @param Fs:        采样率
 * @param percent:   峰值阈值比例 (如 0.3)
 * @param peak_freq: 输出峰值频率数组
 * @param peak_amp:  输出峰值幅值数组
 * @param max_peaks: 峰值数组最大容量
 * @param num_peaks: 输出实际峰值数
 */
void DSP_FFTPeaks(const float *signal, uint16_t sig_len,
                  float *fft_input, float *fft_output,
                  float Fs, float percent,
                  float *peak_freq, float *peak_amp,
                  uint16_t max_peaks, uint16_t *num_peaks);

/**
 * @brief 在 +/-5 个采样点范围内搜索最优时延 (基于去均值点积, 近似皮尔逊相关)
 *
 * 实现说明:
 *   MATLAB 原版使用 corr() (皮尔逊相关), 在线版用去均值点积近似.
 *   对于每个时延 d (-5..+5):
 *     1. 从 ref 中取 [d : d+len-1] 段 (等价于参考信号时间平移 d 个采样)
 *     2. 对 ppg 段和 ref 段分别去均值
 *     3. 计算点积 arm_dot_prod_f32
 *   返回绝对值最大的点积对应的时延 d.
 *
 * @param ppg:     PPG 信号 (8 秒窗口, 长度 len)
 * @param ref:     参考信号 (8 秒窗口 + 10 点余量, 长度 >= len + 10)
 *                 ref[5..5+len-1] 对应零时延对齐位置
 * @param len:     有效窗口长度 (1000 点)
 * @param scratch: 临时缓冲区 (>= len, 用于去均值)
 * @param max_corr:输出最大相关度 (绝对值)
 * @return 最优时延 (采样点数, -5 ~ +5)
 */
int16_t DSP_FindDelay(const float *ppg, const float *ref, uint16_t len,
                      float *scratch, float *max_corr);

/**
 * @brief 按幅值降序排列峰值频率
 * @param freqs: 峰值频率数组
 * @param amps:  峰值幅值数组
 * @param n:     峰值数量
 */
void DSP_SortPeaksByAmp(float *freqs, float *amps, uint16_t n);

/**
 * @brief 在候选峰中选择最接近历史心率的峰值
 * @param freqs:    已排序的候选频率数组 (按幅值降序)
 * @param n:        候选数量
 * @param hr_prev:  上一次心率 (Hz)
 * @param range_plus: 正向搜索范围 (Hz)
 * @param range_minus: 负向搜索范围 (Hz, 通常为负值)
 * @return 选中的心率 (Hz), 若无匹配则返回 hr_prev
 */
float DSP_TrackHR(const float *freqs, uint16_t n,
                  float hr_prev, float range_plus, float range_minus);

/**
 * @brief 频谱惩罚: 抑制运动频率及其二次谐波
 * @param peak_freq: 峰值频率数组 (会被修改)
 * @param peak_amp:  峰值幅值数组 (会被修改)
 * @param num_peaks: 峰值数量
 * @param motion_freq: 运动主频 (Hz)
 * @param penalty_width: 惩罚宽度 (Hz)
 * @param penalty_weight: 惩罚权重 (0-1, 越小抑制越强)
 */
void DSP_SpectrumPenalty(float *peak_freq, float *peak_amp,
                         uint16_t num_peaks,
                         float motion_freq, float penalty_width,
                         float penalty_weight);

/**
 * @brief 简易中值平滑 (滑动窗口)
 * @param hist:     心率历史数组
 * @param len:      数组长度
 * @param win_len:  平滑窗口长度 (奇数)
 * @param out:      输出平滑后的最后一个值
 */
void DSP_MedianSmooth(const float *hist, uint16_t len,
                      uint8_t win_len, float *out);

#ifdef __cplusplus
}
#endif

#endif /* HR_DSP_H */
```

- [ ] **Step 3: Create `hr_lms.h`**

```c
#ifndef HR_LMS_H
#define HR_LMS_H

#include "arm_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化一个归一化 LMS 滤波器实例
 * @param inst:    CMSIS-DSP LMS 实例
 * @param coeffs:  FIR 系数数组 (长度 = num_taps)
 * @param state:   状态缓冲区 (长度 = num_taps + block_size - 1)
 * @param num_taps:FIR 阶数
 * @param mu:      步长参数
 * @param block_size: 处理块大小 (用于计算状态缓冲区大小)
 */
void LMS_Init(arm_lms_norm_instance_f32 *inst,
              float *coeffs, float *state,
              uint16_t num_taps, float mu, uint32_t block_size);

/**
 * @brief 执行一次 LMS 滤波 (处理一个完整窗口)
 * @param inst:      LMS 实例
 * @param ref:       参考噪声信号 (如 ACC 或 HF)
 * @param desired:   期望信号 (PPG, 也作为输入)
 * @param err_out:   误差输出 (去噪后的信号)
 * @param tmp_out:   临时缓冲区 (滤波器输出, 不使用)
 * @param block_len: 数据块长度
 */
void LMS_Process(arm_lms_norm_instance_f32 *inst,
                 const float *ref, const float *desired,
                 float *err_out, float *tmp_out,
                 uint32_t block_len);

/**
 * @brief 级联 LMS 滤波: 多级串联处理
 * @param instances: LMS 实例数组
 * @param num_cascade: 级联级数
 * @param ref_channels: 参考信号通道数组 [级数][样本]
 * @param ppg:     输入 PPG 信号
 * @param err_out: 最终误差输出
 * @param tmp_out:  临时缓冲区 A
 * @param tmp_out2: 临时缓冲区 B (与 tmp_out 交替用于级联)
 * @param block_len: 数据块长度
 */
void LMS_CascadeProcess(arm_lms_norm_instance_f32 *instances,
                        uint8_t num_cascade,
                        float **ref_channels,
                        const float *ppg,
                        float *err_out,
                        float *tmp_out, float *tmp_out2,
                        uint32_t block_len);

#ifdef __cplusplus
}
#endif

#endif /* HR_LMS_H */
```

- [ ] **Step 4: Commit**

```bash
git add Core/Inc/hr_algorithm.h Core/Inc/hr_dsp.h Core/Inc/hr_lms.h
git commit -m "feat: add algorithm header files with API definitions"
```

---

## Task 3: Implement DSP Primitives (`hr_dsp.c`)

**Files:**
- Create: `Core/Src/hr_dsp.c`

- [ ] **Step 1: Implement `hr_dsp.c`**

Key implementations:
1. **`DSP_FFTPeaks`**: Uses `arm_rfft_fast_f32` for 4096-point real FFT, computes magnitude spectrum via `arm_cmplx_mag_f32`, finds peaks by comparing neighbors, filters to 1-4 Hz range, applies amplitude threshold.
2. **`DSP_FindDelay`**: For delay -5 to +5:
   - 取 `ref[5+d : 5+d+len-1]` 作为时延后的参考信号段 (ref 数组需要 len+10 个元素)
   - 对 ppg 段和该 ref 段分别用 `arm_mean_f32` + `arm_offset_f32` 去均值
   - 用 `arm_dot_prod_f32` 计算去均值后的点积
   - 返回绝对值最大的点积对应的时延 d
   - 注意: 在线模式下, ref 窗口首尾各保留 5 个额外样本以支持 ±5 偏移. 这些额外样本来自原始窗口向前后各扩展 5 个采样点, 需要在窗口平移时一起维护 (在 filt_* 缓冲区前后各多分配 5 个 float 即可)
3. **`DSP_SortPeaksByAmp`**: Simple insertion sort (arrays are small, <= 20 elements).
4. **`DSP_TrackHR`**: Scans top 5 sorted peaks, returns first within range of previous HR.
5. **`DSP_SpectrumPenalty`**: Multiplies amplitude of peaks near motion_freq or 2*motion_freq by penalty_weight.
6. **`DSP_MedianSmooth`**: Simple sliding median filter.
7. **BPF coefficients**: Hardcoded `const float HR_BPF_COEFFS[10]` computed from MATLAB `butter(4, [0.5 5]/62.5, 'bandpass')` then `tf2sos`.

Full implementation (~250 lines) should include:
- `arm_rfft_fast_instance_f32` initialized once via `arm_rfft_fast_init_f32`
- Proper single-sided spectrum computation (DC + Nyquist special cases, intermediate bins × 2)
- Frequency axis: `freq = bin_index * Fs / FFT_LEN`
- Peak detection: local maxima (sample[i] > sample[i-1] && sample[i] > sample[i+1])

- [ ] **Step 2: Verify compilation**

Run: build command
Expected: Compiles cleanly with CMSIS-DSP symbols resolved.

- [ ] **Step 3: Commit**

```bash
git add Core/Src/hr_dsp.c
git commit -m "feat: implement DSP primitives (FFT peaks, delay, spectrum penalty)"
```

---

## Task 4: Implement LMS Adaptive Filter (`hr_lms.c`)

**Files:**
- Create: `Core/Src/hr_lms.c`

- [ ] **Step 1: Implement `hr_lms.c`**

Key implementations:

**`LMS_Init`**:
```c
/**
 * @param block_size: 处理块大小 (通常为 HR_WIN_SAMPLES = 1000)
 */
void LMS_Init(arm_lms_norm_instance_f32 *inst,
              float *coeffs, float *state,
              uint16_t num_taps, float mu, uint32_t block_size)
{
    /* CMSIS-DSP 要求: 调用前 coeffs 和 state 必须已清零 */
    memset(coeffs, 0, num_taps * sizeof(float));
    memset(state, 0, (num_taps + block_size - 1) * sizeof(float));
    arm_lms_norm_init_f32(inst, num_taps, coeffs, state, mu);
}
```

**`LMS_Process`**:
```c
void LMS_Process(arm_lms_norm_instance_f32 *inst,
                 const float *ref, const float *desired,
                 float *err_out, float *tmp_out,
                 uint32_t block_len)
{
    /* arm_lms_norm_f32 要求 ref/desired 是非常量指针.
     * 实际使用中传入的都是可修改的缓冲区.
     * 注意: LMS 输入信号需要先做 zscore 归一化 (零均值, 单位方差).
     * 在调用 LMS_Process 前, 由调用者负责归一化. */
    arm_lms_norm_f32(inst,
                     (float *)ref,       /* pSrc: 参考噪声信号 */
                     (float *)desired,   /* pRef: 期望信号 */
                     tmp_out,            /* pOut: 滤波器输出 (不使用) */
                     err_out,            /* pErr: 误差 = desired - output */
                     block_len);
}
```

**`LMS_CascadeProcess`**:
```c
/* 签名变更: 新增 tmp_out2 参数用于级联交替缓冲 */
void LMS_CascadeProcess(arm_lms_norm_instance_f32 *instances,
                        uint8_t num_cascade,
                        float **ref_channels,
                        const float *ppg,
                        float *err_out,
                        float *tmp_out,   /* 临时缓冲区 A */
                        float *tmp_out2,  /* 临时缓冲区 B (级联交替用) */
                        uint32_t block_len)
{
    /* 第一级: ref_channels[0] vs ppg -> err_out */
    LMS_Process(&instances[0], ref_channels[0], ppg, err_out, tmp_out, block_len);

    /* 后续级: 上一级 err_out 作为下一级的 desired
     * 使用两个临时缓冲区交替, 避免读写冲突:
     *   偶数级: tmp_out  作为 desired 拷贝, 结果写 err_out
     *   奇数级: tmp_out2 作为 desired 拷贝, 结果写 err_out
     */
    for (uint8_t i = 1; i < num_cascade; i++) {
        float *desired_buf = (i % 2 == 1) ? tmp_out : tmp_out2;
        float *filter_out  = (i % 2 == 1) ? tmp_out2 : tmp_out;
        memcpy(desired_buf, err_out, block_len * sizeof(float));
        LMS_Process(&instances[i], ref_channels[i], desired_buf, err_out, filter_out, block_len);
    }
}
```

Note: HR_State_t 中已分配 `lms_hf_tmp` 和 `lms_hf_tmp2` (各 1000 floats) 用于 HF 路径级联, `lms_acc_tmp` 和 `lms_acc_tmp2` 用于 ACC 路径级联. IIR 滤波在 LMS 之前完成, 不会产生缓冲区冲突.

Critical detail: Before calling LMS, the input signals must be zscore-normalized (subtract mean, divide by std). This should be done in the algorithm orchestrator before calling cascade.

- [ ] **Step 2: Verify compilation**

- [ ] **Step 3: Commit**

```bash
git add Core/Src/hr_lms.c
git commit -m "feat: implement cascaded NLMS adaptive filter wrapper"
```

---

## Task 5: Implement Algorithm Orchestrator (`hr_algorithm.c`)

**Files:**
- Create: `Core/Src/hr_algorithm.c`

- [ ] **Step 1: Implement core functions**

This is the largest file (~400 lines). Key functions:

**`HR_GetDefaultConfig`**: Returns default parameters matching MATLAB optimized values.

**`HR_Init`**:
1. Clear entire state to zero
2. Initialize all 5 IIR biquad instances with `arm_biquad_cascade_df1_init_f32` (2 sections each, using `HR_BPF_COEFFS`)
3. Initialize LMS instances for HF path (2 cascade) and ACC path (3 cascade) with `LMS_Init`
4. Set default HR values (e.g., 1.2 Hz = 72 BPM)
5. Set `motion_calibrated = 0`, `calib_count = 0`

**`HR_PushSample`**:
```c
void HR_PushSample(HR_State_t *s, float ppg, float ax, float ay, float az, float hf)
{
    if (s->sample_idx >= HR_STEP_SAMPLES) return; /* 防溢出 */
    s->buf_1s_ppg[s->sample_idx]   = ppg;
    s->buf_1s_accx[s->sample_idx]  = ax;
    s->buf_1s_accy[s->sample_idx]  = ay;
    s->buf_1s_accz[s->sample_idx]  = az;
    s->buf_1s_hf[s->sample_idx]    = hf;
    s->sample_idx++;
    if (s->sample_idx >= HR_STEP_SAMPLES) {
        s->flag_1s_ready = 1;
    }
}
```

**`HR_RunSolver`** (the main pipeline):
```
1. 检查 flag_1s_ready, 若未就绪返回 0
2. 清除 flag_1s_ready

3. 原始数据窗口平移:
   - memmove 5 个通道的 win_* (后 7 秒前移 125)
   - memcpy 1 秒新数据 (buf_1s_*) 到 win_* 末尾

4. IIR 流式滤波 (仅新数据):
   - 对 5 个通道的 buf_1s_* (125 个样本) 分别执行 IIR 滤波
   - 状态在 biquad 实例中自然传递
   - 结果存入 iir_filt_1s_*

5. 滤波数据窗口平移:
   - memmove 5 个通道的 filt_* (后 7 秒前移 125)
   - memcpy iir_filt_1s_* 到 filt_* 末尾

6. 运动检测 (混合阈值方案):
   - 使用原始 win_accx/y/z 计算 ACC 幅值: sqrt(ax^2 + ay^2 + az^2)
   - 用 arm_std_f32 计算当前窗口的 acc_mag 标准差 -> acc_std
   - 首次运行: motion_threshold = Default_Motion_Th (硬编码默认值, 立即可用)
   - 后台自动标定 (持续运行, 不阻塞输出):
     a) 若 motion_calibrated == 0:
        - 若 acc_std < Default_Motion_Th (判定为静息):
          calib_std_accum += acc_std
          calib_windows_done++
        - 若 calib_windows_done >= 8 (收集到 8 个静息窗口):
          acc_baseline_std = calib_std_accum / calib_windows_done
          motion_threshold = Motion_Th_Scale * acc_baseline_std
          motion_calibrated = 1
        - 标定期间仍使用默认阈值进行运动判断 (不阻塞)
     b) 若已校准:
        - 使用标定后的 motion_threshold
     c) is_motion = (acc_std > motion_threshold) ? 1 : 0

7. 窗口填充检查:
   - 维护 win_count 计数器 (每步 +1)
   - 若 win_count < HR_WIN_SEC, 返回 0 (窗口未填满)
   - 首次填满后设置 win_filled = 1

8. 时延对齐 (在 filt_* 上操作):
   - 对每个 ACC 通道调用 DSP_FindDelay(filt_ppg, filt_accx, 1000, scratch, &corr)
   - 对 HF 通道调用 DSP_FindDelay(filt_ppg, filt_hf, 1000, scratch, &corr)
   - 注意: filt_* 缓冲区需要前后各多 5 个采样点以支持 ±5 偏移
   - 找最大相关通道, 确定时延 -> 计算 LMS 阶数:
     if delay < 0: order = max(1, min(Max_Order, floor(|delay| * factor)))
     else:         order = 1

9. 路径 A - LMS-HF:
   a) 拷贝 filt_ppg 到 scratch_a, scratch_b 作为 zscore 归一化暂存
   b) zscore 归一化 scratch_a (PPG) 和对应 HF 通道
   c) 2 级级联 LMS: LMS_CascadeProcess(..., lms_hf_tmp, lms_hf_tmp2)
   d) FFT 寻峰 (在 lms_hf_err 上)
   e) 频谱惩罚 (参考信号: HF 最佳通道的 FFT 峰值频率)
   f) 排序峰值 + 历史追踪 → hr_lms_hf

10. 路径 B - LMS-ACC:
    a) 同上, 使用 filt_ppg 和 ACC 通道
    b) 3 级级联 LMS: LMS_CascadeProcess(..., lms_acc_tmp, lms_acc_tmp2)
    c) FFT 寻峰 + 频谱惩罚 → hr_lms_acc

11. 路径 C - Pure FFT:
    a) 拷贝 filt_ppg 到 fft_input, 去均值
    b) 应用 Hamming 窗: arm_mult_f32(fft_input, hamming_win, fft_input, 1000)
    c) 零填充到 4096 点 (HR_Init 时 fft_input 已清零, 仅拷贝前 1000 点)
    d) FFT 寻峰 + 频谱惩罚 (参考: ACC Z轴)
    e) 历史追踪 → hr_fft

12. 融合决策:
    - motion → hr_fused = hr_lms_acc
    - rest   → hr_fused = hr_fft
    - (可选: 同时输出 hr_lms_hf 用于对比)

13. 中值平滑:
    - 对 hr_history 中的最近 5 个融合值取中值 → smoothed_hr
    - 再对平滑后序列的最近 3 个值取中值 (二次平滑)
    - (在线版简化: 仅维护环形缓冲区, 每步计算一次 5 点中值)

14. 更新心率历史:
    - hr_history[hr_history_idx] = smoothed_hr
    - hr_history_idx = (hr_history_idx + 1) % HR_HR_HISTORY_LEN

15. 转换 BPM: hr_bpm = smoothed_hr * 60
16. 返回 hr_bpm
```

- [ ] **Step 2: Verify compilation**

- [ ] **Step 3: Commit**

```bash
git add Core/Src/hr_algorithm.c
git commit -m "feat: implement HR algorithm orchestrator with 3-path fusion"
```

---

## Task 6: Integrate into `main.c`

**Files:**
- Modify: `Core/Src/main.c`
- Modify: `Core/Inc/main.h`

- [ ] **Step 1: Add algorithm include and global state to `main.c`**

In `main.c`, add after existing includes:
```c
#include "hr_algorithm.h"
```

Add global variables after existing variables:
```c
/* --- 在线算法相关 --- */
static HR_Config_t hr_config;
static HR_State_t  hr_state;
static uint8_t algorithm_initialized = 0;

/* ADC 原始数据缓存 (用于转换为 float) */
static int32_t adc_raw[4] = {0};
```

- [ ] **Step 2: Modify main loop to support foreground/background**

Replace the existing `while(1)` loop with the new architecture. The existing data sending logic is preserved but enhanced:

```c
while (1)
{
    /* ---- 后台: 心率算法处理 ---- */
    if (algorithm_initialized && hr_state.flag_1s_ready) {
        float bpm = HR_RunSolver(&hr_state);
        if (bpm > 0) {
            /* 将心率值通过 UART 发送 (调试用, 格式可自定义) */
            /* 例如: "$HR,72.5\r\n" */
        }
    }

    /* ---- 前台数据采集: ADC 完成后收集 ---- */
    if (ADC_1to4Voltage_flag == 4) {
        /* --- 读取 ACC 全量数据 (int16_t → float) --- */
        int16_t ax_raw = (int16_t)((ACC_XYZ[1] << 8) | ACC_XYZ[0]);
        int16_t ay_raw = (int16_t)((ACC_XYZ[3] << 8) | ACC_XYZ[2]);
        int16_t az_raw = (int16_t)((ACC_XYZ[5] << 8) | ACC_XYZ[4]);
        float ax = (float)ax_raw;
        float ay = (float)ay_raw;
        float az = (float)az_raw;

        /* --- 读取 PPG 均值 (MAX30101 FIFO 可能产出 ~2 样本/125Hz tick) --- */
        uint8_t wr_ptr = MAX_ReadOneByte(FIFO_WR_PTR_REG);
        uint8_t rd_ptr = MAX_ReadOneByte(FIFO_RD_PTR_REG);
        uint8_t sample_count = (wr_ptr - rd_ptr) & 0x1F;
        float ppg_val = 0.0f;
        if (sample_count > 0) {
            uint32_t sum_green = 0;
            uint8_t buf[3];
            for (uint8_t i = 0; i < sample_count; i++) {
                MAX_ReadFIFO_Burst(buf, 3);
                sum_green += ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x03FFFF;
            }
            ppg_val = (float)sum_green / (float)sample_count;
        }

        /* --- 读取 HF/ADC (从 ADS124) --- */
        float hf_val = (float)adc_raw[0]; /* 使用第一个 ADC 通道作为 HF */

        /* --- 推送到算法 --- */
        if (algorithm_initialized) {
            HR_PushSample(&hr_state, ppg_val, ax, ay, az, hf_val);
        }

        /* --- 保留原有数据发送逻辑 (可选, 调试阶段同时发送原始数据) --- */
        /* ... existing pack + DMA send ... */

        ADC_1to4Voltage_flag = 0;
    }
}
```

- [ ] **Step 3: Add algorithm initialization in `main()` setup phase**

After sensor init and before `HAL_TIM_Base_Start_IT`, add:
```c
/* 初始化心率算法 */
HR_GetDefaultConfig(&hr_config);
HR_Init(&hr_config, &hr_state);
algorithm_initialized = 1;
HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: HR Algorithm Init OK\r\n", 29, 1000);
```

- [ ] **Step 4: Verify compilation and basic functionality**

Build and verify:
- No compilation errors
- Algorithm state allocated (check .map file for ~58 KB BSS)
- UART debug messages appear

- [ ] **Step 5: Commit**

```bash
git add Core/Src/main.c Core/Inc/main.h
git commit -m "feat: integrate online HR algorithm into main loop"
```

---

## Task 7: Data Output Format for Debugging

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: Define debug output protocol**

When algorithm produces a valid HR result (every 1 second), send a debug packet via UART:

```c
/* 心率结果包格式 (12 字节):
 * [0xAA] [0xCC]                  帧头
 * [HR_H] [HR_L]                  心率 BPM (uint16, x10 精度 0.1)
 * [Motion]                       运动标志 (0/1)
 * [Win_Filled]                   窗口填充状态
 * [HR_LMS_HF_H] [HR_LMS_HF_L]   LMS-HF 路径 BPM (uint16, x10)
 * [HR_LMS_ACC_H] [HR_LMS_ACC_L] LMS-ACC 路径 BPM (uint16, x10)
 * [HR_FFT_H] [HR_FFT_L]         FFT 路径 BPM (uint16, x10)
 * [XOR]                          校验
 */
```

- [ ] **Step 2: Implement and verify with serial monitor**

- [ ] **Step 3: Commit**

```bash
git add Core/Src/main.c
git commit -m "feat: add HR result debug output protocol"
```

---

## Task 8: Fine-tune and Validate

**Files:**
- Modify: `Core/Src/hr_algorithm.c` (parameter adjustments)
- Modify: `Core/Src/hr_dsp.c` (BPF coefficient verification)

- [ ] **Step 1: Verify IIR bandpass filter coefficients**

Use MATLAB/Octave to compute exact coefficients:
```matlab
Fs = 125;
[b,a] = butter(4, [0.5 5]/(Fs/2), 'bandpass');
[sos,g] = tf2sos(b,a);
% 提取 sos 矩阵中的系数, 按行排列为 [b0, b1, b2, -a1, -a2]
```

Update `HR_BPF_COEFFS` in `hr_dsp.c` with verified values.

- [ ] **Step 2: End-to-end validation**

1. Flash firmware
2. Wear sensor, stay still for 8+ seconds (calibration)
3. Verify HR output stabilizes around expected resting HR (60-80 BPM)
4. Start moving, verify motion flag triggers and HR updates
5. Stop moving, verify HR returns to resting and motion flag clears

- [ ] **Step 3: Final commit**

```bash
git add -A
git commit -m "feat: validate and fine-tune online HR algorithm parameters"
```

---

## Key Implementation Notes

### 1. IIR Streaming Filter Strategy (CRITICAL)

**不能**对每个 8 秒窗口重新滤波. IIR 是因果流式滤波器, 状态必须在时间上连续.

**正确做法**:
1. **HR_Init**: 初始化 IIR biquad 实例, 状态清零
2. **每 1 秒步进**:
   - 仅将 `buf_1s_*` 的 125 个新原始样本通过 IIR (状态自然传递):
     ```c
     arm_biquad_cascade_df1_f32(&biquad_ppg, buf_1s_ppg, iir_filt_1s_ppg, 125);
     ```
   - 平移 `filt_ppg[]` 缓冲区: `memmove(filt_ppg, filt_ppg+125, 875*4)`
   - 追加新滤波数据: `memcpy(filt_ppg+875, iir_filt_1s_ppg, 125*4)`
3. **算法使用 `filt_*` 缓冲区** (已滤波的 8 秒数据), 不使用 `win_*` 原始数据
4. `win_*` 原始窗口仅用于运动检测 (ACC 幅值标准差) 和调试输出

**为什么需要两套缓冲区**: 原始 `win_*` 用于 IIR 滤波的输入, `filt_*` 保存滤波后的结果. LMS 和 FFT 都在 `filt_*` 上操作.

### 2. LMS State Management

CMSIS-DSP `arm_lms_norm_f32` processes blocks of data. The state buffer carries over between blocks. For cascaded operation:
- Each cascade level has its own `arm_lms_norm_instance_f32`
- The error output of level N becomes the desired input of level N+1
- Coefficients are NOT reset between windows (adaptive filter learns continuously)

### 3. FFT Input Preparation

For the FFT path:
1. Subtract mean from PPG window
2. Apply Hamming window: `arm_mult_f32` with precomputed Hamming coefficients
3. Zero-pad to 4096 points
4. Execute `arm_rfft_fast_f32`

For LMS outputs:
1. No windowing needed (already filtered)
2. Zero-pad to 4096 points
3. Execute RFFT

### 4. Zscore Normalization

Before LMS, inputs must be normalized. **关键**: zscore 会修改数据, 不能直接在 `filt_*` 缓冲区上操作 (后续路径还需要原始滤波数据).

```c
/* 使用 scratch_a / scratch_b 避免污染 filt_* 数据 */
/* 步骤1: 拷贝到 scratch */
memcpy(scratch_a, filt_ppg, WIN_SAMPLES * sizeof(float));
memcpy(scratch_b, ref_channel, WIN_SAMPLES * sizeof(float));

/* 步骤2: zscore 归一化 (就地修改 scratch) */
static void zscore_inplace(float *sig, uint16_t len) {
    float mean, std;
    arm_mean_f32(sig, len, &mean);
    arm_std_f32(sig, len, &std);
    if (std < 1e-6f) std = 1e-6f;
    arm_offset_f32(sig, -mean, sig, len);
    arm_scale_f32(sig, 1.0f/std, sig, len);
}
zscore_inplace(scratch_a, WIN_SAMPLES);
zscore_inplace(scratch_b, WIN_SAMPLES);

/* 步骤3: 传入 LMS */
LMS_Process(inst, scratch_b, scratch_a, err_out, tmp_out, WIN_SAMPLES);
```

每条 LMS 路径独立使用 scratch_a/b, 路径之间不冲突 (串行执行).

### 5. PPG 采样率匹配

MAX30101 在绿光心率模式下的采样链路:
- 内部采样率: 1000 sps (SPO2_CONFIG = 0x77, bit4-2 = 111)
- 硬件平均: 4 倍 (FIFO_CONFIG = 0x5F, bit6-5 = 10)
- 实际 FIFO 产出率: ~250 sps

主循环以 125 Hz 轮询 (由 ADC DRDY 中断驱动), 每次读取 FIFO 可能获得 ~2 个样本.
处理方式: 在 `ADC_1to4Voltage_flag == 4` 时, 读取所有 FIFO 样本并计算**均值**, 得到一个 PPG float 值, 传入 `HR_PushSample()`. 这样 PPG 数据与 ACC 数据同频 (125 Hz).

```c
/* PPG 采集 (在 main loop 的 ADC_1to4Voltage_flag==4 分支内) */
uint8_t wr_ptr = MAX_ReadOneByte(FIFO_WR_PTR_REG);
uint8_t rd_ptr = MAX_ReadOneByte(FIFO_RD_PTR_REG);
uint8_t sample_count = (wr_ptr - rd_ptr) & 0x1F;

float ppg_val = 0.0f;
if (sample_count > 0) {
    uint32_t sum_green = 0;
    uint8_t buf[3];
    for (uint8_t i = 0; i < sample_count; i++) {
        MAX_ReadFIFO_Burst(buf, 3);
        sum_green += ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x03FFFF;
    }
    ppg_val = (float)sum_green / (float)sample_count;
}
```
