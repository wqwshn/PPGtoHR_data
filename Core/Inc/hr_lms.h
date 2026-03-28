/**
 * @file    hr_lms.h
 * @brief   LMS 自适应滤波器声明: 单级初始化/处理、级联处理
 * @details 基于 CMSIS-DSP 的 arm_lms_norm_f32 (归一化 LMS) 实现封装.
 *          支持多级级联, 用于从 PPG 信号中消除运动伪影.
 *          所有状态缓冲区由调用者静态分配, 不使用动态内存.
 */

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
 * @param coeffs:  FIR 系数数组 (长度 = num_taps, 内部清零)
 * @param state:   状态缓冲区 (长度 = num_taps + block_size - 1, 内部清零)
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
 *
 * 处理流程:
 *   第 0 级: ref_channels[0] vs ppg -> err_out
 *   第 i 级 (i>=1): ref_channels[i] vs (上一级 err_out 拷贝) -> err_out
 *   使用 tmp_out 和 tmp_out2 交替作为中间缓冲区, 避免读写冲突.
 *
 * @param instances:   LMS 实例数组
 * @param num_cascade: 级联级数
 * @param ref_channels:参考信号通道指针数组 (每个元素指向一个 HR_WIN_SAMPLES 长度的缓冲区)
 * @param ppg:         输入 PPG 信号 (HR_WIN_SAMPLES 点)
 * @param err_out:     最终误差输出 (HR_WIN_SAMPLES 点)
 * @param tmp_out:     临时缓冲区 A (HR_WIN_SAMPLES 点)
 * @param tmp_out2:    临时缓冲区 B (HR_WIN_SAMPLES 点, 与 tmp_out 交替用于级联)
 * @param block_len:   数据块长度
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
