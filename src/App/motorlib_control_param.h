#ifndef MOTORLIB_CONTROL_PARAM_H
#define MOTORLIB_CONTROL_PARAM_H

/*硬件相关*/
#define PHASE_CURRENT_GAIN   30.0f      /* 相电流增益 */
#define BUS_CURRENT_GAIN     0.01f      /* 母线电流增益 */
#define BUS_VOLTAGE_GAIN     0.0112793f /* 母线电压增益 */
#define ENCODER_RESOLUTION   16384      /* 编码器分辨率 */
#define ENCODER_RESOLUTION_F (16384.0f) /* 编码器分辨率 */

/*控制相关     */
#define CONTROL_LOOP_FREQ 20000.0f                   /* 控制循环频率 20kHz */
#define CONTROL_PERIOD_DT (1.0f / CONTROL_LOOP_FREQ) /* 控制周期 50us */
#define SPEED_LOOP_FREQ   1000.0f                    /* 速度环频率 1kHz */
#define SPEED_PERIOD_DT   (1.0f / SPEED_LOOP_FREQ)   /* 速度环周期 1ms */
#define SPEED_LOOP_INTERVAL                                                                        \
	(SPEED_PERIOD_DT / CONTROL_PERIOD_DT) /* 速度环执行间隔，单位为控制周期数 */

#define MIN_MECH_ROUNDS 0.02f
#define MIN_ELEC_ROUNDS 0.20f
#define ALIGN_VOLTAGE   0.15f
#define ROTATE_VOLTAGE  (ALIGN_VOLTAGE)
#define ALIGN_TIMEOUT   2000 /* 对齐阶段超时时间 ms */
#define ROTATE_TIMEOUT  5000 /* 旋转阶段超时时间 ms */
#define ROTATE_SPEED    (1.0f * M_PI)
/* 预计算的 timeout tick 阈值（避免运行时浮点运算） */
#define ALIGN_TIMEOUT_TICKS                                                                        \
	((uint32_t)(ALIGN_TIMEOUT / (CONTROL_PERIOD_DT * 1000.0f))) /* 2000ms / 0.05ms = 40000 */
#define ROTATE_TIMEOUT_TICKS                                                                       \
	((uint32_t)(ROTATE_TIMEOUT / (CONTROL_PERIOD_DT * 1000.0f))) /* 5000ms / 0.05ms = 100000 */

#define CURRMENT_LOOP_KP    (0.1f)   /* 电流环P增益 */
#define CURRMENT_LOOP_KI    (800.0f) /* 电流环I增益 */
#define CURRMENT_LOOP_LIMIT (13.0f)  /* 电流调试输出间隔，单位为控制周期数 */

#define SPEED_LOOP_KP    (0.05f)
#define SPEED_LOOP_KI    (6.0f)
#define SPEED_LOOP_LIMIT (13.0f)
#endif /* MOTORLIB_CONTROL_PARAM_H */
