/*
 * config.h
 *
 *  Created on: Aug 21, 2025
 *      Author:
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#ifndef USE_INWHEEL
#define USE_INWHEEL 1
#endif

// ---------- Math ----------
#ifndef PI
#define PI 3.14159f
#endif

// ---------- Mechanics ----------
// 바퀴 지름 [m] — 하드웨어에 맞게 수정
#ifndef WHEEL_DIAMETER_M
#define WHEEL_DIAMETER_M 0.2000f // 예: 8-inch ≈ 0.2032 m
#endif
#define WHEEL_CIRCUM_M (PI * (WHEEL_DIAMETER_M))

// Hall 엣지 수 (한 바퀴당 인터럽트 수
//소형 BLDC모터:24.0f, 인휠모터:90.0f로 수정
#ifndef HALL_EDGES_PER_REV
#define HALL_EDGES_PER_REV 90.0f
#endif

// 1: 시계방향(CW), 0: 반시계(CCW)
#ifndef MOTOR_DIR_CW
#define MOTOR_DIR_CW 1
#define MOTOR_DIR_CCW 0
#endif

#ifndef MOTOR_DIR
#define MOTOR_DIR MOTOR_DIR_CW
#endif

// ---------- ADC & Sensors ----------
#define ADC_VREF 3.3f
#define ADC_FS 4095.0f
#define VDIV_RATIO 0.057362f // Vdc 측정 저항분배 비율
#define OFFSET_Volt 1.65f
#define OPAMP_GAIN 0.044f
// ---------- Thresholds ----------
#define THROTTLE_OFF 1.00f
#define THROTTLE_ON 1.05f

#define OC_LEVEL 35.0f
#define OC_TRIP_COUNT 50 // 2.5ms@20kHz 예시
#define RPM_TO_KMH(rpm) ((rpm) * (WHEEL_CIRCUM_M) * 60.0f / 1000.0f)
#endif /* INC_CONFIG_H_ */
