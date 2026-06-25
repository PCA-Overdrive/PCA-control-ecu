#ifndef APP_AUTO_EXIT_CONFIG_H
#define APP_AUTO_EXIT_CONFIG_H

#include "App_Types.h"

#define APP_AUTO_EXIT_SERVICE_PERIOD_MS 10u

#define APP_AUTO_EXIT_DRIVE_STOP       127u
#define APP_AUTO_EXIT_STEER_CENTER     127u

#define APP_AUTO_EXIT_DRIVE_FORWARD    80u
#define APP_AUTO_EXIT_DRIVE_REVERSE    200u

#define APP_AUTO_EXIT_STEER_LEFT       0u
#define APP_AUTO_EXIT_STEER_RIGHT      255u

#define APP_AUTO_EXIT_SHIFT_STOP_MS    300u
#define APP_AUTO_EXIT_FORWARD_1_MS     3000u
#define APP_AUTO_EXIT_TURN_1_MS        2500u
#define APP_AUTO_EXIT_REVERSE_1_MS     1800u
#define APP_AUTO_EXIT_TURN_2_MS        2700u
#define APP_AUTO_EXIT_REVERSE_TURN_MS  2400u
#define APP_AUTO_EXIT_TURN_3_MS        3000u
#define APP_AUTO_EXIT_FORWARD_2_MS     500u
#define APP_AUTO_EXIT_FINAL_STOP_MS    700u
#define APP_AUTO_EXIT_MIN_STEP_MS      100u

#define APP_AUTO_EXIT_START_STOP_MS          300u
/*
 * 측면 회피 판단 기준
 *
 * CRITICAL:
 * - 이보다 가까우면 너무 위험한 거리
 * - 짧은 회피로는 부족하므로 LONG 회피 후보
 *
 * NEAR:
 * - 이보다 가까우면 거리만으로 AVOID 필요
 *
 * FAR_SAFE:
 * - 앞/뒤 둘 다 이보다 멀면 기울어져 있어도 NORMAL
 *
 * TILT_DIFF:
 * - 애매한 거리 구간에서 앞/뒤 거리 차이가 이 이상이면
 *   옆차가 기울어진 것으로 판단
 *
 * SENSOR_SPACING:
 * - 측면 앞 초음파와 뒤 초음파 사이의 차체 길이 방향 거리
 *
 * FRONT_OVERHANG:
 * - 측면 앞 초음파 위치에서 실제 앞코너까지의 거리
 */
#define APP_AUTO_EXIT_SIDE_CRITICAL_MM           180u
#define APP_AUTO_EXIT_SIDE_NEAR_MM               450u
#define APP_AUTO_EXIT_SIDE_FAR_SAFE_MM           650u
#define APP_AUTO_EXIT_SIDE_TILT_DIFF_MM           80u

#define APP_AUTO_EXIT_SIDE_SENSOR_SPACING_MM     700u
#define APP_AUTO_EXIT_SIDE_FRONT_OVERHANG_MM     200u

#define APP_AUTO_EXIT_AVOID_ESCAPE_MIN_MS    250u
#define APP_AUTO_EXIT_AVOID_ESCAPE_SHORT_MS  700u
#define APP_AUTO_EXIT_AVOID_ESCAPE_LONG_MS   1500u

#define APP_AUTO_EXIT_AVOID_REALIGN_MIN_MS   350u

#define APP_AUTO_EXIT_AVOID_REALIGN_SHORT_MS 700u
#define APP_AUTO_EXIT_AVOID_REALIGN_LONG_MS  1800u

#define APP_AUTO_EXIT_REALIGN_LEFT_STEER     64u
#define APP_AUTO_EXIT_REALIGN_RIGHT_STEER    191u

#define APP_AUTO_EXIT_AVOID_ESCAPE_FORWARD_RATIO_PERCENT  60u
#define APP_AUTO_EXIT_AVOID_REALIGN_FORWARD_RATIO_PERCENT 60u

#define APP_AUTO_EXIT_STATUS_TX_PERIOD_MS (100u)
#define APP_AUTO_EXIT_RESULT_HOLD_MS      (2000u)

#define APP_AUTO_EXIT_YAW_VALIDATION_ENABLE    (0u)
#define APP_AUTO_EXIT_BASE_TURN_DEG            (90)
#define APP_AUTO_EXIT_TARGET_TURN_MIN_DEG      (45)
#define APP_AUTO_EXIT_TARGET_TURN_MAX_DEG      (135)
#define APP_AUTO_EXIT_YAW_TARGET_TOL_DEG       (15)
#define APP_AUTO_EXIT_IMU_RIGHT_SIGN           (1)

#define APP_AUTO_EXIT_ARRAY_COUNT(array) \
    ((uint32)(sizeof(array) / sizeof((array)[0])))

#endif /* APP_AUTO_EXIT_CONFIG_H */
