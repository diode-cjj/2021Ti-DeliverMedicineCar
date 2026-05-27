#ifndef __CARCONFIG_H
#define __CARCONFIG_H

#include "main.h"
#include "StepperMotor.h"

/* 编码器方向修正：把左右轮编码器读数统一成前进为正。 */
#define ENCODER_LEFT_DIR          -1    /* 左轮编码器方向系数，-1表示读数取反后才是前进为正。 */
#define ENCODER_RIGHT_DIR          1    /* 右轮编码器方向系数，1表示读数本身就是前进为正。 */

/* 测试/运行状态机编号。 */
#define TEST_IDLE                  0U   /* 空闲状态，等待按键选择或启动模式。 */
#define TEST_BALL_TRACK            1U   /* 追球模式运行状态。 */
#define TEST_TIMED_TRACK           2U   /* 定时轨迹模式运行状态。 */
#define TEST_COMBO                 3U   /* 组合轨迹模式运行状态。 */
#define TEST_BRAKE                 4U   /* 刹车状态，车辆正在减速停止。 */

/* 菜单可选择的三种运行模式。 */
#define MODE_COMBO                 1U   /* 菜单模式1：组合轨迹。 */
#define MODE_TIMED_TRACK           2U   /* 菜单模式2：定时轨迹。 */
#define MODE_BALL_TRACK            3U   /* 菜单模式3：追球。 */

/* 组合/定时轨迹的分段步骤。 */
#define COMBO_STEP_IDLE            0U   /* 轨迹未运行或已结束。 */
#define COMBO_STEP_STRAIGHT1       1U   /* 第一段直行。 */
#define COMBO_STEP_TURN            2U   /* 中间右转弯。 */
#define COMBO_STEP_STRAIGHT2       3U   /* 第二段直行。 */

/* 云台搜索方向定义。 */
#define BALL_SEARCH_LEFT          -1    /* 水平云台向左搜索。 */
#define BALL_SEARCH_RIGHT          1    /* 水平云台向右搜索。 */
#define BALL_SEARCH_UP            -1    /* 俯仰云台向上搜索。 */
#define BALL_SEARCH_DOWN           1    /* 俯仰云台向下搜索。 */

/* 追球流程状态：搜索、转向、前进、车体旋转搜索。 */
#define BALL_STEP_SEARCH           0U   /* 云台原地扫描寻找目标。 */
#define BALL_STEP_TURN             1U   /* 车体转向目标方向。 */
#define BALL_STEP_FORWARD          2U   /* 车辆向目标前进。 */
#define BALL_STEP_ROTATE_SEARCH    3U   /* 云台扫不到目标时，车体旋转扩大搜索范围。 */

/* 直行基础速度和定时轨迹时间参数。 */
#define STRAIGHT_SPEED             30.0f /* 组合模式直行目标速度。 */

#define TRACK_TIME_MIN             500U  /* 定时轨迹最短设定时间，显示为5秒。 */
#define TRACK_TIME_MAX             1000U /* 定时轨迹最长设定时间，显示为10秒。 */
#define TRACK_TIME_STEP            100U  /* 每按一次K3增加的设定时间，显示增加1秒。 */
#define TRACK_BASE_TIME            312U  /* 定时轨迹速度缩放参考时间。 */
#define TRACK_STOP_ADVANCE_TIME    40U   /* 定时轨迹提前刹车补偿时间。 */
#define TRACK_STRAIGHT1_REDUCE_TIME 8U   /* 从第一段直行转移到第二段直行的时间补偿。 */

/* 模式1轨迹参数：两段直行时间、转弯半径、轮距和转弯中心速度。 */
#define COMBO_STRAIGHT1_TIME       149U  /* 组合模式第一段直行持续时间，单位为10ms任务次数。 */
#define COMBO_STRAIGHT2_TIME       62U   /* 组合模式第二段直行持续时间，单位为10ms任务次数。 */
#define COMBO_TURN_RADIUS_CM       20.0f /* 组合模式转弯中心半径，单位cm。 */
#define CAR_TRACK_WIDTH_CM         20.0f /* 小车左右轮距，单位cm。 */
#define COMBO_TURN_CENTER_SPEED    20.0f /* 转弯时车体中心目标速度。 */
#define COMBO_TURN_MAX_TIME        500U  /* 转弯最长允许时间，单位为10ms任务次数。 */

/* 蜂鸣器引脚和单次鸣叫持续时间。 */
#define BUZZER_PIN                 GPIO_PIN_12 /* 蜂鸣器连接的GPIO引脚。 */
#define BUZZER_GPIO_PORT           GPIOD       /* 蜂鸣器所在GPIO端口。 */
#define BUZZER_TIME                20U         /* 蜂鸣器单次鸣叫时长，单位为10ms任务次数。 */

/* 云台步进电机一圈脉冲数。 */
#define STEPPER_BASE_360_PULSE     400U  /* 水平底座电机转一圈所需脉冲数。 */
#define STEPPER_TOP_360_PULSE      400U  /* 俯仰电机转一圈所需脉冲数。 */

/* 追球云台限位、步进方向和单次调整脉冲。 */
#define BALL_BASE_HALF_RANGE_PULSE (STEPPER_BASE_360_PULSE / 2U)        /* 水平云台左右半圈搜索范围。 */
#define BALL_TOP_UP_RANGE_PULSE    (STEPPER_TOP_360_PULSE * 60U / 360U) /* 俯仰云台向上最大搜索范围，约60度。 */
#define BALL_TOP_DOWN_RANGE_PULSE  (STEPPER_TOP_360_PULSE * 60U / 360U) /* 俯仰云台向下最大搜索范围，约60度。 */
#define BALL_BASE_RIGHT_DIR        STEPPERMOTOR_DIR_BACK                /* 水平云台向右时使用的步进方向。 */
#define BALL_BASE_LEFT_DIR         STEPPERMOTOR_DIR_FORWARD             /* 水平云台向左时使用的步进方向。 */
#define BALL_TOP_DOWN_DIR          STEPPERMOTOR_DIR_FORWARD             /* 俯仰云台向下时使用的步进方向。 */
#define BALL_TOP_UP_DIR            STEPPERMOTOR_DIR_BACK                /* 俯仰云台向上时使用的步进方向。 */
#define BALL_SEARCH_PULSE          1U                                   /* 搜索时每次移动的脉冲数。 */
#define BALL_TRACK_BASE_PULSE      1U                                   /* 跟踪时水平云台每次调整的脉冲数。 */
#define BALL_TRACK_TOP_PULSE       1U                                   /* 跟踪时俯仰云台每次调整的脉冲数。 */
#define BALL_TOP_SEARCH_PERIOD     2U                                   /* 俯仰云台每隔多少次搜索调用移动一次。 */
#define BALL_BASE_DEG_PER_PULSE    (360.0f / (float)STEPPER_BASE_360_PULSE) /* 水平云台每个脉冲对应角度。 */

/* K230图像中心、目标死区、停车距离和追球速度参数。 */
#define BALL_IMAGE_CENTER_X        160   /* K230图像水平中心像素坐标。 */
#define BALL_IMAGE_CENTER_Y        120   /* K230图像垂直中心像素坐标。 */
#define BALL_X_DEADZONE            10    /* 水平方向误差死区，单位像素。 */
#define BALL_Y_DEADZONE            12    /* 垂直方向误差死区，单位像素。 */
#define BALL_LOCK_X_ERROR          18    /* 判断目标水平锁定的最大误差，单位像素。 */
#define BALL_CENTER_BASE_STEP      6     /* 水平云台回中允许误差，单位步。 */
#define BALL_FORWARD_RECENTER_STEP 18    /* 前进时允许的云台偏移步数。 */
#define BALL_FRONT_STOP_DISTANCE_CM 3U   /* 车头距离目标的期望停车距离，单位cm。 */
#define BALL_CAMERA_FRONT_OFFSET_CM 14U  /* 摄像头到车头的前后偏移距离，单位cm。 */
#define BALL_STOP_DISTANCE_CM      (BALL_FRONT_STOP_DISTANCE_CM + BALL_CAMERA_FRONT_OFFSET_CM) /* 摄像头测距下的停车距离阈值，单位cm。 */
#define BALL_STOP_WIDTH_MIN        35    /* 停车时目标在图像中的最小宽度，防止远处误停。 */
#define BALL_SLOW_DISTANCE_CM      25U   /* 小于该距离后降低追球速度，单位cm。 */
#define BALL_LOST_TIMEOUT_MS       600U  /* 超过该时间未收到目标数据则认为目标丢失，单位ms。 */
#define BALL_FORWARD_SPEED         14.0f /* 追球前进正常速度。 */
#define BALL_ALIGN_SPEED           6.0f  /* 前进时左右差速修正的最大速度。 */
#define BALL_TURN_ONLY_SPEED       6.0f  /* 原地转向目标时的速度。 */
#define BALL_SLOW_SPEED            12.0f /* 接近目标后的低速前进速度。 */
#define BALL_BODY_TURN_SPEED       8.0f  /* 车体转向目标方向时的差速速度。 */
#define BALL_ROTATE_SEARCH_SPEED   6.0f  /* 车体旋转搜索目标时的差速速度。 */
#define BALL_FORWARD_TURN_KP       0.03f /* 前进追球时由像素误差转成差速修正的比例系数。 */
#define BALL_TURN_ALLOW_ERROR      3.0f  /* 车体转向允许角度误差，单位度。 */
#define BALL_TURN_MAX_TIME         300U  /* 追球转向最长允许时间，单位为10ms任务次数。 */
#define BALL_SEARCH_EDGE_COUNT     2U    /* 云台扫到水平边界多少次后切换为车体旋转搜索。 */

/* 刹车控制参数。 */
#define BRAKE_TIME                 20U   /* 刹车持续时间，单位为10ms任务次数。 */
#define BRAKE_STOP_SPEED           2     /* 低于该编码器速度时认为对应车轮已接近停止。 */
#define BRAKE_MIN_OUT              80.0f /* 车轮仍在转动时施加的最小反向制动力。 */

/* 右转目标角度、允许误差和超时保护。 */
#define TURN_TARGET_ANGLE          88.0f /* 轨迹右转目标角度，单位度。 */
#define TURN_ALLOW_ERROR           2.0f  /* 转弯完成允许角度误差，单位度。 */
#define TURN_STABLE_TIME           10U   /* 预留的转弯稳定时间参数。 */
#define TURN_MAX_TIME              500U  /* 转弯最长允许时间，单位为10ms任务次数。 */

/* 蓝牙调试输出周期，单位为10ms任务次数。 */
#define DEBUG_PERIOD               20U   /* 每20次10ms任务输出一次调试信息，即约200ms。 */

/* 速度环PID参数。 */
#define SPEED_KP                   1.5f  /* 速度环比例系数。 */
#define SPEED_KI                   0.5f  /* 速度环积分系数。 */
#define SPEED_KD                   0.0f  /* 速度环微分系数。 */
#define SPEED_MAX_OUT              1000.0f /* 速度环输出最大值。 */
#define SPEED_MAX_INT              1000.0f /* 速度环积分限幅。 */
#define SPEED_LEFT_RATIO           1.02f /* 左轮目标速度补偿系数。 */

/* 角度环PID参数。 */
#define ANGLE_KP                   1.0f  /* 角度环比例系数。 */
#define ANGLE_KI                   0.0f  /* 角度环积分系数。 */
#define ANGLE_KD                   6.0f  /* 角度环微分系数。 */
#define ANGLE_MIN_OUT              8.0f  /* 角度环最小有效输出。 */
#define ANGLE_MAX_OUT              25.0f /* 角度环输出最大值。 */

/* 灰度传感器黑线阈值，数值越小表示越接近黑色。 */
#define GRAY_BLACK_VALUE           450U  /* 灰度值小于等于该阈值时认为检测到黑线。 */

#endif
