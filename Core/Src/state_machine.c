#include "state_machine.h"
#include "pid_speed.h"
#include "pid_line.h"
#include "pid_angle.h"
#include "Gray_Sensor.h"
#include "vision.h"
#include <math.h>

/* 外部变量：当前航向角，由 main.c 中的陀螺仪积分得到 */
extern float current_yaw;

/* 巡线基础速度，数值越大直线速度越快 */
#define BASE_SPEED 30.0f

/* 转弯后重新找线时的慢速前进速度 */
#define ALIGN_SPEED 12.0f

/* 起步或扫线成功后，前 50 个控制周期使用慢速对齐 */
#define ALIGN_TICKS 50

/* 识别到路口后继续前冲的控制周期数 */
#define FORWARD_TIME_TICKS 15

/* 识别到路口后前冲使用的速度 */
#define FORWARD_AFTER_CROSS_SPEED 25.0f

/* 连续多少个周期检测不到线才确认到达终点 */
#define LINE_END_CONFIRM_TICKS 15

/* 停车等待时间，用于转向前后稳定车身 */
#define WAIT_TICKS 50

/* 原地左右扫线时的轮速 */
#define SCAN_SPEED 8.0f

/* 扫线时允许左右摆动的最大航向角度 */
#define SCAN_YAW_LIMIT 8.0f

/* 灰度误差小于该值时认为车头已经对准红线 */
#define SCAN_CENTER_ERROR 0.5f

/* 巡线中心补偿，用于修正小车实际偏左的问题 */
#define LINE_CENTER_BIAS 0.12f

/* K230 左右准备信号有效时间，100 个 10ms 周期约等于 1s */
#define PREPARED_SIDE_VALID_TICKS 100U

/* 最多记录三次去程转向，用于返程反向回放 */
#define MAX_ROUTE_TURNS 3U

/* 当前状态函数指针，每次 State_Machine_Run 调用它指向的状态 */
void (*Current_State)(void);

/* 空闲状态需要给 main.c 判断零漂追踪条件 */
void State_Idle(void);

/* 去程状态：从起点装药后出发，到达病房终点 */
static void State_WaitLoad(void);
static void State_FollowMain(void);
static void State_ForwardAfterIntersection(void);
static void State_StopBeforeTurn(void);
static void State_Turn(void);
static void State_WaitAfterTurn(void);
static void State_ScanLine(void);
static void State_FollowToEnd(void);
static void State_WaitUnload(void);

/* 返程状态：卸药后 180° 转身，再按反向路径回到起点 */
static void State_ReturnTurn180(void);
static void State_ReturnFollowToIntersection(void);
static void State_ReturnForwardAfterIntersection(void);
static void State_ReturnStopBeforeTurn(void);
static void State_ReturnTurn(void);
static void State_ReturnWaitAfterTurn(void);
static void State_ReturnScanLine(void);
static void State_ReturnFollowToEnd(void);

/* 调试状态：蓝牙发送 turn 命令时进入角度转向测试 */
static void State_Debug_Turn(void);

/* 药品状态：空载、已装药、已卸药 */
#define MEDICINE_EMPTY   0U
#define MEDICINE_LOADED  1U
#define MEDICINE_REMOVED 2U

/* 应用状态：主要用于 OLED 显示当前比赛流程 */
#define APP_IDLE          0U
#define APP_WAIT_TARGET   1U
#define APP_WAIT_LOAD     2U
#define APP_OUTBOUND      3U
#define APP_WAIT_UNLOAD   4U
#define APP_RETURNING     5U
#define APP_FINISHED      6U

/* 转向命令：左转、右转、掉头和无转向 */
#define TURN_NONE   0
#define TURN_LEFT   1
#define TURN_RIGHT -1
#define TURN_BACK   2

typedef struct {
    uint8_t mode;                  /* 当前模式：1 近端，2 中端，3 远端 */
    uint8_t app_state;             /* 当前应用流程状态，用于 OLED 显示 */
    uint8_t medicine;              /* 当前药品状态，由 K3 按键模拟 */
    uint8_t target_digit;          /* K230 在起点识别到的目标房间号 */
    uint8_t intersection_count;    /* 灰度传感器累计识别到的路口数量 */
    uint8_t target_intersection;   /* 当前模式需要决策的目标路口 */
    uint8_t start_confirmed;       /* K2 启动确认标志 */
    uint8_t far_first_side;        /* 模式三第一次远端分岔时记录的左右方向 */
    uint8_t prepared_side;         /* K230 提前识别到的目标在左侧或右侧 */
    uint16_t prepared_side_timer;  /* prepared_side 已经保存的 10ms 周期数 */
    uint8_t outbound_turn_count;   /* 去程已经记录的转向次数 */
    uint8_t return_turn_count;     /* 返程需要执行的转向次数 */
    uint8_t return_turn_index;     /* 返程当前执行到第几个转向 */
    int8_t pending_turn;           /* 当前等待执行的转向命令 */
    int8_t outbound_turns[MAX_ROUTE_TURNS]; /* 去程转向记录，用于返程反推 */
    int8_t return_turns[MAX_ROUTE_TURNS];   /* 返程转向表，由去程转向反向生成 */
    char route_code;               /* 路线代号，显示在 OLED 上方便调试 */
} ContestContext;

/* 比赛上下文变量，所有状态共用这一份任务信息 */
static ContestContext car;

/* 路口前冲计时器 */
static int fwd_timer;

/* 终点脱线确认计时器 */
static int line_end_timer;

/* 巡线低速对齐计时器 */
static int align_timer;

/* 转向前后停车等待计时器 */
static int wait_timer;

/* 路口锁存标志，防止同一个路口被连续计数多次 */
static uint8_t intersection_latched;

/* 扫线开始时的航向角，用来限制左右摆动范围 */
static float scan_base_yaw;

/* 扫线方向，1 表示左轮正右轮反，-1 表示相反方向 */
static int scan_dir = 1;

/* 停车等待结束后真正要执行的转向命令 */
static int8_t turn_after_stop;

static void SetMedicineIndicators(void)
{
    /* 装药后点亮绿色指示灯 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (car.medicine == MEDICINE_LOADED) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 卸药后点亮红色指示灯 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (car.medicine == MEDICINE_REMOVED) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ResetRuntimeCounters(void)
{
    /* 清零路口前冲、终点确认、低速对齐和停车等待计时器 */
    fwd_timer = 0;
    line_end_timer = 0;
    align_timer = 0;
    wait_timer = 0;

    /* 清除路口锁存，允许下一个路口重新计数 */
    intersection_latched = 0;
}

static void Helper_SetTurnTarget(float angle_offset)
{
    /* 目标角度等于当前航向角加上本次要转的角度 */
    float target = current_yaw + angle_offset;

    /* 把目标角度限制在 -180° 到 +180°，避免跨界时 PID 算错方向 */
    while (target > 180.0f) target -= 360.0f;
    while (target < -180.0f) target += 360.0f;

    /* 通知角度 PID 开始转向 */
    PID_Angle_StartTurn(target);
}

static int8_t ReverseTurn(int8_t turn)
{
    /* 去程左转，返程同一路口需要右转 */
    if (turn == TURN_LEFT) return TURN_RIGHT;

    /* 去程右转，返程同一路口需要左转 */
    if (turn == TURN_RIGHT) return TURN_LEFT;

    /* 掉头和无转向保持原样 */
    return turn;
}

static void RecordOutboundTurn(int8_t turn)
{
    /* 记录去程每一次转向，返程时按相反顺序执行反向转向 */
    if (car.outbound_turn_count < MAX_ROUTE_TURNS) {
        car.outbound_turns[car.outbound_turn_count++] = turn;
    }
}

static void StartTurn(int8_t turn)
{
    /* 去程转向先记录下来，保证返程能按原路返回 */
    RecordOutboundTurn(turn);

    /* 保存当前要执行的转向，停车等待结束后再真正开始转 */
    car.pending_turn = turn;
    turn_after_stop = turn;

    /* 让停车状态第一次进入时可以直接开始等待计时 */
    wait_timer = WAIT_TICKS;

    /* 决策完成后先停车，避免边走边转导致扫线失败 */
    PID_Speed_SetTarget(0, 0);
    Current_State = State_StopBeforeTurn;
}

static void BeginStoppedTurn(int8_t turn, void (*next_state)(void))
{
    /* 根据转向命令设置角度 PID 的目标角度 */
    if (turn == TURN_LEFT) {
        Helper_SetTurnTarget(90.0f);
    } else if (turn == TURN_RIGHT) {
        Helper_SetTurnTarget(-90.0f);
    } else if (turn == TURN_BACK) {
        Helper_SetTurnTarget(180.0f);
    }

    /* 进入实际转向状态 */
    Current_State = next_state;
}

static uint8_t FollowToIntersectionStep(void)
{
    /* 读取灰度误差，正常值用于巡线，888 表示路口，999 表示脱线 */
    float err = Get_Grayscale_Error();

    /* 只有正常巡线误差才加中心补偿，特殊标志不能修改 */
    if (err != 888.0f && err != 999.0f) err -= LINE_CENTER_BIAS;

    /* 检测到路口时只触发一次，避免同一个路口反复累加 */
    if (err == 888.0f) {
        if (!intersection_latched) {
            intersection_latched = 1U;
            car.intersection_count++;
            return 1U;
        }
    } else {
        intersection_latched = 0U;
    }

    /* 正常在线上时使用巡线 PID 修正左右轮速度 */
    if (err != 999.0f && err != 888.0f) {
        float turn = PID_Line_GetTurn(err);
        float speed = (align_timer < ALIGN_TICKS) ? ALIGN_SPEED : BASE_SPEED;
        align_timer++;
        PID_Speed_SetTarget(speed + turn, speed - turn);
    } else {
        /* 路口或短暂脱线时先直行，防止车在路口乱转 */
        PID_Speed_SetTarget(BASE_SPEED, BASE_SPEED);
    }
    return 0U;
}

static uint8_t ForwardTicksStep(void)
{
    /* 路口处继续向前走一点，让车身中心越过路口再转向 */
    PID_Speed_SetTarget(FORWARD_AFTER_CROSS_SPEED, FORWARD_AFTER_CROSS_SPEED);

    /* 前冲时间到达设定值后返回 1，通知状态机进入下一步 */
    fwd_timer++;
    return fwd_timer >= FORWARD_TIME_TICKS;
}

static uint8_t FollowToEndStep(void)
{
    /* 终点没有红线，连续检测到脱线才认为真正到达终点 */
    float err = Get_Grayscale_Error();
    if (err == 999.0f) {
        PID_Speed_SetTarget(0, 0);
        if (++line_end_timer >= LINE_END_CONFIRM_TICKS) {
            return 1U;
        }
    } else {
        /* 只要重新看到线，就清零终点确认计时 */
        line_end_timer = 0;

        /* 路口宽红线按中心线处理，正常线则加中心补偿 */
        float safe_err = (err == 888.0f) ? 0.0f : (err - LINE_CENTER_BIAS);
        float turn = PID_Line_GetTurn(safe_err);
        float speed = (align_timer < ALIGN_TICKS) ? ALIGN_SPEED : BASE_SPEED;
        align_timer++;
        PID_Speed_SetTarget(speed + turn, speed - turn);
    }
    return 0U;
}

static void PrepareScan(int dir)
{
    /* 记录扫线开始时的航向角，后面左右摆动不能超过设定角度 */
    scan_base_yaw = current_yaw;

    /* 设置第一次扫线方向 */
    scan_dir = dir;

    /* 清零等待计时器，避免上一个状态的计时影响扫线 */
    wait_timer = 0;
}

static uint8_t ScanLineStep(void)
{
    /* 扫线过程中不断读取灰度误差 */
    float err = Get_Grayscale_Error();

    /* 如果还压在路口宽红线上，就慢慢往前离开路口，不立即判定扫线成功 */
    if (err == 888.0f) {
        PID_Speed_SetTarget(ALIGN_SPEED, ALIGN_SPEED);
        return 0U;
    }

    /* 找到正常红线且车头接近中心时，扫线完成 */
    if (err != 999.0f && fabs(err) <= SCAN_CENTER_ERROR) {
        align_timer = 0;
        return 1U;
    }

    /* 没找到线时原地左右小角度摆动 */
    PID_Speed_SetTarget(SCAN_SPEED * scan_dir, -SCAN_SPEED * scan_dir);

    /* 计算当前航向角相对扫线起点偏了多少度 */
    float yaw_diff = current_yaw - scan_base_yaw;
    while (yaw_diff > 180.0f) yaw_diff -= 360.0f;
    while (yaw_diff < -180.0f) yaw_diff += 360.0f;

    /* 到达左侧角度限制后改向右扫 */
    if (yaw_diff >= SCAN_YAW_LIMIT) scan_dir = -1;

    /* 到达右侧角度限制后改向左扫 */
    if (yaw_diff <= -SCAN_YAW_LIMIT) scan_dir = 1;
    return 0U;
}

/* K1/K2/K3 按键事件标志，按下后由对应状态取走 */
static uint8_t k1_event;
static uint8_t k2_event;
static uint8_t k3_event;

void State_Machine_ScanKeys10ms(void)
{
    /* 保存上一轮按键电平，用于检测下降沿 */
    static uint8_t k1_last = 1U;
    static uint8_t k2_last = 1U;
    static uint8_t k3_last = 1U;

    /* 读取当前按键电平，按键上拉，按下为低电平 */
    uint8_t k1_now = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
    uint8_t k2_now = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
    uint8_t k3_now = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);

    /* 从高电平变成低电平，说明按键刚刚被按下 */
    if (k1_last == 1U && k1_now == 0U) k1_event = 1U;
    if (k2_last == 1U && k2_now == 0U) k2_event = 1U;
    if (k3_last == 1U && k3_now == 0U) k3_event = 1U;

    /* 更新历史电平，供下一轮判断 */
    k1_last = k1_now;
    k2_last = k2_now;
    k3_last = k3_now;
}

static uint8_t TakeK1Press(void)
{
    uint8_t event = k1_event;
    k1_event = 0U;
    return event;
}

static uint8_t TakeK2Press(void)
{
    uint8_t event = k2_event;
    k2_event = 0U;
    return event;
}

static uint8_t TakeK3Press(void)
{
    uint8_t event = k3_event;
    k3_event = 0U;
    return event;
}

static void UpdatePreparedSide(void)
{
    /* 读取 K230 返回的目标左右位置 */
    uint8_t side = Vision_GetLoR();

    /* 只有识别到目标数字并且左右有效，才保存为准备转向信号 */
    if (Vision_TargetMatched() && (side == VISION_LOR_LEFT || side == VISION_LOR_RIGHT)) {
        car.prepared_side = side;
        car.prepared_side_timer = 0U;
    } else if (car.prepared_side_timer < 65535U) {
        /* 没有新信号时持续计时，超时后这个左右信号不再可靠 */
        car.prepared_side_timer++;
    }
}

static uint8_t HasPreparedSide(void)
{
    /* prepared_side 只在约 1 秒内有效，防止旧视觉结果误导后续路口 */
    return (car.prepared_side == VISION_LOR_LEFT || car.prepared_side == VISION_LOR_RIGHT) && car.prepared_side_timer <= PREPARED_SIDE_VALID_TICKS;
}

static void ClearPreparedSide(void)
{
    /* 清除 K230 左右准备信号，下一个路口需要重新识别 */
    car.prepared_side = VISION_LOR_NONE;
    car.prepared_side_timer = PREPARED_SIDE_VALID_TICKS + 1U;
}

static void DecideRouteAtIntersection(void)
{
    /* 保存本次路口使用的左右信号，随后立即清除，防止重复使用 */
    uint8_t lor = car.prepared_side;
    ClearPreparedSide();

    /* 模式一是近端病房，1 固定左转，2 固定右转 */
    if (car.mode == 1U) {
        if (car.pending_turn == TURN_LEFT) {
            car.route_code = 'A';
            StartTurn(TURN_LEFT);
        } else {
            car.route_code = 'B';
            StartTurn(TURN_RIGHT);
        }
        return;
    }

    /* 模式二在目标路口根据 K230 左右结果决定转向 */
    if (car.mode == 2U) {
        if (lor == VISION_LOR_LEFT) {
            car.route_code = 'M';
            StartTurn(TURN_LEFT);
        } else {
            car.route_code = 'N';
            StartTurn(TURN_RIGHT);
        }
        return;
    }

    /* 模式三第 3 个路口先记录远端第一次分岔方向 */
    if (car.intersection_count == 3U) {
        if (lor == VISION_LOR_LEFT) {
            car.route_code = 'C';
            car.far_first_side = VISION_LOR_LEFT;
            StartTurn(TURN_LEFT);
        } else {
            car.route_code = 'D';
            car.far_first_side = VISION_LOR_RIGHT;
            StartTurn(TURN_RIGHT);
        }
        return;
    }

    /* 模式三第 4 个路口结合第一次分岔和当前左右结果决定最终路线 */
    if (car.far_first_side == VISION_LOR_LEFT) {
        if (lor == VISION_LOR_LEFT) {
            car.route_code = 'E';
            StartTurn(TURN_LEFT);
        } else {
            car.route_code = 'F';
            StartTurn(TURN_RIGHT);
        }
    } else {
        if (lor == VISION_LOR_LEFT) {
            car.route_code = 'G';
            StartTurn(TURN_LEFT);
        } else {
            car.route_code = 'H';
            StartTurn(TURN_RIGHT);
        }
    }
}

static void SetupReturnTurns(void)
{
    uint8_t i;

    /* 返程从第 0 个转向开始执行 */
    car.return_turn_index = 0U;

    /* 返程转向次数默认等于去程记录次数 */
    car.return_turn_count = car.outbound_turn_count;

    /* 先清空返程转向表，避免残留旧任务数据 */
    for (i = 0U; i < MAX_ROUTE_TURNS; i++) {
        car.return_turns[i] = TURN_NONE;
    }

    /* 去程最后一个转向，返程第一个执行，并且左右方向相反 */
    for (i = 0U; i < car.outbound_turn_count; i++) {
        car.return_turns[i] = ReverseTurn(car.outbound_turns[car.outbound_turn_count - 1U - i]);
    }

    /* 如果异常情况下没有记录到去程转向，就用当前 pending_turn 兜底生成返程 */
    if (car.return_turn_count == 0U) {
        car.return_turn_count = 1U;
        car.return_turns[0] = ReverseTurn(car.pending_turn);
    }
}

void State_Machine_Init(void)
{
    /* 初始化速度环、巡线环和角度环 PID */
    PID_Speed_Init();
    PID_Line_Init();
    PID_Angle_Init();

    /* 默认进入模式一，等待 K230 识别目标数字 */
    car.mode = 1U;
    car.app_state = APP_IDLE;
    car.medicine = MEDICINE_EMPTY;
    car.start_confirmed = 0U;
    car.route_code = '-';

    /* 清空视觉左右准备信号和返程路径记录 */
    ClearPreparedSide();
    car.outbound_turn_count = 0U;
    car.return_turn_count = 0U;
    car.return_turn_index = 0U;

    /* 刷新装药和卸药指示灯 */
    SetMedicineIndicators();

    /* 通知 K230 进入任务 1：起点识别目标数字 */
    Vision_SetTask(1U, 0U);

    /* 状态机从空闲状态开始运行 */
    Current_State = State_Idle;
}

void State_Emergency_Stop(void)
{
    /* 紧急停止时立即清零目标速度 */
    PID_Speed_SetTarget(0, 0);

    /* 恢复到空闲任务状态 */
    car.app_state = APP_IDLE;
    car.medicine = MEDICINE_EMPTY;
    ClearPreparedSide();
    car.outbound_turn_count = 0U;
    car.return_turn_count = 0U;
    car.return_turn_index = 0U;
    SetMedicineIndicators();
    Current_State = State_Idle;
}

void State_Idle(void)
{
    /* 空闲状态下电机保持停止 */
    PID_Speed_SetTarget(0, 0);

    /* K2 用作启动确认键 */
    if (TakeK2Press()) {
        car.start_confirmed = 1U;
    }

    /* K1 在模式 1、2、3 之间循环切换 */
    if (TakeK1Press()) {
        car.mode++;
        if (car.mode > 3U) car.mode = 1U;
    }

    /* K3 模拟装药完成 */
    if (TakeK3Press()) {
        car.medicine = MEDICINE_LOADED;
        SetMedicineIndicators();
    }

    /* K230 起点识别到稳定数字后，保存目标房间号 */
    if (Vision_HasStableDigit()) {
        car.target_digit = Vision_GetStableDigit();
        car.target_intersection = car.mode;
        car.route_code = '-';
        car.intersection_count = 0U;
        car.far_first_side = VISION_LOR_NONE;

        /* 新任务开始前清空旧的视觉信号和返程路径 */
        ClearPreparedSide();
        car.outbound_turn_count = 0U;
        car.return_turn_count = 0U;
        car.return_turn_index = 0U;

        /* 模式一中 1 号房固定左转，2 号房固定右转 */
        car.pending_turn = (car.target_digit == 1U) ? TURN_LEFT : TURN_RIGHT;
        if (car.mode == 1U) {
            car.route_code = (car.pending_turn == TURN_LEFT) ? 'A' : 'B';
        }

        /* 通知 K230 进入任务 2：只寻找当前目标并返回左右位置 */
        Vision_SetTask(2U, car.target_digit);
        car.app_state = APP_WAIT_LOAD;
        Current_State = State_WaitLoad;
    } else {
        /* 还没识别到目标时，在 OLED 上显示等待目标或等待装药 */
        car.app_state = (car.medicine == MEDICINE_LOADED) ? APP_WAIT_LOAD : APP_WAIT_TARGET;
    }
}

static void State_WaitLoad(void)
{
    /* 等待装药和启动确认时，小车必须保持停止 */
    PID_Speed_SetTarget(0, 0);

    /* K2 可以在等待装药阶段再次确认启动 */
    if (TakeK2Press()) {
        car.start_confirmed = 1U;
    }

    /* K3 模拟药品已经放到车上 */
    if (TakeK3Press()) {
        car.medicine = MEDICINE_LOADED;
        SetMedicineIndicators();
    }

    /* 只有装药完成并且 K2 已确认，才允许进入去程巡线 */
    if (car.medicine == MEDICINE_LOADED && car.start_confirmed) {
        car.app_state = APP_OUTBOUND;
        ResetRuntimeCounters();
        Current_State = State_FollowMain;
    }
}

static void State_FollowMain(void)
{
    /* 去程巡线过程中持续接收 K230 左右准备信号 */
    UpdatePreparedSide();

    /* 灰度检测到路口后，先进入前冲状态，不立即转向 */
    if (FollowToIntersectionStep()) {
        fwd_timer = 0;
        Current_State = State_ForwardAfterIntersection;
    }
}

static void State_ForwardAfterIntersection(void)
{
    /* 路口处继续前进一点，直到车身中心越过路口 */
    if (!ForwardTicksStep()) {
        return;
    }

    /* 前冲结束后先停车，再判断是否需要转向 */
    fwd_timer = 0;
    PID_Speed_SetTarget(0, 0);

    /* 模式一第一个路口固定按目标 1/2 左右转 */
    if (car.mode == 1U && car.intersection_count >= 1U) {
        DecideRouteAtIntersection();
    } else if (car.mode == 3U && car.intersection_count == 3U && HasPreparedSide()) {
        /* 模式三第 3 个路口需要 K230 左右信号 */
        DecideRouteAtIntersection();
    } else if (car.mode == 3U && car.intersection_count == 4U && HasPreparedSide()) {
        /* 模式三第 4 个路口再次根据 K230 左右信号决策 */
        DecideRouteAtIntersection();
    } else if (car.mode == 2U && car.intersection_count >= car.target_intersection && HasPreparedSide()) {
        /* 模式二到达目标路口并且已有有效左右信号时才转向 */
        DecideRouteAtIntersection();
    } else {
        /* 当前路口不需要转向，继续沿主线前进 */
        Current_State = State_FollowMain;
    }
}

static void State_StopBeforeTurn(void)
{
    /* 转向前先停车等待，让车身稳定在路口中心 */
    PID_Speed_SetTarget(0, 0);
    if (wait_timer >= WAIT_TICKS) {
        wait_timer = 0;
        BeginStoppedTurn(turn_after_stop, State_Turn);
    } else {
        wait_timer++;
    }
}

static void State_Turn(void)
{
    /* 角度 PID 输出为差速，左右轮反向实现原地转向 */
    float turn_out = PID_Angle_GetTurn(current_yaw);
    PID_Speed_SetTarget(-turn_out, turn_out);

    /* 角度稳定后先停车等待，再开始扫线 */
    if (PID_Angle_IsSettled()) {
        wait_timer = 0;
        Current_State = State_WaitAfterTurn;
    }
}

static void State_WaitAfterTurn(void)
{
    /* 转向完成后短暂停车，减少惯性对扫线的影响 */
    PID_Speed_SetTarget(0, 0);
    if (++wait_timer >= WAIT_TICKS) {
        PrepareScan(1);
        Current_State = State_ScanLine;
    }
}

static void State_ScanLine(void)
{
    /* 左右小角度扫线，直到灰度误差接近中心 */
    if (ScanLineStep()) {
        ResetRuntimeCounters();

        /* 模式三第一次分岔后还没到终点，需要继续沿主线找下一个路口 */
        if (car.mode == 3U && (car.route_code == 'C' || car.route_code == 'D')) {
            Current_State = State_FollowMain;
        } else {
            Current_State = State_FollowToEnd;
        }
    }
}

static void State_FollowToEnd(void)
{
    /* 沿分支线巡线，直到连续检测不到红线认为到达病房终点 */
    if (FollowToEndStep()) {
        car.app_state = APP_WAIT_UNLOAD;
        PID_Speed_SetTarget(0, 0);
        Current_State = State_WaitUnload;
    }
}

static void State_WaitUnload(void)
{
    /* 等待卸药时小车保持停止 */
    PID_Speed_SetTarget(0, 0);

    /* K3 第二次按下表示药品已经取走，开始返程 */
    if (TakeK3Press()) {
        car.medicine = MEDICINE_REMOVED;
        SetMedicineIndicators();
        car.app_state = APP_RETURNING;

        /* 根据去程记录生成返程转向表 */
        SetupReturnTurns();

        /* 卸药后先原地 180° 掉头 */
        car.pending_turn = TURN_BACK;
        Helper_SetTurnTarget(180.0f);
        Current_State = State_ReturnTurn180;
    }
}

static void State_ReturnTurn180(void)
{
    /* 卸药后先原地掉头，车头对准返程方向 */
    float turn_out = PID_Angle_GetTurn(current_yaw);
    PID_Speed_SetTarget(-turn_out, turn_out);
    if (PID_Angle_IsSettled()) {
        wait_timer = 0;
        Current_State = State_ReturnWaitAfterTurn;
    }
}

static void State_ReturnFollowToIntersection(void)
{
    /* 返程沿线行驶，遇到路口后先前冲再决定是否转向 */
    if (FollowToIntersectionStep()) {
        fwd_timer = 0;
        Current_State = State_ReturnForwardAfterIntersection;
    }
}

static void State_ReturnForwardAfterIntersection(void)
{
    /* 返程路口同样需要前冲，让车身中心越过路口 */
    if (!ForwardTicksStep()) {
        return;
    }

    fwd_timer = 0;

    /* 返程转向表还没执行完，就取出下一次转向命令 */
    if (car.return_turn_index < car.return_turn_count) {
        turn_after_stop = car.return_turns[car.return_turn_index++];
        car.pending_turn = turn_after_stop;
        wait_timer = WAIT_TICKS;
        PID_Speed_SetTarget(0, 0);
        Current_State = State_ReturnStopBeforeTurn;
    } else {
        /* 所有返程转向执行完后，继续巡线回到起点终点 */
        ResetRuntimeCounters();
        Current_State = State_ReturnFollowToEnd;
    }
}

static void State_ReturnStopBeforeTurn(void)
{
    /* 返程转向前同样先停车等待 */
    PID_Speed_SetTarget(0, 0);
    if (wait_timer >= WAIT_TICKS) {
        wait_timer = 0;
        BeginStoppedTurn(turn_after_stop, State_ReturnTurn);
    } else {
        wait_timer++;
    }
}

static void State_ReturnTurn(void)
{
    /* 执行返程路口转向 */
    float turn_out = PID_Angle_GetTurn(current_yaw);
    PID_Speed_SetTarget(-turn_out, turn_out);
    if (PID_Angle_IsSettled()) {
        wait_timer = 0;
        Current_State = State_ReturnWaitAfterTurn;
    }
}

static void State_ReturnWaitAfterTurn(void)
{
    /* 返程转向后停车稳定，再开始扫线 */
    PID_Speed_SetTarget(0, 0);
    if (++wait_timer >= WAIT_TICKS) {
        if (car.pending_turn == TURN_BACK) {
            PrepareScan(1);
        } else {
            PrepareScan(-1);
        }
        Current_State = State_ReturnScanLine;
    }
}

static void State_ReturnScanLine(void)
{
    /* 返程扫到线后，根据是否还有转向决定下一步 */
    if (ScanLineStep()) {
        ResetRuntimeCounters();
        if (car.return_turn_index < car.return_turn_count) {
            Current_State = State_ReturnFollowToIntersection;
        } else {
            Current_State = State_ReturnFollowToEnd;
        }
    }
}

static void State_ReturnFollowToEnd(void)
{
    /* 回到起点终点后停止，并恢复到下一轮任务等待状态 */
    if (FollowToEndStep()) {
        PID_Speed_SetTarget(0, 0);
        car.app_state = APP_FINISHED;
        car.medicine = MEDICINE_EMPTY;
        SetMedicineIndicators();
        car.start_confirmed = 0U;
        Vision_ResetStableDigit();
        Vision_SetTask(1U, 0U);
        Current_State = State_Idle;
    }
}

static float debug_target_yaw = 0.0f;
static void State_Debug_Turn(void)
{
    /* 蓝牙角度调试状态，只执行原地转向，不参与送药流程 */
    float turn_out = PID_Angle_GetTurn(current_yaw);
    PID_Speed_SetTarget(-turn_out, turn_out);
    if (PID_Angle_IsSettled()) {
        Current_State = State_Idle;
    }
}

void Exec_Debug_Turn(float angle_offset)
{
    /* 如果已经在调试转向中，就忽略新的转向命令 */
    if (Current_State == State_Debug_Turn) return;

    /* 根据当前航向角和蓝牙给定偏移量计算目标角度 */
    debug_target_yaw = current_yaw + angle_offset;
    while (debug_target_yaw > 180.0f) debug_target_yaw -= 360.0f;
    while (debug_target_yaw < -180.0f) debug_target_yaw += 360.0f;

    /* 启动角度 PID，并切换到调试转向状态 */
    PID_Angle_StartTurn(debug_target_yaw);
    Current_State = State_Debug_Turn;
}

void State_Machine_Run(void)
{
    /* 先运行当前任务状态 */
    if (Current_State != 0) {
        Current_State();
    }

    /* 再统一执行速度闭环，让目标速度真正作用到电机 */
    PID_Speed_Execute();
}

uint8_t State_GetMode(void) { return car.mode; }
uint8_t State_GetTargetDigit(void) { return car.target_digit; }
uint8_t State_GetIntersectionCount(void) { return car.intersection_count; }
uint8_t State_GetMedicineState(void) { return car.medicine; }
uint8_t State_GetStartConfirmed(void) { return car.start_confirmed; }
uint8_t State_GetAppState(void) { return car.app_state; }
uint8_t State_IsIdle(void) { return Current_State == State_Idle; }
char State_GetRouteCode(void) { return car.route_code; }
