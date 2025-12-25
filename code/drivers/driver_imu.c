/* yis_imu.c */
#include <driver_imu.h>

#define LED1                    (P20_9)

/* ================= UART 配置 ================= */
#define YIS_UART_INDEX          UART_5
#define YIS_BAUDRATE            115200
#define YIS_TX_PIN              UART5_TX_P22_2
#define YIS_RX_PIN              UART5_RX_P22_3

/* ================= 协议定义 ================= */
#define YIS_HEADER_1            0x59
#define YIS_HEADER_2            0x53

/* ================= 长度合法性 ================= */
#define YIS_MAX_PAYLOAD_LEN     (128u)  /* 必须 <= data_buffer 大小 */
#define YIS_MIN_PAYLOAD_LEN     (1u)    /* 可按协议调整：0/1/2... */

/* ================= 解析状态 ================= */
typedef enum
{
    STATE_IDLE = 0,
    STATE_HEADER1,
    STATE_HEADER2,
    STATE_SKIP_ID1,
    STATE_SKIP_ID2,
    STATE_GET_LEN,
    STATE_DATA,
    STATE_SKIP_CK1,
    STATE_SKIP_CK2,

    /* 丢帧：吃掉剩余字节直到到达本帧末尾，再复位 */
    STATE_DROP_FRAME

} yis_parse_state_t;

/* ================= 全局变量 ================= */
yis_imu_t yis_imu = {0};

static uint8  data_buffer[YIS_MAX_PAYLOAD_LEN];
static uint8  frame_len = 0;       /* payload 长度 */
static uint8  data_index = 0;      /* payload 已接收字节数 */
static yis_parse_state_t parser_state = STATE_IDLE;

static uint8  drop_remaining = 0;  /* 丢帧时还需要吞掉多少 payload 字节（不含校验） */
static uint8  drop_ck_remaining = 0; /* 丢帧时还需要吞掉多少校验字节（一般 2） */

static inline int yis_len_is_valid(uint8 len)
{
    if (len < YIS_MIN_PAYLOAD_LEN) return 0;
    if (len > YIS_MAX_PAYLOAD_LEN) return 0;
    return 1;
}

/* ================= 帧解析 ================= */
static void yis_parse_frame(void)
{
    uint8 pos = 0;

    while (pos < frame_len)
    {
        /* 至少要有 data_id + sub_len 两个字节 */
        if ((uint8)(frame_len - pos) < 2u) break;

        uint8 data_id = data_buffer[pos++];
        uint8 sub_len = data_buffer[pos++];

        /* 子段长度越界：直接结束解析（丢弃本帧剩余部分） */
        if ((uint8)(frame_len - pos) < sub_len) break;

        /* -------- 角速度 (0x20) -------- */
        if (data_id == 0x20 && sub_len >= 12u)
        {
            int32 raw;

            /* 小端 int32 -> float */
            raw = (int32)((uint32)data_buffer[pos+3] << 24 |
                          (uint32)data_buffer[pos+2] << 16 |
                          (uint32)data_buffer[pos+1] << 8  |
                          (uint32)data_buffer[pos+0]);
            yis_imu.wx = raw * 0.000001f;
            pos += 4;

            raw = (int32)((uint32)data_buffer[pos+3] << 24 |
                          (uint32)data_buffer[pos+2] << 16 |
                          (uint32)data_buffer[pos+1] << 8  |
                          (uint32)data_buffer[pos+0]);
            yis_imu.wy = raw * 0.000001f;
            pos += 4;

            raw = (int32)((uint32)data_buffer[pos+3] << 24 |
                          (uint32)data_buffer[pos+2] << 16 |
                          (uint32)data_buffer[pos+1] << 8  |
                          (uint32)data_buffer[pos+0]);
            yis_imu.wz = raw * 0.000001f;
            pos += 4;

            /* 如果 sub_len > 12，跳过多余字节 */
            if (sub_len > 12u) pos += (uint8)(sub_len - 12u);
        }
        /* -------- 欧拉角 (0x40) -------- */
        else if (data_id == 0x40 && sub_len >= 12u)
        {
            int32 raw;

            raw = (int32)((uint32)data_buffer[pos+3] << 24 |
                          (uint32)data_buffer[pos+2] << 16 |
                          (uint32)data_buffer[pos+1] << 8  |
                          (uint32)data_buffer[pos+0]);
            yis_imu.pitch = raw * 0.000001f;
            pos += 4;

            raw = (int32)((uint32)data_buffer[pos+3] << 24 |
                          (uint32)data_buffer[pos+2] << 16 |
                          (uint32)data_buffer[pos+1] << 8  |
                          (uint32)data_buffer[pos+0]);
            yis_imu.roll = raw * 0.000001f;
            pos += 4;

            raw = (int32)((uint32)data_buffer[pos+3] << 24 |
                          (uint32)data_buffer[pos+2] << 16 |
                          (uint32)data_buffer[pos+1] << 8  |
                          (uint32)data_buffer[pos+0]);
            yis_imu.yaw = raw * 0.000001f;
            pos += 4;

            if (sub_len > 12u) pos += (uint8)(sub_len - 12u);
        }
        else
        {
            /* 其它字段直接跳过 */
            pos += sub_len;
        }
    }
}

/* ================= 字节解析状态机 ================= */
static void yis_parse_byte(uint8 data)
{
    switch (parser_state)
    {
        case STATE_IDLE:
            if (data == YIS_HEADER_1) parser_state = STATE_HEADER1;
            break;

        case STATE_HEADER1:
            parser_state = (data == YIS_HEADER_2) ? STATE_HEADER2 : STATE_IDLE;
            break;

        case STATE_HEADER2:
            parser_state = STATE_SKIP_ID1;
            break;

        case STATE_SKIP_ID1:
            parser_state = STATE_SKIP_ID2;
            break;

        case STATE_SKIP_ID2:
            /* 这里按你原逻辑：此字节就是 LEN */
            frame_len  = data;
            data_index = 0;

            /* ===== 长度合法性检查 ===== */
            if (!yis_len_is_valid(frame_len))
            {
                /* 丢帧：吞掉 payload(frame_len) + CK1 + CK2 */
                drop_remaining    = frame_len; /* 可能为 0 或超大，但 uint8 会截断；此处按协议 len 是 1B */
                drop_ck_remaining = 2u;
                parser_state      = STATE_DROP_FRAME;
                break;
            }

            parser_state = (frame_len > 0u) ? STATE_DATA : STATE_SKIP_CK1;
            break;

        case STATE_DATA:
            /* frame_len 已保证 <= buffer */
            data_buffer[data_index++] = data;

            if (data_index >= frame_len)
                parser_state = STATE_SKIP_CK1;
            break;

        case STATE_SKIP_CK1:
            parser_state = STATE_SKIP_CK2;
            break;

        case STATE_SKIP_CK2:
            /* 正常完成一帧，解析 payload */
            yis_parse_frame();
            parser_state = STATE_IDLE;
            break;

        case STATE_DROP_FRAME:
            /* 吞掉剩余 payload 字节 */
            if (drop_remaining > 0u)
            {
                drop_remaining--;
            }
            else if (drop_ck_remaining > 0u)
            {
                /* payload 吃完后再吃校验字节（CK1/CK2） */
                drop_ck_remaining--;
            }
            else
            {
                /* 丢帧结束，复位 */
                parser_state = STATE_IDLE;
            }
            break;

        default:
            parser_state = STATE_IDLE;
            break;
    }
}

/* ================= UART 接收中断回调 ================= */
void yis_uart_rx_handler(void)
{
    uint8 data;
    while (uart_query_byte(YIS_UART_INDEX, &data))
    {
        yis_parse_byte(data);
    }
}

/* ================= 初始化 ================= */
void yis_init(void)
{
    uart_init(YIS_UART_INDEX, YIS_BAUDRATE, YIS_TX_PIN, YIS_RX_PIN);
    uart_rx_interrupt(YIS_UART_INDEX, 1);

    parser_state = STATE_IDLE;
    frame_len = 0;
    data_index = 0;
    drop_remaining = 0;
    drop_ck_remaining = 0;
}


