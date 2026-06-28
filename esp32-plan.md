# 陇参卫士 ESP32-S3 除草执行端开发计划

## 项目目标

ESP32-S3 作为除草杆执行控制器，通过 UART 接收 Jetson 下发的杂草目标坐标，
完成逆运动学解算与除草杆动作，并回报动作状态。

角色边界：

```text
Jetson    视觉、定位、决策，下发 arm_base 下的目标坐标
ESP32-S3  逆运动学、除草杆动作、状态反馈
```

底盘前进时机械臂不停车。底盘前进动力参与除草杆拔除动作。



项目情况：我需要A电机在控制B电机水平平面旋转，B电机控制除草杆在垂直于水平平面的平面的运动。

主板情况：ESP32S3

电机资料：K:\电机资料\YZ-AIM_2_58资料

Jertson Nano项目：C:\Users\Tika\Desktop\VMShare\Jetson_ws

---

# 一、硬件与连接

## 1. 主控

```text
ESP32-S3
```

## 2. 串口

```text
UART0（GPIO43 TXD / GPIO44 RXD）
115200 8N1
ASCII 十六进制行协议
经板载 COM 口（CH343P）出，Jetson 侧枚举 /dev/ttyUSB0
```

UART0 是 ESP32-S3 默认 console，要把它让给行协议：console 改走 USB-Serial-JTAG（USB 口，GPIO19/20），见 `sdkconfig.defaults`。改动后 build 端须删除已生成的 `sdkconfig` 让其按 defaults 重建，或 `idf.py menuconfig` 手动切到 USB-Serial-JTAG，否则旧 sdkconfig 仍把 console 钉在 UART0，log 文本会混进 AA55 帧。电机 RS485 走 UART2，与此无关。

## 3. 除草杆机构

```text
按 arm_base 目标点定位
垂直插入除草杆
保持
拔出
```

机构自由度、传动与电机参数由固件本地维护，本计划不约定。

---

# 二、坐标系

目标坐标在 `arm_base` 坐标系下，单位毫米。

```text
x：机器人前方
y：机器人左方
z：向下
```

Jetson 已把相机坐标转换到 `arm_base` 后再下发。ESP32 收到的就是 `arm_base` 下的目标点。

---

# 三、串口帧格式

帧结构：

```text
AA55 CMD ID X Y Z ERR SUM\n
```

字段：

```text
AA55  帧头，2 字节，固定 ASCII "AA55"
CMD   命令码，uint8，2 位大写十六进制
ID    目标 ID，uint16，4 位大写十六进制
X     x_mm，int16 补码，4 位大写十六进制
Y     y_mm，int16 补码，4 位大写十六进制
Z     z_mm，int16 补码，4 位大写十六进制
ERR   错误码，uint8，2 位大写十六进制
SUM   校验，uint8，2 位大写十六进制
\n    行尾
```

去掉行尾后每帧固定 26 个 ASCII 字符。Jetson 侧丢弃长度小于 26 或帧头不是 `AA55` 的行。ESP32 回帧同样按此格式与长度。

字段偏移（从 0 计，单位字符）：

```text
[0:4]   AA55
[4:6]   CMD
[6:10]  ID
[10:14] X
[14:18] Y
[18:22] Z
[22:24] ERR
[24:26] SUM
```

负数按 int16 补码取 4 位十六进制。例如 -18 写作 `FFEE`，235 写作 `00EB`。

---

# 四、命令与状态码

Jetson → ESP32：

```text
01 TARGET
02 HOME
03 STOP
04 RESET
05 PING
```

ESP32 → Jetson：

```text
81 ACCEPTED
82 BUSY
83 DONE
84 ERROR
85 READY
86 PONG
```

回帧的 `ID` 填对应目标 ID，`X Y Z` 可填 0，`ERR` 仅在 ERROR 帧填错误码。SUM 按实际字段计算。

Jetson 当前只按 `CMD` 映射状态，不解析回帧的 `X Y Z ERR`。错误码用于现场排查与日志，需要时再约定上位机解析。

---

# 五、校验算法

`SUM` 为 `CMD` 到 `ERR` 各字节求和后取低 8 位。求和按二进制字节，不是十六进制字符。

```c
uint8_t checksum(uint8_t cmd, uint16_t id,
                 int16_t x, int16_t y, int16_t z, uint8_t err) {
  uint32_t sum = cmd;
  sum += (id >> 8) & 0xFF;
  sum += id & 0xFF;
  uint16_t ux = (uint16_t)x, uy = (uint16_t)y, uz = (uint16_t)z;
  sum += (ux >> 8) & 0xFF; sum += ux & 0xFF;
  sum += (uy >> 8) & 0xFF; sum += uy & 0xFF;
  sum += (uz >> 8) & 0xFF; sum += uz & 0xFF;
  sum += err;
  return (uint8_t)(sum & 0xFF);
}
```

可复现示例：

```text
CMD=01 ID=002A X=00EB(235) Y=FFEE(-18) Z=0040(64) ERR=00
sum = 01 + 00 + 2A + 00 + EB + FF + EE + 00 + 40 + 00 = 0x343
SUM = 0x43
整帧 = AA5501002A00EBFFEE00400043\n
```

Jetson 侧 plan 第六节示例帧把 SUM 写成 `1C`，按本算法应为 `43`，`1C` 是文档占位值。固件以本累加和算法为准。上机联调时，两端任一方改算法都要同步另一方。

---

# 六、作业状态机

握手流程：

```text
收到 TARGET
  ↓ 回 ACCEPTED
逆运动学解算
  ↓ 回 BUSY
移动到 approach point
移动到 insert point
保持 hold_ms
移动到 retract point
  ↓ 回 DONE
```

约束：

```text
同一时刻只处理一个目标
BUSY 期间收到新 TARGET 不打断当前动作
TARGET 锁定后坐标不再变更
```

异常回帧：

```text
CMD = 84 ERROR
ID  = 当前目标 ID
ERR = 错误码
```

Jetson 第一版不重复发送同一个 TARGET。ERROR 后该目标在 Jetson 侧记为失败，不立即重试。

---

# 七、动作参数

参数保存在 ESP32 本地：

```text
approach_height_mm   接近高度
insert_depth_mm      插入深度
hold_ms              保持时长
retract_height_mm    拔出高度
move_speed           平移速度
insert_speed         插入速度
```

approach / insert / retract 三个点由目标点叠加上述参数算出。具体叠加方式按机构定义。

---

# 八、其它命令

```text
HOME   02   回到初始位姿
STOP   03   立即停止当前动作
RESET  04   清除错误，回到就绪
```

STOP 后若动作未完成，按 ERROR 或停在安全位姿，由固件定义并据实回报。

---

# 九、上电与保活

```text
上电完成自检后回 READY (85)
收到 PING (05) 立即回 PONG (86)
```

Jetson 周期发 PING，并在 `pong_timeout_ms`（默认 3000ms）内未收到任何回帧时判串口离线。
离线且机械臂处于 BUSY 是 Jetson 触发系统急停的条件之一。ESP32 必须及时回 PONG，保持回帧不中断。

PING 间隔 `ping_interval_ms` 默认 1000ms。回帧不限于 PONG，任何合法 `AA55` 帧都会刷新 Jetson 侧存活计时。

---

# 十、开发顺序

```text
1. 串口收发与行分帧
2. 帧解析、校验与字段解码
3. READY / PING-PONG 保活
4. TARGET 解析与 ACCEPTED 回报
5. 逆运动学解算
6. 除草杆动作与 BUSY / DONE 回报
7. 错误处理与 ERROR 回报
8. HOME / STOP / RESET
9. 与 Jetson serial_bridge 联调
10. 现场动作参数标定
```
