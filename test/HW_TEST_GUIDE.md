# SO-101 夹爪硬件测试指南

本文档说明如何在 SO-101 机器人上测试夹爪硬件功能。

## 硬件连接

1. **夹爪舵机**: Feetech STS3215，默认 ID = 6
2. **通信接口**: UART，默认 `/dev/ttyACM0`，波特率 1Mbaud
3. **供电**: 确保舵机已正确供电（7.4V 锂电池）

## 编译

### 编译参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `GRASP_BUILD_TESTS` | 是否编译测试程序 | `ON` |
| `GRASP_BUILD_HW_TEST` | 是否编译硬件测试程序 | `OFF` |
| `MOTOR_INCLUDE_PATH` | motor 库头文件路径 | 自动检测 |

### 编译步骤

```bash
cd components/control/grasp
mkdir -p build && cd build

# 配置 CMake（启用硬件测试）
cmake .. -DGRASP_BUILD_HW_TEST=ON

# 编译
make -j$(nproc)
```

**注意**: 如果 motor 库未编译，需要先编译 motor 库：
```bash
cd ../../peripherals/motor
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

编译成功后会生成：
- `libgrasp.a` - 夹爪控制库
- `test_grasp` - 单元测试（使用 dummy 驱动）
- `test_hw_so101_gripper` - 硬件测试程序

## 运行硬件测试

### 基本用法

```bash
./test_hw_so101_gripper
```

### 命令行参数

```bash
./test_hw_so101_gripper [选项]

选项:
  --port <path>  串口设备路径 (默认: /dev/ttyACM0)
  --baud <rate>  波特率 (默认: 1000000)
  --id <id>      夹爪舵机 ID (默认: 6)
  --help         显示帮助信息
```

### 示例

```bash
# 使用默认配置
./test_hw_so101_gripper

# 指定串口和舵机 ID
./test_hw_so101_gripper --port /dev/ttyUSB0 --id 6

# 修改波特率
./test_hw_so101_gripper --baud 115200
```

## 测试功能

程序提供交互式菜单，包含以下测试项：

### 1. 完全打开 (position = 1.0)
- 夹爪完全张开到最大开口
- 测试目的：验证夹爪能否正确打开

### 2. 完全闭合 (position = 0.0)
- 夹爪完全闭合
- 测试目的：验证夹爪能否正确闭合

### 3. 中间位置 (position = 0.5)
- 夹爪移动到中间位置（半开状态）
- 测试目的：验证精细位置控制

### 4. 手动输入位置
- 输入任意位置值 [0.0, 1.0]
- 测试目的：验证任意位置控制精度

### 5. 抓取测试 (GRAB 命令)
- 执行 GRAB 命令，夹爪闭合并检测物体
- 测试目的：验证抓取功能和负载检测
- 预期行为：
  - 如果夹住物体 → 状态变为 `HOLDING`
  - 如果夹空 → 状态变为 `EMPTY`

### 6. 释放测试 (RELEASE 命令)
- 执行 RELEASE 命令，夹爪完全打开
- 测试目的：验证释放功能

### 7. 放松测试 (RELAX 命令)
- 执行 RELAX 命令，舵机掉电
- 测试目的：验证省电模式，可手动移动夹爪

### 8. 查看当前状态和反馈
- 显示当前状态、位置和负载
- 测试目的：验证状态反馈功能

### 9. 校准夹爪（推荐首次使用）
- 交互式校准，学习实际硬件边界
- 测试目的：适应不同夹爪的机械差异
- 校准流程：
  1. 释放舵机扭矩
  2. 手动移动到完全打开位置，按回车
  3. 手动移动到完全闭合位置，按回车
  4. 自动验证并保存校准数据
- 校准数据保存到：`./config/so101_gripper_calibration.json`
- 下次启动自动加载校准数据

## 状态说明

| 状态 | 说明 |
|------|------|
| `IDLE` | 空闲，动作已完成 |
| `MOVING` | 运动中 |
| `HOLDING` | 夹住物体（负载超过阈值） |
| `EMPTY` | 夹空（完全闭合但无负载） |
| `ERROR` | 错误状态 |

## 位置映射

SO-101 夹爪支持两种位置映射模式：

### 1. 默认映射（未校准）

基于实际硬件测量的线性映射：

- **position = 0.0**: 完全闭合（对应 Feetech ticks = 2031）
- **position = 0.5**: 中间位置（对应 Feetech ticks ≈ 2750）
- **position = 1.0**: 完全打开（对应 Feetech ticks = 3468）

**实际硬件范围**: [2031, 3468] ticks，跨度 1437 ticks

**映射公式**:
```
ticks = 2031 + position × 1437
position = (ticks - 2031) / 1437
```

### 2. 校准映射（推荐）

通过测试程序的选项 9 进行交互式校准：

**校准流程**:
1. 选择菜单选项 9 - 校准夹爪
2. 确认继续校准
3. 程序释放舵机扭矩，允许手动移动
4. 手动将夹爪移动到完全打开位置，按回车
5. 程序记录 open_ticks
6. 手动将夹爪移动到完全闭合位置，按回车
7. 程序记录 closed_ticks
8. 自动验证范围有效性（范围 > 100 ticks，open > closed）
9. 保存到配置文件 `./config/so101_gripper_calibration.json`

**校准数据格式** (JSON):
```json
{
  "motor_id": 6,
  "open_ticks": 3468.0,
  "closed_ticks": 2031.0
}
```

**优势**:
- 适应不同夹爪的机械差异
- 自动学习实际可用范围
- 数据持久化，程序重启后自动加载
- 与 manipulator 配置文件在同一目录

**注意**: 
- 首次使用建议先进行校准
- 校准后的映射公式: `ticks = closed_ticks + position × (open_ticks - closed_ticks)`
- 如需重新校准，再次运行选项 9 即可覆盖旧数据

## 负载检测

- **hold_threshold**: 默认 100.0（Feetech 负载单位）
- 当负载超过阈值时，状态自动切换为 `HOLDING`
- 可通过配置调整阈值以适应不同物体

## 常见问题

### 1. 无法打开串口
```
错误: 无法创建夹爪实例
```
**解决方法**:
- 检查串口设备是否存在: `ls -l /dev/ttyACM*`
- 添加串口访问权限: `sudo usermod -aG dialout $USER`（需重新登录）
- 或临时使用 sudo: `sudo ./test_hw_so101_gripper`

### 2. 舵机无响应
**可能原因**:
- 舵机 ID 不匹配（使用 `--id` 参数指定正确 ID）
- 波特率不匹配（使用 `--baud` 参数）
- 舵机未上电或连接松动
- 串口被其他程序占用

### 3. 夹爪移动到错误位置
**可能原因**:
- 夹爪未校准，使用的是默认范围 [2031, 3468]
- 实际硬件范围与默认值不同

**解决方法**:
- 运行菜单选项 9 进行夹爪校准
- 校准后会自动保存到 `./config/so101_gripper_calibration.json`
- 重启程序会自动加载校准数据

### 4. 负载检测不准确
**解决方法**:
- 调整 `hold_threshold` 参数
- 检查夹爪扭矩保护参数（Max_Torque_Limit, Protection_Current）

## 扭矩保护参数

夹爪使用以下保护参数（在驱动中自动配置）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `Max_Torque_Limit` | 500 | 最大扭矩限制（约 50%） |
| `Protection_Current` | 250 mA | 过流保护阈值 |
| `Overload_Torque` | 25 | 过载扭矩检测阈值 |

这些参数防止夹爪过载损坏，同时保证足够的抓取力。

## 调试技巧

1. **查看实时状态**: 使用菜单选项 8 查看当前位置和负载
2. **监控动作过程**: 程序会实时显示状态转换和位置变化
3. **测试序列**: 建议按顺序测试：打开 → 闭合 → 中间位置 → 抓取
4. **负载测试**: 在夹爪中放置不同重量的物体，观察负载值变化

## 与 manipulator 的集成

- **独立控制**: grasp 和 manipulator 是独立的模块
- **协同工作**: 应用层可同时调用两个模块的 API
- **示例流程**:
  1. 使用 manipulator 移动机械臂到目标位置
  2. 使用 grasp 控制夹爪抓取物体
  3. 使用 manipulator 移动到放置位置
  4. 使用 grasp 释放物体

## 参考

- API 文档: `include/grasp.h`
- 驱动实现: `src/drivers/drv_uart_so101_gripper.c`
- 舵机寄存器: `../manipulator/include/sts3215_regs.h`
