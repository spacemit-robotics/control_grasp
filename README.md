# Grasp / End-Effector Component

## 项目简介

通用末端执行器控制库 — 统一 C API，支持多种硬件（抓夹、吸盘等）通过驱动插件接入。

## 代码结构

```
grasp/
├── include/
│   └── grasp.h                          # 公共 API 接口
├── src/
│   ├── grasp.c                          # 核心实现（驱动注册、设备管理）
│   ├── grasp_core.h                     # 内部头文件（驱动接口定义）
│   └── drivers/
│       ├── drv_dummy.c                  # Dummy 驱动（测试/占位）
│       └── drv_uart_so101_gripper.c     # SO-101 夹爪驱动（Feetech 舵机）
├── third_party/
│   └── motor/                           # 本地 motor 依赖源码（优先参与构建）
│       ├── include/                     # motor 公共头文件
│       └── src/                         # 电机驱动与适配实现
├── test/
│   ├── test_grasp.c                     # 单元测试（dummy 驱动）
│   ├── test_hw_so101_gripper.c          # 硬件测试程序
│   └── HW_TEST_GUIDE.md                 # 硬件测试指南
├── CMakeLists.txt                       # 构建配置
├── package.xml                          # 依赖声明
├── LICENSE                              # Apache-2.0 许可证
├── NOTICE                               # 第三方归属声明
└── README.md                            # 本文档
```

## 功能特性

- 驱动插件架构：自动注册，无需修改核心代码即可扩展新硬件
- 状态机管理：IDLE/MOVING/HOLDING/EMPTY/ERROR 自动流转
- 精细位置控制：支持 [0.0~1.0] 归一化开合度
- 交互式校准：学习实际硬件边界，适配不同夹爪
- 线程安全状态管理

### 已支持硬件

| 驱动            | 文件                                   | 说明                                      |
|-----------------|----------------------------------------|-------------------------------------------|
| `dummy`         | `src/drivers/drv_dummy.c`              | 测试驱动，模拟状态转换，无硬件依赖        |
| `so101_gripper` | `src/drivers/drv_uart_so101_gripper.c` | SO-101 夹爪，Feetech STS3215 舵机（ID 6） |

## 快速开始

### 环境准备

| 依赖     | 必需/可选 | 安装方式                       |
|----------|-----------|--------------------------------|
| pthreads | 必需      | 系统自带                       |
| libm     | 必需      | 系统自带                       |
| motor    | 必需      | `components/peripherals/motor`          |

### 构建编译

```bash
mkdir build && cd build
cmake ..
make
```

完整编译（SO-101 夹爪硬件测试）：

```bash
mkdir build && cd build
cmake .. -DGRASP_BUILD_HW_TEST=ON
make -j$(nproc)
```

CMake 选项：

| 选项                  | 默认 | 说明                         |
|-----------------------|------|------------------------------|
| `GRASP_BUILD_TESTS`   | ON   | 单元测试                     |
| `GRASP_BUILD_HW_TEST` | OFF  | SO-101 夹爪硬件交互测试      |

### 运行示例

```c
#include "grasp.h"

struct grasp_dev *gripper = grasp_alloc("so101_gripper", &config);

grasp_execute(gripper, GRASP_CMD_GRAB, 0.8f);

grasp_state_t state = grasp_get_state(gripper);
if (state == GRASP_STATE_HOLDING) {
    printf("Object grasped!\n");
}

grasp_execute(gripper, GRASP_CMD_RELEASE, 0.0f);

grasp_stop(gripper);
grasp_free(gripper);
```

## 详细使用

### 公共 API (`grasp.h`)

#### 设备生命周期

```c
struct grasp_dev *grasp_alloc(const char *driver_name, void *args);
void grasp_free(struct grasp_dev *dev);
```

#### 运动控制

```c
int grasp_execute(struct grasp_dev *dev, grasp_cmd_type_t type, float effort);
int grasp_set_position(struct grasp_dev *dev, float position);
void grasp_stop(struct grasp_dev *dev);
```

#### 状态查询

```c
grasp_state_t grasp_get_state(struct grasp_dev *dev);
int grasp_get_feedback(struct grasp_dev *dev, float *out_pos, float *out_load);
void grasp_tick(struct grasp_dev *dev, float dt_s);
```

#### 校准

```c
int grasp_calibrate(struct grasp_dev *dev);
```

> 交互式校准流程，学习实际硬件边界。调用后需手动移动夹爪到完全打开和完全闭合位置。

#### 错误码

| 宏                  | 值   | 含义         |
|---------------------|------|--------------|
| `GRASP_OK`          |  0   | 成功         |
| `GRASP_ERR_ALLOC`   | -1   | 内存分配失败 |
| `GRASP_ERR_CONNECT` | -2   | 通信失败     |
| `GRASP_ERR_TIMEOUT` | -3   | 超时         |
| `GRASP_ERR_CONFIG`  | -4   | 配置错误     |
| `GRASP_ERR_PARAM`   | -5   | 参数错误     |
| `GRASP_ERR_NOSYS`   | -6   | 功能未实现   |

### SO-101 夹爪驱动

SO-101 夹爪使用单个 Feetech STS3215 舵机（默认 ID 6）实现开合控制。详细的硬件测试说明请参考 `test/HW_TEST_GUIDE.md`。

当 `motor` 组件可用时，CMake 会自动启用 Feetech 驱动（`-DHAVE_MOTOR`）。

配置结构（`struct so101_gripper_config`）：
```c
struct so101_gripper_config {
    const char *uart_path;         // 串口设备，如 "/dev/ttyACM0"
    uint32_t baud;                 // 波特率，如 1000000
    uint8_t id;                    // 夹爪电机 ID，默认 6
    grasp_config_t grasp_cfg;      // 可选配置覆盖
};
```

通过 `grasp_alloc("so101_gripper", &config)` 创建实例。

### 扩展新驱动

1. 在 `src/drivers/` 中新增 `.c` 文件
2. 包含 `"../grasp_core.h"`
3. 实现 `struct grasp_ops` 操作表和工厂函数
4. 使用 `REGISTER_GRASP_DRIVER("your_name", your_factory)` 注册
5. 在 `CMakeLists.txt` 中添加源文件
6. 用户通过 `grasp_alloc("your_name", args)` 选用

## 常见问题

**Q: 编译时找不到 motor 库？**
确保 motor 组件已编译，或在 CMake 时指定 `MOTOR_INCLUDE_PATH` 和 `MOTOR_LIB_PATH`。

**Q: SO-101 夹爪连接失败？**
检查串口权限（`sudo chmod 666 /dev/ttyACM0`）和波特率（默认 1000000）。

**Q: 校准后位置不准？**
确认校准时手动将夹爪移到了**完全打开**和**完全闭合**位置，确认 `config/so101_gripper_calibration.json` 文件正确保存。

## 版本与发布

版本以本组件文档或仓库 tag 为准。

| 版本  | 说明                                           |
|-------|------------------------------------------------|
| 1.0.0 | 初始版本，支持 SO-101 夹爪驱动，交互式校准功能 |

## 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

1. C/C++ 代码遵循 [Google C++ 风格](https://google.github.io/styleguide/cppguide.html)
2. Python 代码遵循 [PEP 8](https://peps.python.org/pep-0008/)
3. Git commit 遵循 [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
