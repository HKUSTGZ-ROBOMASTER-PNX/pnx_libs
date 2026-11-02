# libs

C++17 头文件库。父工程先添加 `boards`，再添加 `libs`，应用链接 `libs` target。
板型、工具链和 ThreadX 配置由 boards 管理；libs 通过 `stm32cubemx` 获取依赖。

```cmake
add_subdirectory(boards)
add_subdirectory(libs)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE libs)
```

## 文件职能

| 文件 | 主要职能 |
| --- | --- |
| `types/include/status.hpp` | 通用状态码，如 ok、busy、invalid_arg |
| `types/include/callback.hpp` | 不持有对象的函数/成员函数回调，无动态分配 |
| `msg/include/msg.hpp` | 基于 ThreadX mutex 的最新值发布订阅 |
| `crc/include/crc.hpp` | CRC8、CRC16、Modbus CRC16 的计算、校验和追加 |
| `math/include/constants.hpp` | 数学常量、角度转换、平方根倒数 |
| `math/include/constrains.hpp` | 限幅、`wrap(input, min, max)` 周期回绕、绝对值限幅 |
| `math/include/geometry.hpp` | 三维向量、四元数运算与欧拉角转换 |
| `math/include/quantize.hpp` | `dequantize(value, min, max, bits)` 协议整数反量化；调用方保证位宽 1～31、数值在范围内且有限的 `min < max` |
| `filters/include/biquad.hpp` | 二阶滤波系数、状态、更新和复位 |
| `filters/include/iir.hpp` | 二阶 Butterworth 低通/高通与陷波器系数设计 |
| `filters/include/kalman_1d.hpp` | 当前为空，尚未实现 |
| `control/include/pid.hpp` | 位置式 PID、外部微分输入及可组合的积分/微分选项 |
| `control/include/smc.hpp` | 指数、幂次、终端、速度积分和误差积分滑模控制器 |

## pid

```cpp
#include "pid.hpp"

control::pid controller(1.0f, 0.01f, 0.1f, 100.0f, 20.0f,
                        control::pid_mode::position |
                        control::pid_mode::trapezoid_integral);

float step(float reference, float feedback)
{
    controller.ref = reference;
    controller.fdb = feedback;
    controller.update();
    return controller.result;
}
```

实现全部位于头文件，链接 `libs` 即可使用，无需添加 PID 源文件。
固定周期调用 `update()`；`ki` 已包含采样周期，`kd` 已包含采样周期的倒数。
输入和参数须为有限值，`max_out`、`max_iout`、`scalar_a`、`scalar_b` 须非负。
同一实例的读写由调用方串行化。

基础模式选择 `position` 或 `delta` 之一，再用 `|` 组合选项。
`delta` 沿用外部微分输入语义：`d = kd * update(velocity)`，并非增量式 PID。
调用方负责速度的单位、符号及采样周期换算；若使用反馈速度实现微分先行，通常传入负值。
`derivative_on_measurement` 会覆盖基础模式的微分项，使用 `kd * (上次反馈 - 当前反馈)`。

积分选项保留原算法顺序：梯形积分、积分分离、变速积分、积分限幅。
积分分离沿用 `max_out` 作为误差阈值；分离和变速仅在误差与已有积分同号时生效。
变速积分在 `scalar_b` 内完整积分，随后在宽度 `scalar_a` 内线性降为零；
`scalar_a = 0` 时直接切换，不执行除法。输出和积分始终分别受两个限幅参数约束。

`tune()` 修改增益并保留状态。`clear()` 清零输入和运行状态，保留增益、模式及限幅/变速参数。
`reset_state(reference, feedback)` 清零累计输出并用给定输入初始化历史，
避免相同输入下下一次更新出现由空历史导致的差分微分突跳。

## smc

```cpp
#include "smc.hpp"

control::smc_config config;
config.mode = control::smc_mode::exponential;
config.inertia = 1.0f;
config.reaching_gain = 10.0f;
config.surface_gain = 5.0f;
config.switching_gain = 1.0f;
config.boundary_layer = 0.1f;
config.output_limit = 100.0f;

control::smc controller(config);

float step(float target, float position, float velocity, float dt)
{
    return controller.update_position(target, position, velocity, dt);
}
```

SMC 同样完全实现在头文件中。位置模式调用 `update_position()`，速度模式调用
`update_velocity()`；`dt` 单位为秒，每次调用直接完成目标差分、误差积分、滑模面和输出计算。
目标与位置单位相同，速度为该单位每秒，输出单位由模型和增益共同决定。误差符号沿用迁移来源，
为“反馈减目标”。首次有效更新将目标一、二阶导置零，再正常计算本周期输出，避免非零初值产生
差分冲击。

`configure()` 要求 `inertia`、`output_limit`、`saturation_limit` 为正，其他增益和限幅非负；
幂次模式要求
`0 < power_exponent < 1`，终端模式要求正奇数 `p > q`、正 `terminal_gain` 和正
`position_deadband`。失败时保留原配置。
重新配置会清空动态状态，`reset()` 清状态但保留配置，`reset_integrals()` 只清两个积分项。
输入、`dt` 或计算结果非法时输出归零，并令 `state().valid == false`。同一实例由调用方串行访问。

| 模式 | 更新入口 | 主要专用参数 |
| --- | --- | --- |
| `exponential` | `update_position()` | `surface_gain`、`switching_gain` |
| `power` | `update_position()` | `surface_gain`、`power_exponent`；不使用 `switching_gain` |
| `terminal` | `update_position()` | `terminal_gain`、`terminal_p`、`terminal_q`、`switching_gain` |
| `velocity` | `update_velocity()` | `surface_gain`、`switching_gain`、`velocity_integral_limit` |
| `error_integral` | `update_position()` | `surface_gain`、`integral_gain`、`switching_gain` |

`boundary_layer > 0` 时，饱和函数将 `s / boundary_layer` 连续限幅到
`[-saturation_limit, saturation_limit]`；为零时使用同幅值的符号函数。位置静止判定要求位置和
速度误差分别进入 `position_deadband`、`velocity_deadband`，死区内不累积位置误差。终端模式还
使用 `position_deadband` 正则化零点附近的奇异项；目标加速度非零时保留前馈输出。指数、幂次和
误差积分模式保留来源公式，
不加入目标加速度前馈，适用于静态或足够平滑的目标；终端模式使用目标二阶导，速度模式使用目标
速度的一阶导。

原实现的 `epsilon` 同时承担多个含义。迁移旧参数时，指数、终端、速度积分和误差积分模式
应将它同时填入 `switching_gain` 与 `boundary_layer`；幂次模式应填入 `boundary_layer` 和
`power_exponent`。原 `u_max`、`pos_esp` 分别对应 `output_limit`、`position_deadband`；原
`limit = 1` 对应 `saturation_limit = 1`。原算法在 `limit != 1` 时会在边界跳变，新实现将该值
解释为连续限幅幅值，不能直接复现旧行为，需重新验证参数。`velocity_deadband` 需按实际速度单位
另行设置。位置和速度积分默认分别限幅到 2000，可在配置中调整。

宿主机回归测试使用独立 CMake 工程：

```sh
cmake -S libs/control/tests -B build/smc_tests -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/smc_tests
ctest --test-dir build/smc_tests --output-on-failure
```

测试覆盖五种控制律、参数校验、输出限幅、死区、终端零点和无效采样恢复；实际对象参数和控制
效果仍需在对应机构上验证。

## geometry

`math::multiply(quaternion, quaternion)` computes the Hamilton product in W, X, Y, Z order.
The existing `math::multiply(quaternion, vector3)` treats the vector as a pure quaternion;
it does not rotate the vector.

## callback

```cpp
#include "callback.hpp"

int increment(int value) { return value + 1; }

struct receiver
{
    int receive(int value) const { return value * 2; }
};

void example()
{
    using callback = pnxtypes::callback<int(int)>;
    auto function = callback::bind<increment>();
    receiver object;
    auto method = callback::bind<&receiver::receive>(&object);

    const int a = function(3); // 4
    const int b = method(3);   // 6
    (void)a;
    (void)b;
}
```

默认构造的回调为空，调用前用 `if (callback)` 或 `valid()` 检查。
成员函数绑定不拥有对象，对象必须比回调的使用期更长。
`==` / `!=` 比较绑定的上下文和调用入口。

## msg

```cpp
#include "msg.hpp"

struct sample { float value; };
msg::topic<sample> samples; // 生命周期覆盖所有订阅者

// 在允许创建 ThreadX mutex 的初始化阶段调用；发布和读取用于线程上下文。
void example()
{
    if (samples.init() != pnxtypes::status::ok)
        return;

    auto sub = samples.subscribe();
    if (sub)
    {
        if (samples.publish(sample{1.0f}) == pnxtypes::status::ok)
        {
            sample output{};
            if (sub.read(output) == pnxtypes::status::ok)
            {
                // 使用 output.value
            }
        }
    }

    // 所有发布者、订阅者停止使用后，显式释放 mutex。
    (void)samples.deinit();
}
```

消息类型必须可平凡复制；每个 topic 最多 8 个订阅槽，当前不支持退订。
连续发布会覆盖旧值，各订阅者只读取最新值，成功读取后清除自己的待读标记。
可用 `available()` 查询是否有新值，但 `read()` 的返回状态才是最终结果。
锁被占用或没有待读消息时返回 `busy`，不阻塞等待；这些接口不用于 ISR。
销毁 topic 前需显式 `deinit()`，重新初始化后应重新订阅。

`math::wrap(input, min, max)` 使用原循环限幅实现，将输入回绕到半开区间 `[min, max)`。
例如弧度使用 `wrap(angle, -math::pi, math::pi)`，角度使用 `wrap(angle, -180.0f, 180.0f)`。
上下界相等时返回该边界，上界小于下界时返回原输入。
