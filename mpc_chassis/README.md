# MPC 底盘控制

本工程面向 STM32H723，使用 CMSIS-DSP 进行矩阵运算，并使用 qpOASES 求解 MPC 的二次规划（QP）问题。

## Keil 配置

1. 在 Keil **Pack Installer** 搜索并安装 **CMSIS-DSP**。
2. 打开 **Manage Run-Time Environment**，勾选并应用 `CMSIS > DSP`。
3. 将 qpOASES 的 C++ 包装层和求解器源码加入工程，启用 C++ 编译。
4. qp默认用double，把__USE_SINGLE_PRECISION__注释取消可改用为float。跑到单片机上，为了节省开销，选择改用float

MPC 将未来控制输入作为优化变量，以状态误差和控制代价构造目标函数，由 qpOASES 求解，并限制控制量上下界。当前求解器不支持额外的一般线性约束。部署前请在目标板验证求解状态及运行时间。
