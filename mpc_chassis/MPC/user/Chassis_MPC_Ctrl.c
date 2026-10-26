#include "Chassis_MPC_Ctrl.h"
#include <stdlib.h>
#include <string.h>

// 初始化 mpc 结构体，这里直接把 Q 和 R 设好
static mpc_t mpc = 
{
    .Q = {100.0f, 100.0f, 50.0f, 1.0f, 1.0f, 5.0f}, // 状态权重 [x, y, yaw, vx, vy, w]
    .R = {1.0f, 1.0f, 10.0f},                       // 控制权重 [Fx, Fy, Mz]
    .RobotParam = 
    {
        .mass = 10.0f,      
        .inertia_z = 0.5f,  
        .max_force_x = 50.0f,
        .max_force_y = 50.0f,
        .max_torque_z = 10.0f
    },
};

// ==========================================
// 全局静态变量定义
// ==========================================
static float32_t H_data[NV * NV]; 
static arm_matrix_instance_f32 H_mat;

static float32_t A_data[NX * NX]; 
static float32_t B_bar_data[(NX * MPC_MAX_N) * (NU * MPC_MAX_N)];
static float32_t B_bar_T_data[(NU * MPC_MAX_N) * (NX * MPC_MAX_N)];
static float32_t BtB_data[NV * NV];
static float32_t QB_bar_data[(NX * MPC_MAX_N) * (NU * MPC_MAX_N)]; 

// 新增：控制量上下界
static float32_t lb[NV];
static float32_t ub[NV];


//提前计算好H矩阵和约束边界
void MPC_Init(void)
{
    // ==========================================
    // 1. 初始化 H 矩阵描述符
    // ==========================================
    arm_mat_init_f32(&H_mat, NV, NV, H_data);

    // ==========================================
    // 2. 生成单步 A 和 B 矩阵 (代码保持不变)
    // ==========================================
    float32_t B_data[NX * NU] = {0};
    arm_matrix_instance_f32 A_mat, B_mat;
    
    memset(A_data, 0, sizeof(A_data));
    for (int i = 0; i < NX; i++) {
        A_data[i * NX + i] = 1.0f;
    }
    for (int i = 0; i < NX / 2; i++) {
        A_data[i * NX + (i + NX / 2)] = mpc_dt; 
    }
    arm_mat_init_f32(&A_mat, NX, NX, A_data);

    float m_val = (mpc.RobotParam.mass > 0.001f) ? mpc.RobotParam.mass : 1.0f;
    float Iz_val = (mpc.RobotParam.inertia_z > 0.001f) ? mpc.RobotParam.inertia_z : 1.0f;
    B_data[3*NU + 0] = mpc_dt / m_val;
    B_data[4*NU + 1] = mpc_dt / m_val;
    B_data[5*NU + 2] = mpc_dt / Iz_val;
    arm_mat_init_f32(&B_mat, NX, NU, B_data);

    // ==========================================
    // 3. 构建 B_bar (代码保持不变)
    // ==========================================
    memset(B_bar_data, 0, sizeof(B_bar_data)); 
    arm_matrix_instance_f32 B_bar_mat;
    arm_mat_init_f32(&B_bar_mat, NX * MPC_MAX_N, NU * MPC_MAX_N, B_bar_data);

    float32_t temp_curr[NX * NU] = {0};
    float32_t temp_next[NX * NU] = {0};
    arm_matrix_instance_f32 temp_curr_mat, temp_next_mat;
    arm_mat_init_f32(&temp_curr_mat, NX, NU, temp_curr);
    arm_mat_init_f32(&temp_next_mat, NX, NU, temp_next);

    for (uint16_t j = 0; j < MPC_MAX_N; j++) 
    {
        memcpy(temp_curr, B_data, NX * NU * sizeof(float32_t));
        for (uint16_t i = j; i < MPC_MAX_N; i++) 
        {
            for (uint16_t r = 0; r < NX; r++) {
                for (uint16_t c = 0; c < NU; c++) {
                    B_bar_data[(i * NX + r) * (NU * MPC_MAX_N) + (j * NU + c)] = temp_curr[r * NU + c];
                }
            }
            if (i < MPC_MAX_N - 1) {
                arm_mat_mult_f32(&A_mat, &temp_curr_mat, &temp_next_mat);
                memcpy(temp_curr, temp_next, NX * NU * sizeof(float32_t));
            }
        }
    }

    // ==========================================
    // 4. 核心修改：原地算 H (不再需要其他大数组)
    // ==========================================
    uint16_t rows = NX * MPC_MAX_N;  // B_bar 行数 (180)
    uint16_t cols = NU * MPC_MAX_N;  // B_bar 列数 / H 维度 (90)

    // 4.1 把 Q 直接乘在 B_bar 的行上
    for (uint32_t r = 0; r < rows; r++) {
        uint8_t state_idx = r % NX; 
        float q_val = mpc.Q[state_idx];
        for (uint16_t c = 0; c < cols; c++) {
            B_bar_data[r * cols + c] *= q_val; // 原地修改
        }
    }

    // 4.2 手工计算 H = 2 * (B_bar^T * B_bar) + 2 * R
    for (uint16_t i = 0; i < cols; i++) {       // H 的行
        for (uint16_t j = 0; j < cols; j++) {   // H 的列
            float32_t sum = 0.0f;
            
            // B_bar^T 的 (i,k) 就是 B_bar 的 (k,i)
            for (uint16_t k = 0; k < rows; k++) {
                sum += B_bar_data[k * cols + i] * B_bar_data[k * cols + j];
            }
            H_data[i * NV + j] = 2.0f * sum;
        }
        // 顺便把 R 加在对角线上
        uint16_t ctrl_idx = i % NU;
        H_data[i * NV + i] += 2.0f * mpc.R[ctrl_idx];
    }

    // ==========================================
    // 5. 构建控制量约束边界 lb 和 ub
    // ==========================================
    for (int k = 0; k < MPC_MAX_N; k++) {
        lb[k * NU + 0] = -mpc.RobotParam.max_force_x;
        ub[k * NU + 0] =  mpc.RobotParam.max_force_x;
        
        lb[k * NU + 1] = -mpc.RobotParam.max_force_y;
        ub[k * NU + 1] =  mpc.RobotParam.max_force_y;
        
        lb[k * NU + 2] = -mpc.RobotParam.max_torque_z;
        ub[k * NU + 2] =  mpc.RobotParam.max_torque_z;
    }
}

/**
 * @brief 使用反向递推法计算梯度 g (f 向量)
 * @param x0 当前状态数组 (长度 NX)
 * @param X_ref 预测域内的参考轨迹 (长度 MPC_MAX_N * NX)
 * @param g_data 输出：梯度向量 (长度 NV)
 */
void mpc_g_figure(const float32_t *x0, const float32_t *X_ref, float32_t *g_data)
{
    float32_t e_all[MPC_MAX_N * NX]; // 存放正向误差 (30 * 6 * 4 = 720字节)
    float32_t x_pred[NX];
    float32_t lambda[NX] = {0};      // 协态向量

    // 1. 正向预测：算出现有惯性下的误差序列
    for(int i=0; i<NX; i++) x_pred[i] = x0[i];
    
    for (int k = 0; k < MPC_MAX_N; k++) {
        // x_{k+1} = A * x_k (单步预测)
        float32_t x_next[NX] = {0};
        for(int i = 0; i < NX; i++){
            for(int j = 0; j < NX; j++){
                x_next[i] += A_data[i * NX + j] * x_pred[j];
            }
        }
        for(int i=0; i<NX; i++) x_pred[i] = x_next[i];
        
        // 计算误差 e_k = 预测状态 - 参考状态
        for (int i = 0; i < NX; i++) {
            e_all[k * NX + i] = x_pred[i] - X_ref[k * NX + i];
        }
    }
    
    // 2. 反向递推：计算 lambda 和 g
    float m_val = (mpc.RobotParam.mass > 0.001f) ? mpc.RobotParam.mass : 1.0f;
    float Iz_val = (mpc.RobotParam.inertia_z > 0.001f) ? mpc.RobotParam.inertia_z : 1.0f;
    
    float b_fx = mpc_dt / m_val;
    float b_fy = mpc_dt / m_val;
    float b_mz = mpc_dt / Iz_val;

    // 从第 N 步倒推到第 1 步
    for (int k = MPC_MAX_N - 1; k >= 0; k--) {
        
        // 2.1 计算 A^T * lambda
        float32_t A_T_lambda[NX] = {0};
        for(int i = 0; i < NX; i++){
            for(int j = 0; j < NX; j++){
                // A^T 的 (i,j) 其实就是 A 的 (j,i)
                A_T_lambda[i] += A_data[j * NX + i] * lambda[j]; 
            }
        }
        
        // 2.2 更新 lambda = A^T * lambda + 2 * Q * e_k
        for(int i = 0; i < NX; i++){
            lambda[i] = A_T_lambda[i] + 2.0f * mpc.Q[i] * e_all[k * NX + i];
        }

        // 2.3 计算 g_k = B^T * lambda
        // 因为 B 矩阵极稀疏，只有后三行有值，直接取 lambda 对应的分量
        g_data[k * NU + 0] = b_fx * lambda[3]; // Fx 梯度
        g_data[k * NU + 1] = b_fy * lambda[4]; // Fy 梯度
        g_data[k * NU + 2] = b_mz * lambda[5]; // Mz 梯度
    }
}
