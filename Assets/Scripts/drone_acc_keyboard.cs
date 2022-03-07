/// <summary>
/// keyboard - 键盘飞行
/// </summary>

using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.OdeSolvers;

public class drone_acc_keyboard : MonoBehaviour
{
    #region 参数初始化
    // 初始化机体参数
    double mass = 4;        double gravity = 9.8;   double length_drone_arm = 0.325;
    double I_xx = 7.278e-2; double I_yy = 7.278e-2; double I_zz = 1.367e-1;
    double Ct = 4.801e-5;   double Cm = 9.758e-7; // 螺旋桨推力系数  螺旋桨力矩系数
    public Rigidbody rb;
    // 初始化系统状态
    public double y_axis_input = 0;
    public double x_axis_input = 0;
    public double z_axis_input = 0;
    private Vector<double> target_x_dot = Vector<double>.Build.Dense(new[] {0.0, 0.0, 0.0});
    private Vector<double> cur_state = Vector<double>.Build.Dense(new[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    #endregion

    private void FixedUpdate() 
    {
        Vector<double> target_acc = keyboard_input();
        Vector<double> moter_speed = PID_controller_without_PWM(cur_state, target_acc, target_x_dot);
        Vector<double> U_model_input = throttle_to_force_without_PWM(moter_speed);
        cur_state = drone_dynamics(U_model_input, cur_state, mass, gravity);
        drone_update_anmiation(cur_state);

    }

    Vector<double> keyboard_input()
    {
        y_axis_input = Input.GetAxis("z-axis") + y_axis_input; 
        x_axis_input = Input.GetAxis("x-axis") + x_axis_input;
        z_axis_input = Input.GetAxis("forward") + z_axis_input; // 竖直方向
        Vector<double> target_acc = Vector<double>.Build.Dense(new[] {x_axis_input, z_axis_input, y_axis_input});
        return target_acc;
    }

    public Vector<double> PID_controller_without_PWM(Vector<double> cur_state, Vector<double> target_pos_in_PID, Vector<double> target_vel_in_PID)
    {
        double k1 = 4; double k2 = 10;
        double x = cur_state[8];    double y = cur_state[10];       double z = cur_state[6];
        double v_x = cur_state[9];  double v_y = cur_state[11];     double v_z = cur_state[7];
        double phi = cur_state[0];  double theta = cur_state[2];    double psi = cur_state[4];
        double v_phi = cur_state[1];double v_theta = cur_state[3];  double v_psi = cur_state[5];
        // 加入饱和策略
        if(psi > Math.PI)
        {
            psi = Math.PI;
        }
        if(theta > Math.PI)
        {
            theta = Math.PI;
        }
        if(phi > Math.PI)
        {
            phi = Math.PI;
        }
        double[] array_u = {0.0, 0.0, 0.0, 0.0};
        Matrix<double> u = CreateMatrix.Dense(4, 1, array_u); // 初始化控制量
        Vector<double> cur_pos = Vector<double>.Build.Dense(new[] {x, y, z});
        Vector<double> cur_vel = Vector<double>.Build.Dense(new[] {v_x, v_y, v_z});
        Vector<double> input_delta = k1*(target_pos_in_PID - cur_pos) + k2*(target_vel_in_PID - cur_vel);   // PID控制输出量
        Vector<double> mass_gravity = Vector<double>.Build.Dense(new[] {0, 0, gravity});                    // 重力加速度
        Vector<double> attr = Vector<double>.Build.Dense(new[] {0.0, 0.0, 0.0});
        for(int index = 0; index < 3; index++)
        {
            attr[index] = Math.Tanh(input_delta[index])*2 + mass_gravity[index];    // Ud + g
        }
        double[] rot_array = {Math.Cos(psi), Math.Sin(psi), 0, -Math.Sin(psi), Math.Cos(psi), 0, 0, 0, 1};
        Matrix<double> rot = CreateMatrix.Dense(3, 3, rot_array);   // 机体坐标系B转换到地面坐标系{E}的旋转矩阵之一 绕x轴的旋转矩阵之一
        Matrix<double> attr_matrix = rot.Transpose()*attr.ToColumnMatrix();
        u[0, 0] = mass*attr_matrix.L2Norm();
        double phi_p = Math.Asin(-attr_matrix[1, 0] / attr_matrix.L2Norm());    // 期待的角度 --- 横滚角
        double theta_p = Math.Atan(attr_matrix[0, 0] / attr_matrix[2, 0]);      // 期待的角度 --- 俯仰角
        if(attr_matrix[2, 0] < 0)
        {
            theta_p=theta_p-Math.PI;
        }
        double psi_p = 0;   // 期待的角度 --- 横滚角
        u[3, 0] = I_zz * Math.Tanh(angleDelta(psi_p,psi)+0-v_psi);
        if(u[3, 0] > 0.5)
        {
            Debug.Log("error in here!");
        }
        u[1, 0] = 2*I_xx * (u[0, 0] + u[3, 0]) / 2 * Math.Tanh(angleDelta(phi_p,phi)+0-1*v_phi) / length_drone_arm;
        u[2, 0] = 2*I_yy * (u[0, 0] - u[3, 0]) / 2 * Math.Tanh(angleDelta(theta_p,theta)+0-1*v_theta)/length_drone_arm;
        #region 反解转速
        double tmp_array_inv_tr_1 = 5207.2; double tmp_array_inv_tr_2 = 8.02261708248223e-13;   double tmp_array_inv_tr_3 = 10414.4969797959;
        double tmp_array_inv_tr_4 = 256200.040992007;   double tmp_array_inv_tr_5 = 2.86638405825313e-13;
        double[] array_inv_tr = {tmp_array_inv_tr_1, tmp_array_inv_tr_2, -tmp_array_inv_tr_3, -tmp_array_inv_tr_4, tmp_array_inv_tr_1, -tmp_array_inv_tr_3, tmp_array_inv_tr_5, tmp_array_inv_tr_4, tmp_array_inv_tr_1, tmp_array_inv_tr_2, tmp_array_inv_tr_3, -tmp_array_inv_tr_4, tmp_array_inv_tr_1, tmp_array_inv_tr_3, tmp_array_inv_tr_5, tmp_array_inv_tr_4};
        Matrix<double> inv_tr = CreateMatrix.Dense(4, 4, array_inv_tr);
        inv_tr = inv_tr.Transpose();
        Matrix<double> omega2 = inv_tr * u;
        for (int i = 0; i<4; i++)
        {
            if(omega2[i, 0] < 0)
            {
                omega2[i, 0] = 0;
            }
            omega2[i, 0] = Math.Pow(omega2[i, 0], 0.5);
        }
        Vector<double> omega = Vector<double>.Build.Dense(new[] {omega2[0, 0], omega2[1, 0], omega2[2, 0], omega2[3, 0]});
        #endregion
        return omega;
    }

    Vector<double> throttle_to_force_without_PWM(Vector<double> moter_speed)
    {
        var moter_speed_vector = CreateVector.Dense(new double[] {moter_speed[0], moter_speed[1], moter_speed[2], moter_speed[3]});
        double U_model_input_1 = Ct * Math.Pow(moter_speed_vector.L2Norm(), 2);
        double U_model_input_2 = Ct * (Math.Pow(moter_speed_vector[3],2) - Math.Pow(moter_speed_vector[1],2));
        double U_model_input_3 = Ct * (Math.Pow(moter_speed_vector[2],2) - Math.Pow(moter_speed_vector[0],2));
        double U_model_input_4 = Cm * (Math.Pow(moter_speed_vector[0],2) - Math.Pow(moter_speed_vector[1],2) + Math.Pow(moter_speed_vector[2],2) - Math.Pow(moter_speed_vector[3],2));
        double sigma_r = moter_speed_vector[0] - moter_speed_vector[1] + moter_speed_vector[2] - moter_speed_vector[3];
        var U_model_input = CreateVector.Dense(new double[] {U_model_input_1, U_model_input_2, U_model_input_3, U_model_input_4, sigma_r});
        return U_model_input;
    }

    /// <summary>
    /// 计算角度的差值
    /// </summary>
    /// <param name="p2"></param>
    /// <param name="p1"></param>
    /// <returns></returns>
    double angleDelta(double p2, double p1)
    {
        double[] tmp_1_array = {Math.Cos(p1), Math.Sin(p1), Math.Cos(p2), Math.Sin(p2)};
        Matrix<double> tmp_1 = CreateMatrix.Dense(2, 2, tmp_1_array);
        double[] tmp_2_1_array = {Math.Cos(p1), Math.Sin(p1)};
        double[] tmp_2_2_array = {Math.Cos(p2), Math.Sin(p2)};
        Matrix<double> tmp_2_1 = CreateMatrix.Dense(1, 2, tmp_2_1_array);
        Matrix<double> tmp_2_2 = CreateMatrix.Dense(2, 1, tmp_2_2_array);
        Matrix<double> tmp_2 = tmp_2_1 * tmp_2_2;
        double tmp_1_Determinant = tmp_1.Determinant();
        double ang = Math.Sign(tmp_1_Determinant) * Math.Acos(tmp_2[0, 0]);
        return ang;
    }

    /// <summary>
    /// 描述机体模型
    /// </summary>
    /// <param name="U_model_input"></param>
    /// <param name="cur_state"></param>
    /// <param name="masss"></param>
    /// <param name="gravity"></param>
    /// <returns></returns>
    public Vector<double> drone_dynamics(Vector<double> U_model_input, Vector<double> cur_state, double mass, double gravity)
    {
        int step_number = 45;
        double start_time = 0;
        double end_time = Time.fixedDeltaTime;
        Func<double, Vector<double>, Vector<double>> ode_system = ode_function(U_model_input);
        Vector<double>[] results = RungeKutta.FourthOrder(cur_state.SubVector(0,12), start_time, end_time, step_number, ode_system);
        // 这里面cur_state[12]是为了拿到加速度信息
        cur_state.SetSubVector(0, 12, results[step_number - 1]); // 计算后的终值当做下一次循环的初值
        double z_double_dot = -gravity + U_model_input[0] / mass; // 这块有问题
        cur_state[12] = z_double_dot;
        if(cur_state[6] <= 0)
        {
            cur_state[6] = 0;
        }
            
        if(cur_state[6] == 0)
        {
            cur_state[7] = 0;
        }
        return cur_state;
    }

    Func<double, Vector<double>, Vector<double>> ode_function(Vector<double> U_model_input)
    {
        return (t, Z) =>
        {
            double I_xx = 7.278e-2; double I_yy = 7.278e-2; double I_zz = 1.367e-1; double J_t = I_xx + I_yy + I_zz; // 转动惯量
            double mass = 4; double length_drone_arm = 0.325; double g = 9.8;
            double[] x = Z.ToArray();
            double Phi = x[0];      double p = x[1];    // 滚转角 在Unity对应z轴 主视角
            double theta = x[2];    double q = x[3];    // 俯仰角 在Unity对应x轴
            double Psi = x[4];      double r = x[5];    // 偏航角 在Unity对应y轴
            double z = x[6];        double z_dot = x[7];
            double x_corr = x[8];   double x_dot = x[9];
            double y = x[10];       double y_dot = x[11];
            double U_1 = U_model_input[0]; double U_2 = U_model_input[1]; double U_3 = U_model_input[2]; double U_4 = U_model_input[3]; double sigma_own = U_model_input[4];
            // 机体动态方程
            double phi_dot = p;     double p_dot = q * r * (I_yy - I_zz)/I_xx - q * J_t * sigma_own + length_drone_arm / I_xx * U_2;
            double theta_dot = q;   double q_dot = p * r * (I_zz - I_xx)/I_yy + p * J_t * sigma_own + length_drone_arm / I_yy * U_3;
            double Psi_dot = r;     double r_dot = p * q * (I_xx - I_yy) / I_zz + U_4 * length_drone_arm / I_zz;
            double z_double_dot = -g + Mathf.Cos((float)Phi) * Mathf.Cos((float)theta) * U_1 / mass;
            double x_double_dot = (Mathf.Cos((float)Phi)*Mathf.Sin((float)theta)*Mathf.Cos((float)Psi) + Mathf.Sin((float)Phi)*Mathf.Sin((float)Psi)) * U_1 / mass;
            double y_double_dot = (Mathf.Cos((float)Phi)*Mathf.Sin((float)theta)*Mathf.Sin((float)Psi) - Mathf.Sin((float)Phi)*Mathf.Cos((float)Psi)) * U_1 / mass;
            return Vector<double>.Build.Dense(new[] {phi_dot, p_dot, theta_dot, q_dot, Psi_dot, r_dot, z_dot, z_double_dot, x_dot, x_double_dot, y_dot, y_double_dot});
        };
    }
    
    void drone_update_anmiation(Vector<double> cur_state)
    {
        rb.position = Vector3.up * (float)cur_state[6] + Vector3.forward * (float)cur_state[10] + Vector3.right * (float)cur_state[8];
        transform.eulerAngles = new Vector3(Mathf.Rad2Deg * (float)cur_state[2], Mathf.Rad2Deg * (float)cur_state[4], Mathf.Rad2Deg * (float)cur_state[0]);
    }
}
