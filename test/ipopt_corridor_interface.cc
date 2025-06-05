/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */
#include "trajectory_smoother/ipopt_corridor_interface.h"

// 1.通过构造函数获取参数，并把所有参数赋值给私有成员变量。2.通过主函数相关处理。3.
//
DistanceApproachIPOPTCorridorInterface::DistanceApproachIPOPTCorridorInterface(
    const size_t horizon, const double ds_init, const Eigen::MatrixXd &ego,
    const Eigen::MatrixXd &xWS, const Eigen::MatrixXd &uWS,
    const Eigen::MatrixXd &x0, const Eigen::MatrixXd &xf,
    const Eigen::MatrixXd &last_time_u, const std::vector<double> &XYbounds,
   
    const Eigen::MatrixXd &f_driving_bound,
    const Eigen::MatrixXd &b_driving_bound,
    const PlannerOpenSpaceConfig &planner_open_space_config)
    : ds_init_(ds_init), // ts采样时间间隔
      ego_(ego), xWS_(xWS), uWS_(uWS), x0_(x0), xf_(xf),
      last_time_u_(last_time_u), XYbounds_(XYbounds),
      f_driving_bound_(f_driving_bound), b_driving_bound_(b_driving_bound) {
  
  horizon_ = static_cast<int>(horizon);
  

 
  weight_state_x_ = distance_approach_config_.weight_x; //是为了将读和写分开
  weight_state_y_ = distance_approach_config_.weight_y;
  weight_state_phi_ = distance_approach_config_.weight_phi;
  weight_state_v_ = distance_approach_config_.weight_v;
  weight_input_steer_ = distance_approach_config_.weight_steer;
  weight_input_a_ = distance_approach_config_.weight_a;
  weight_rate_steer_ = distance_approach_config_.weight_steer_rate;
  weight_rate_a_ = distance_approach_config_.weight_a_rate;
  weight_stitching_steer_ = distance_approach_config_.weight_steer_stitching;
  weight_stitching_a_ = distance_approach_config_.weight_a_stitching;
  weight_first_order_time_ = distance_approach_config_.weight_first_order_time;
  weight_second_order_time_ =
      distance_approach_config_.weight_second_order_time;
  min_safety_distance_ = distance_approach_config_.min_safety_distance;
  max_steer_angle_ =
      vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio;
  max_speed_forward_ = distance_approach_config_.max_speed_forward;
  max_speed_reverse_ = distance_approach_config_.max_speed_reverse;
  max_acceleration_forward_ =
      distance_approach_config_.max_acceleration_forward;
  max_acceleration_reverse_ =
      distance_approach_config_.max_acceleration_reverse;
  min_time_sample_scaling_ = distance_approach_config_.min_time_sample_scaling;
  max_time_sample_scaling_ = distance_approach_config_.max_time_sample_scaling;
  max_steer_rate_ =
      vehicle_param_.max_steer_angle_rate / vehicle_param_.steer_ratio;
  use_fix_time_ = distance_approach_config_.use_fix_time;
  wheelbase_ = vehicle_param_.wheel_base;
  enable_constraint_check_ = distance_approach_config_.enable_constraint_check;
  enable_jacobian_ad_ = distance_approach_config_.enable_jacobian_ad;
}
// IPOPT APP step 1
bool DistanceApproachIPOPTCorridorInterface::get_nlp_info(
    int &n, int &m, int &nnz_jac_g, int &nnz_h_lag, 
    //nnz_jac_g雅克比矩阵非零值个数 nnz_h_lag  海森矩阵非零值个数，需要考虑矩阵的对称性，使用adol_c来求
    IndexStyleEnum &index_style) {
  std::cout << "get_nlp_info" << std::endl;
  // n1 : states variables, 4 * (N+1)  x,y,θ，v状态变量，为什么需要v,
  int n1 = 4 * (horizon_ + 1); // n + 1终点
  std::cout << "状态变量数量n1: " << n1 << std::endl;
  // n2 : control inputs variables δ，a控制变量
  int n2 = 2 * horizon_;
  std::cout << "控制变量n2: " << n2 << std::endl;
  // n3 : sampling time variables tf时间变量
  int n3 = 1;
  std::cout << "时间变量n3: " << n3 << std::endl;

  // m1 : dynamics constatins 动力学约束
  int m1 = 4 * horizon_;
  // m2 : control rate constraints (only steering) 控制变化率约束方向盘
  int m2 = horizon_;
  // m3 : sampling time equality constraints 采样时间约束
  int m3 = 0;
  // m4 : obstacle constraints 行车隧道约束包含于变量约束
  // x_f=f1(x),y_f=f2(x),x_b,y_b   包括起点，终点约束
  int m4 = 4 * (horizon_ + 1);

  num_of_variables_ = n1 + n2 + n3;
  num_of_constraints_ = m1 + m2 + m4;

  // m=动力学约束+控制变换率约束+行车隧道约束
  //  number of variables
  n = num_of_variables_;
  std::cout << "num_of_variables_ " << num_of_variables_ << std::endl;
  // number of constraints
  m = num_of_constraints_;
  std::cout << "num_of_constraints_ " << num_of_constraints_ << std::endl;

  generate_tapes(n, m, &nnz_jac_g, &nnz_h_lag); //自动微分记录

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}
/*
min f(x)
s.t. g_l≤ g(x) ≤g_u
      x_l≤ x ≤x_u
*/

bool DistanceApproachIPOPTCorridorInterface::get_bounds_info(int n, double *x_l,
                                                             double *x_u, int m,
                                                             double *g_l,
                                                             double *g_u) {
  std::cout << "get_bounds_info" << std::endl;

  //x_l x_u 


  // ACHECK(XYbounds_.size() == 4)
  //     << "XYbounds_ size is not 4, but" << XYbounds_.size();

  // Variables: includes state, u, sample time and lagrange
  // multipliers为什么把x上下界变为约束呢？？x上下届约束
  // 1. state variables, 4 * [0, horizon]
  // start point pose
  int variable_index = 0;

  x_l[variable_index] = x0_(0, 0);
  x_u[variable_index] = x0_(0, 0);

  x_l[variable_index + 1] = x0_(1, 0);
  x_u[variable_index + 1] = x0_(1, 0);

  x_l[variable_index + 2] = x0_(2, 0);
  x_u[variable_index + 2] = x0_(2, 0);

  x_l[variable_index + 3] = x0_(3, 0);
  x_u[variable_index + 3] = x0_(3, 0);

  variable_index += 4;

  // During horizons, 2 ~ N ???此处应该为 i<horizon_+1吧,不是，因为从0开始索引的
  //中间点约束为地图边界
  for (int i = 1; i < horizon_; ++i) {
    // x
    x_l[variable_index] = XYbounds_[0];
    x_u[variable_index] = XYbounds_[1];

    // y
    x_l[variable_index + 1] = XYbounds_[2];
    x_u[variable_index + 1] = XYbounds_[3];

    // phi
    x_l[variable_index + 2] = -2e19;
    x_u[variable_index + 2] = 2e19;

    // v
    x_l[variable_index + 3] = -max_speed_reverse_;
    x_u[variable_index + 3] = max_acceleration_forward_;

    variable_index += 4;
  }

  // end point pose 0.1 0.2是什么作用，松弛吗？？？应该是
  x_l[variable_index] = xf_(0, 0) - 0.1;
  x_u[variable_index] = xf_(0, 0) + 0.1;
  x_l[variable_index + 1] = xf_(1, 0) - 0.1;
  x_u[variable_index + 1] = xf_(1, 0) + 0.1;
  x_l[variable_index + 2] = xf_(2, 0) - 0.2;
  x_u[variable_index + 2] = xf_(2, 0) + 0.2;
  x_l[variable_index + 3] = xf_(3, 0);
  x_u[variable_index + 3] = xf_(3, 0);

  variable_index += 4;
  std::cout << "x上下界 variable_index after adding state variables : "
            << variable_index << std::endl;

  // 2. control variables, 2 * [0, horizon_-1]
  for (int i = 0; i < horizon_; ++i) {
    // steer
    x_l[variable_index] = -max_steer_angle_;
    x_u[variable_index] = max_steer_angle_;

    // a
    x_l[variable_index + 1] = -max_acceleration_reverse_;
    x_u[variable_index + 1] = max_acceleration_forward_;

    variable_index += 2;
  }

  // 3.dt约束
  x_l[variable_index] = 0;
  x_u[variable_index] = 10;
  variable_index += 2;

  // g_l g_u 约束边界信息
  // Constraints: includes four state Euler forward constraints, three
  // 1. dynamics constraints 4 * [0, horizons-1] f(x,u)=0实际是四个方程 等式约束形式吗？？
  int constraint_index = 0;
  for (int i = 0; i < 4 * horizon_; ++i) {
    g_l[i] = 0.0;
    g_u[i] = 0.0;
  }
  constraint_index += 4 * horizon_;

  std::cout
      << "constraint_index after adding Euler forward dynamics constraints: "
      << constraint_index << std::endl;

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  for (int i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = -max_steer_rate_;
    g_u[constraint_index] = max_steer_rate_;
    ++constraint_index;
  }

  std::cout << "constraint_index after adding steering rate constraints: "
            << constraint_index << std::endl;

  // 4. 行车隧道约束4*n (x_min,x_max,y_min,y_max)'
  for (int i = 0; i < horizon_ + 1; ++i) {
    g_l[constraint_index] = f_driving_bound_(0, i); // xf_min
    g_u[constraint_index] = f_driving_bound_(1, i); // xf_max

    g_l[constraint_index + 1] = f_driving_bound_(2, i);
    g_u[constraint_index + 1] = f_driving_bound_(3, i);

    g_l[constraint_index + 2] = b_driving_bound_(0, i); // xb_min
    g_u[constraint_index + 2] = b_driving_bound_(1, i); // xb_max

    g_l[constraint_index + 3] = b_driving_bound_(2, i);
    g_u[constraint_index + 3] = b_driving_bound_(3, i);

    constraint_index += 4;
  }

  
  std::cout << "get_bounds_info out" << std::endl;
  return true;
}
// IPOPT APP step 3
bool DistanceApproachIPOPTCorridorInterface::get_starting_point(
    int n, bool init_x, double *x, bool init_z, double *z_L, double *z_U, int m,
    bool init_lambda, double *lambda) {
      //assert(init_x==0)之类的是什么意思
  std::cout << "get_starting_point" << std::endl;
  // ACHECK(init_x) << "Warm start init_x setting failed";

  // CHECK_EQ(horizon_, uWS_.cols());
  // CHECK_EQ(horizon_ + 1, xWS_.cols());

  // 1. state variables 4 * (horizon_ + 1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    int index = i * 4;
    for (int j = 0; j < 4; ++j) {
      x[index + j] = xWS_(j, i);
    }
  }

  // 2. control variable initialization, 2 * horizon_
  for (int i = 0; i < horizon_; ++i) {
    int index = i * 2;
    x[control_start_index_ + index] = uWS_(0, i);
    x[control_start_index_ + index + 1] = uWS_(1, i);
  }

  // 2. time scale variable initialization, horizon_ + 1
  x[time_start_index_] = 0.5; //应该为混合A*的时间间隔

  std::cout << "get_starting_point out" << std::endl;
  return true;
}

bool DistanceApproachIPOPTCorridorInterface::eval_f(int n, const double *x,
                                                    bool new_x,
                                                    double &obj_value) {
  eval_obj(n, x, &obj_value); // x为数组
  return true;
}

bool DistanceApproachIPOPTCorridorInterface::eval_grad_f(int n, const double *x,
                                                         bool new_x,
                                                         double *grad_f) {
  gradient(tag_f, n, x, grad_f);
  return true;
}

bool DistanceApproachIPOPTCorridorInterface::eval_g(int n, const double *x,
                                                    bool new_x, int m,
                                                    double *g) {
  eval_constraints(n, x, m, g); //调用自动微分
  // if (enable_constraint_check_) check_g(n, x, m, g);
  return true;
}

bool DistanceApproachIPOPTCorridorInterface::eval_jac_g(int n, const double *x,
                                                        bool new_x, int m,
                                                        int nele_jac, int *iRow,
                                                        int *jCol,
                                                        double *values) {
                                                          //不使用手动求解，提高适用性
 
    if (values == nullptr) {
      // return the structure of the jacobian
      for (int idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      // ADOL-C函数求解雅克比矩阵
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval,
                 options_g);
      for (int idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
      }
    }
    return true;
 
}


bool DistanceApproachIPOPTCorridorInterface::eval_h(
    int n, const double *x, bool new_x, double obj_factor, int m,
    const double *lambda, bool new_lambda, int nele_hess, int *iRow, int *jCol,
    double *values) {
  if (values == nullptr) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    for (int idx = 0; idx < nnz_L; idx++) {
      iRow[idx] = rind_L[idx];
      jCol[idx] = cind_L[idx];
    }
  } else {
    // return the values. This is a symmetric matrix, fill the lower left
    // triangle only

    obj_lam[0] = obj_factor;

    for (int idx = 0; idx < m; idx++) {
      obj_lam[1 + idx] = lambda[idx];
    }
    // ADOL-C函数求解海森矩阵
    set_param_vec(tag_L, m + 1, obj_lam);
    sparse_hess(tag_L, n, 1, const_cast<double *>(x), &nnz_L, &rind_L, &cind_L,
                &hessval, options_L);

    for (int idx = 0; idx < nnz_L; idx++) {
      values[idx] = hessval[idx];
    }
  }

  return true;
}

void DistanceApproachIPOPTCorridorInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double *x, const double *z_L,
    const double *z_U, int m, const double *g, const double *lambda,
    double obj_value, const Ipopt::IpoptData *ip_data,
    Ipopt::IpoptCalculatedQuantities *ip_cq) {
  int state_index = state_start_index_;
  int control_index = control_start_index_;
  int time_index = time_start_index_;
  int dual_l_index = l_start_index_;
  int dual_n_index = n_start_index_;

  // enable_constraint_check_: for debug only
  // 1. state variables, 4 * [0, horizon]
  // 2. control variables, 2 * [0, horizon_-1]
  // 3. sampling time variables,1
  // 4. dual_l 0
  // 5. dual_n 0
  for (int i = 0; i < horizon_; ++i) {
    state_result_(0, i) = x[state_index];
    state_result_(1, i) = x[state_index + 1];
    state_result_(2, i) = x[state_index + 2];
    state_result_(3, i) = x[state_index + 3];
    control_result_(0, i) = x[control_index];
    control_result_(1, i) = x[control_index + 1];
    //    time_result_(0, i) = x[time_index] * horizon_; // tf

    state_index += 4;
    control_index += 2;
  }

  time_result_(0, 0) = x[time_index]; // dt

  state_result_(0, 0) = x0_(0, 0);
  state_result_(1, 0) = x0_(1, 0);
  state_result_(2, 0) = x0_(2, 0);
  state_result_(3, 0) = x0_(3, 0);
  // push back last horizon for states
  state_result_(0, horizon_) = xf_(0, 0);
  state_result_(1, horizon_) = xf_(1, 0);
  state_result_(2, horizon_) = xf_(2, 0);
  state_result_(3, horizon_) = xf_(3, 0);

  // memory deallocation of ADOL-C variables
  delete[] obj_lam;
  if (enable_jacobian_ad_) {
    free(rind_g);
    free(cind_g);
    free(jacval);
  }
  free(rind_L);
  free(cind_L);
  free(hessval);
}

void DistanceApproachIPOPTCorridorInterface::get_optimization_results(
    Eigen::MatrixXd *state_result, Eigen::MatrixXd *control_result,
    Eigen::MatrixXd *time_result, Eigen::MatrixXd *dual_l_result,
    Eigen::MatrixXd *dual_n_result) const {
  std::cout << "get_optimization_results" << std::endl;
  *state_result = state_result_;
  *control_result = control_result_;
  *time_result = time_result_;

  double state_diff_max = 0.0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4; ++j) {
      state_diff_max =
          std::max(std::abs(xWS_(j, i) - state_result_(j, i)), state_diff_max);
    }
  }

  // 2. control variable initialization, 2 * horizon_
  double control_diff_max = 0.0;
  for (int i = 0; i < horizon_; ++i) {
    control_diff_max = std::max(std::abs(uWS_(0, i) - control_result_(0, i)),
                                control_diff_max);
    control_diff_max = std::max(std::abs(uWS_(1, i) - control_result_(1, i)),
                                control_diff_max);
  }

  std::cout << "state_diff_max: " << state_diff_max << std::endl;
  std::cout << "control_diff_max: " << control_diff_max << std::endl;
}

//***************    start ADOL-C part ***********************************
template <class T> //这里T是adouble
void DistanceApproachIPOPTCorridorInterface::eval_obj(int n, const T *x,
                                                      T *obj_value) {
  // Objective is :
  // min control inputs 最小控制输出
  // min input rate 最小输入变化率
  // min time (if the time step is not fixed) 最小时间
  // regularization wrt warm start trajectory
  // DCHECK(ts_ != 0) << "ts in distance_approach_ is 0";
  std::cout << "代价函数eval_obj start " << std::endl;

  //为什么索引都是从零开始，而不是（x,u,t）'
  int control_index = control_start_index_; // 0+4n         2n个(δ，a)
  int time_index = time_start_index_;       // 0+4*n+2*n    1个ts
  int state_index = state_start_index_;     // 0            4n个(x，y,θ，v)

  // TODO(QiL): Initial implementation towards earlier understanding and debug
  // purpose, later code refine towards improving efficiency

  *obj_value = 0.0;
  // 1. objective to minimize state diff to warm up //1.状态变量与初始值的误差
  for (int i = 0; i < horizon_ + 1; ++i) {
    T x1_diff = x[state_index] - xWS_(0, i);
    T x2_diff = x[state_index + 1] - xWS_(1, i);
    T x3_diff = x[state_index + 2] - xWS_(2, i);
    T x4_abs = x[state_index + 3];
    *obj_value += weight_state_x_ * x1_diff * x1_diff +
                  weight_state_y_ * x2_diff * x2_diff +
                  weight_state_phi_ * x3_diff * x3_diff +
                  weight_state_v_ * x4_abs * x4_abs;
    state_index += 4;
  }

  // 2. objective to minimize u square //2.控制变量的平方
  for (int i = 0; i < horizon_; ++i) {
    *obj_value += weight_input_steer_ * x[control_index] * x[control_index] +
                  weight_input_a_ * x[control_index + 1] * x[control_index + 1];
    control_index += 2;
  }

  //此次调试仅规划一次，所以不适用拼接轨迹，也不使用上一时刻的变量
  // 3. objective to minimize input change rate for first
  // horizon//初始状态控制变化率。
  // control_index = control_start_index_;
  // T last_time_steer_rate = (x[control_index] - last_time_u_(0, 0)) /
  //                          x[time_index]; //假设最优一个优化变量是ts
  // T last_time_a_rate =
  //     (x[control_index + 1] - last_time_u_(1, 0)) / x[time_index];
  // *obj_value +=
  //     weight_stitching_steer_ * last_time_steer_rate * last_time_steer_rate +
  //     weight_stitching_a_ * last_time_a_rate * last_time_a_rate;

  // 4. objective to minimize input change rates, [0- horizon_ -2]
  // 0-n-2控制变化率

  control_index = control_start_index_;
  for (int i = 0; i < horizon_ - 1; ++i) {
    T steering_rate = (x[control_index + 2] - x[control_index]) / x[time_index];
    T a_rate = (x[control_index + 3] - x[control_index + 1]) / x[time_index];
    *obj_value += weight_rate_steer_ * steering_rate * steering_rate +
                  weight_rate_a_ * a_rate * a_rate;
    control_index += 2;
  }

  // 5. objective to minimize total time [0, horizon_]//5.做少时间目标
  T first_order_penalty =
      weight_first_order_time_ * x[time_index] * horizon_; //花费总时间代价
  *obj_value += first_order_penalty;

  std::cout << "代价函数eval_obj out " << std::endl;
}

template <class T>
void DistanceApproachIPOPTCorridorInterface::eval_constraints(int n, const T *x,
                                                              int m, T *g) {

  std::cout << "约束函数eval_constraints start " << std::endl;
  // state start index
  int state_index = state_start_index_;

  // control start index.
  int control_index = control_start_index_;

  // time start index
  int time_index = time_start_index_;

  int constraint_index = 0;

  // // 1. state constraints 4 * [0, horizons-1]
  // //动力学方程约束，离散方程horizons个
  for (int i = 0; i < horizon_; ++i) {
    // x1

    // x(k+1)=x(k)+dt*(v(k)+0.5*dt*a)*cos(φ+0.5*dt*v(k)*tan(δ)/L)
    //使用中点欧拉法进行计算
    g[constraint_index] = x[state_index + 4] -
                          (x[state_index] + x[time_index] * x[state_index + 3] *
                                                cos(x[state_index + 2]));

    
    // x2
    g[constraint_index + 1] =
        x[state_index + 5] -
        (x[state_index + 1] +
         x[time_index] * x[state_index + 3] * sin(x[state_index + 2]));
   

    // x3
    g[constraint_index + 2] =
        x[state_index + 6] -
        (x[state_index + 2] + x[time_index] * x[state_index + 3] *
                                  tan(x[control_index]) / wheelbase_);
   

    // x4
    g[constraint_index + 3] =
        x[state_index + 7] -
        (x[state_index + 3] + x[time_index] * x[control_index + 1]);

   

    control_index += 2;
    constraint_index += 4;
    state_index += 4;
  }

  std::cout
      << "constraint_index after adding Euler forward dynamics constraints "
         "updated: " //写的是前向欧拉，但实际用的是中点欧拉
      << constraint_index << std::endl;

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now控制增量约束，仅方向角
  control_index = control_start_index_;

  // First rate is compare first with stitch point
  g[constraint_index] = (x[control_index] - last_time_u_(0, 0)) / x[time_index];
  control_index += 2;
  constraint_index++;

  for (int i = 1; i < horizon_; ++i) { //控制变化率约束
    g[constraint_index] = (x[control_index] - x[control_index - 2]) /
                          x[time_index] / x[time_index];
    constraint_index++;
    control_index += 2;
  }

  // 4. Three obstacles related equal constraints, one equality constraints,
  //替换为可行驶隧道约束 4*horizon_+1个
  state_index = state_start_index_;
  control_index = control_start_index_;
  time_index = time_start_index_;

  for (int i = 0; i < horizon_ + 1; i++) {

    // //根据车辆中心计算前后覆盖圆中心
    // double x_f = x + (3 / 4 * vehicle_config_.length -
    // vehicle_config_.back_edge_to_center) * cos(theta); double y_f = y + (3 /
    // 4 * vehicle_config_.length - vehicle_config_.back_edge_to_center) *
    // sin(theta); double x_b = x + (1 / 4 * vehicle_config_.length -
    // vehicle_config_.back_edge_to_center) * cos(theta); double y_b = y + (1 /
    // 4 * vehicle_config_.length - vehicle_config_.back_edge_to_center) *
    // sin(theta); 
    //x_f
    g[constraint_index] =
        x[state_index] +
        (3 / 4 * vehicle_param_.length - vehicle_param_.back_edge_to_center) *
            cos(x[state_index + 2]);
    // y_f
    g[constraint_index + 1] =
        x[state_index + 1] +
        (3 / 4 * vehicle_param_.length - vehicle_param_.back_edge_to_center) *
            sin(x[state_index + 2]);
    // x_b
    g[constraint_index + 2] =
        x[state_index] +
        (1 / 4 * vehicle_param_.length - vehicle_param_.back_edge_to_center) *
            cos(x[state_index + 2]);
    // y_b
    g[constraint_index + 3] =
        x[state_index + 1] +
        (1 / 4 * vehicle_param_.length - vehicle_param_.back_edge_to_center) *
            sin(x[state_index + 2]);
    state_index += 4;
    constraint_index += 4;
  }

  std::cout << "约束函数eval_constraints out " << std::endl;

  
}

bool DistanceApproachIPOPTCorridorInterface::check_g(int n, const double *x,
                                                     int m, const double *g) {
  int kN = n;
  int kM = m;
  double x_u_tmp[kN];
  double x_l_tmp[kN];
  double g_u_tmp[kM];
  double g_l_tmp[kM];

  get_bounds_info(n, x_l_tmp, x_u_tmp, m, g_l_tmp, g_u_tmp);

  const double delta_v = 1e-4;
  for (int idx = 0; idx < n; ++idx) {
    x_u_tmp[idx] = x_u_tmp[idx] + delta_v;
    x_l_tmp[idx] = x_l_tmp[idx] - delta_v;
    if (x[idx] > x_u_tmp[idx] || x[idx] < x_l_tmp[idx]) {
      std::cout << "x idx unfeasible: " << idx << ", x: " << x[idx]
                << ", lower: " << x_l_tmp[idx] << ", upper: " << x_u_tmp[idx]
                << std::endl;
    }
  }

  // m1 : dynamics constatins
  int m1 = 4 * horizon_;

  // m2 : control rate constraints (only steering)
  int m2 = m1 + horizon_;

  // m3 : sampling time equality constraints
  int m3 = m2 + horizon_;

  // m4 : obstacle constraints
  int m4 = m3 + 4 * obstacles_num_ * (horizon_ + 1);

  // 5. load variable bounds as constraints
  // start configuration
  int m5 = m4 + 3 + 1;

  // constraints on x,y,v
  int m6 = m5 + 3 * (horizon_ - 1);

  // end configuration
  int m7 = m6 + 3 + 1;

  // control variable bnd
  int m8 = m7 + 2 * horizon_;

  // time interval variable bnd
  int m9 = m8 + (horizon_ + 1);

  // lambda_horizon_
  int m10 = m9 + lambda_horizon_;

  // miu_horizon_
  int m11 = m10 + miu_horizon_;

  // CHECK_EQ(m11, num_of_constraints_);

  std::cout << "dynamics constatins to: " << m1 << std::endl;
  std::cout << "control rate constraints (only steering) to: " << m2
            << std::endl;
  std::cout << "sampling time equality constraints to: " << m3 << std::endl;
  std::cout << "obstacle constraints to: " << m4 << std::endl;
  std::cout << "start conf constraints to: " << m5 << std::endl;
  std::cout << "constraints on x,y,v to: " << m6 << std::endl;
  std::cout << "end constraints to: " << m7 << std::endl;
  std::cout << "control bnd to: " << m8 << std::endl;
  std::cout << "time interval constraints to: " << m9 << std::endl;
  std::cout << "lambda constraints to: " << m10 << std::endl;
  std::cout << "miu constraints to: " << m11 << std::endl;
  std::cout << "total constraints: " << num_of_constraints_ << std::endl;

  for (int idx = 0; idx < m; ++idx) {
    if (g[idx] > g_u_tmp[idx] + delta_v || g[idx] < g_l_tmp[idx] - delta_v) {
      std::cout << "constrains idx unfeasible: " << idx << ", g: " << g[idx]
                << ", lower: " << g_l_tmp[idx] << ", upper: " << g_u_tmp[idx]
                << std::endl;
    }
  }
  return true;
}
// IPOPT APP step 2
void DistanceApproachIPOPTCorridorInterface::generate_tapes(int n, int m,
                                                            int *nnz_jac_g,
                                                            int *nnz_h_lag) {
  std::vector<double> xp(n);
  std::vector<double> lamp(m);
  std::vector<double> zl(m);
  std::vector<double> zu(m);
  std::vector<adouble> xa(n);
  std::vector<adouble> g(m);
  std::vector<double> lam(m);

  double sig;
  adouble obj_value;
  double dummy = 0.0;
  obj_lam = new double[m + 1];
  get_starting_point(n, 1, &xp[0], 0, &zl[0], &zu[0], m, 0,
                     &lamp[0]); //获取初始化值

  trace_on(tag_f); //开始记录代价函数
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_obj(n, &xa[0], &obj_value);
  obj_value >>= dummy;
  trace_off();

  trace_on(tag_g); //开始记录约束函数
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    g[idx] >>= dummy;
  }
  trace_off();

  trace_on(tag_L); //开始记录拉个朗日函数
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  for (int idx = 0; idx < m; idx++) {
    lam[idx] = 1.0;
  }
  sig = 1.0; //初始化系数因子
  eval_obj(n, &xa[0], &obj_value);
  obj_value *= mkparam(sig);
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    obj_value += g[idx] * mkparam(lam[idx]);
  }
  obj_value >>= dummy;

  trace_off();

  if (enable_jacobian_ad_) {
    rind_g = nullptr;
    cind_g = nullptr;
    jacval = nullptr;
    options_g[0] = 0; /* sparsity pattern by index domains (default) */
    options_g[1] = 0; /*                         safe mode (default) */
    options_g[2] = 0;
    options_g[3] = 0; /*                column compression (default) */

    sparse_jac(tag_g, m, n, 0, &xp[0], &nnz_jac, &rind_g, &cind_g, &jacval,
               options_g);
    *nnz_jac_g = nnz_jac;
  }

  rind_L = nullptr;
  cind_L = nullptr;
  hessval = nullptr;
  options_L[0] = 0;
  options_L[1] = 1;

  sparse_hess(tag_L, n, 0, &xp[0], &nnz_L, &rind_L, &cind_L, &hessval,
              options_L);
  *nnz_h_lag = nnz_L;
}
//***************    end   ADOL-C part ***********************************