/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/fast_stop_trajectory_fallback/fast_stop_trajectory_fallback.h"

#include <utility>
#include <vector>

#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

SpeedData FastStopTrajectoryFallback::GenerateFallbackSpeed(
    const EgoInfo* ego_info, const double stop_distance) { // @param 两个ego_info和stop_distance
  // 紧急停车轨迹生成器
  AERROR << "Fallback using piecewise jerk speed!";
  const double init_v = ego_info->start_point().v(); // 当前ego_info的初始速度
  const double init_a = ego_info->start_point().a(); // 当前ego_info的初始加速度
  AWARN << "init_v = " << init_v << ", init_a = " << init_a;
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // if already stopped
  // 若车辆已停止（速度和加速度≤0），则直接返回空速速数据
  if (init_v <= 0.0 && init_a <= 0.0) {
    AWARN << "Already stopped! Nothing to do in GenerateFallbackSpeed()";
    SpeedData speed_data;
    speed_data.AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
    SpeedProfileGenerator::FillEnoughSpeedPoints(&speed_data);
    return speed_data;
  }

  std::array<double, 3> init_s = {0.0, init_v, init_a};

  // TODO(all): dt is too small;
  double delta_t = FLAGS_fallback_time_unit; // 设置时间步长
  double total_time = FLAGS_fallback_total_time; // 设置总规划时间
  const size_t num_of_knots = static_cast<size_t>(total_time / delta_t) + 1; // 轨迹节点总数

  // 使用PiecewiseJerkSpeedProblem类设置初始状态、时间步长、边界约束等参数
  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);

  std::vector<double> end_state_ref(num_of_knots, stop_distance);
  piecewise_jerk_problem.set_x_ref(1.0, std::move(end_state_ref));

  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  piecewise_jerk_problem.set_x_bounds(0.0, std::fmin(stop_distance, 100.0));
  piecewise_jerk_problem.set_dx_bounds(
      0.0, std::fmin(FLAGS_planning_upper_speed_limit, init_v));
  piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(), 0.0);
  piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                        0.0);

  // Solve the problem
  // 求解优化问题。若求解失败，则记录错误日志并返回通过GenerateStopProfile生成的停车轨迹
  if (!piecewise_jerk_problem.Optimize()) {
    AERROR << "Piecewise jerk fallback speed optimizer failed!";
    return GenerateStopProfile(init_v, init_a);
  }

  // Extract output
  // 提取并输出优化问题的解
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  for (size_t i = 0; i < num_of_knots; ++i) {
    // 遍历所有时间节点，打印每个时刻的位置、速度和加速度值，时间间隔为delta_t
    ADEBUG << "For[" << delta_t * static_cast<double>(i) << "], s = " << s[i]
           << ", v = " << ds[i] << ", a = " << dds[i];
  }

  // 初始化SpeedData对象，并添加第一个速度点
  SpeedData speed_data;
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (size_t i = 1; i < num_of_knots; ++i) {
    // Avoid the very last points when already stopped
    // 遍历后续节点，若距离或速度无效则停止循环
    if (s[i] - s[i - 1] <= 0.0 || ds[i] <= 0.0) {
      break;
    }
    // 计算时间、加速度变化率，并将有效速度点追加到speed_data
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
  }
  // 调用FillEnoughSpeedPoints补充足够多的速度点
  SpeedProfileGenerator::FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}

SpeedData FastStopTrajectoryFallback::GenerateStopProfile(
    const double init_speed, const double init_acc) {
  // 车辆紧急减速的轨迹生成逻辑 -> 分段的jerk优化策略求解失败时、采用固定的预设减速度对Speed_data信息的规划
  AERROR << "Slowing down the car within a constant deceleration with fallback "
            "stopping profile.";
  SpeedData speed_data;

  // 初始化参数（最大时间、时间步长、减速度）
  const double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;

  double pre_s = 0.0;
  double pre_v = init_speed;
  double s = 0.0;
  double v = 0.0;
  double acc = FLAGS_slowdown_profile_deceleration;
  speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, init_acc, 0.0);
  for (double t = unit_t; t < max_t; t += unit_t) {
    v = std::fmax(0.0, pre_v + unit_t * acc);
    s = std::fmax(pre_s, pre_s + 0.5 * (pre_v + (pre_v + v)) * unit_t);
    speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);
    pre_s = s;
    pre_v = v;
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}

}  // namespace planning
}  // namespace apollo
