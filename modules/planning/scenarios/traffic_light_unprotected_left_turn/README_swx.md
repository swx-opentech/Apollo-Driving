# 没有路权保护的红绿灯->左转
### 由 `swx-opentech` 研究并翻译！

`TrafficLightUnprotectedLeftTurnScenario` 是没有路权保护的红绿灯左转场景。在该场景下，主车在左转车道线上

--
## stage_approach.cc
### Process 函数
* 检查场景 not null
* 获取当前场景的上下文对象、提取配置信息 `ScenarioTrafficLightUnprotectedLeftTurnConfig`
* 检查流水线配置是否启用 `pipeline_config_.enabled()`
* 通过 `LimitCruiseSpeed` 将巡航速度设为接近路口时的较低值
* 执行参考线上的规划
* 检查交通灯重叠区域，如果没有则结束场景
* 遍历当前交通信号灯重叠区域ID，查找参考线上的对应重叠对象
    > 遍历 `context->current_traffic_light_overlap_ids` 中的每个 `ID`
    >
    > 调用 `GetOverlapOnReferenceLine` 获取参考线上与该 `ID` 对应的信号灯重叠对象。若未找到有效重叠对象，则跳过当前循环
    >
    > 更新交通信号灯的状态，使其等于当前重叠区域的信号灯状态
    >
    > 设置路权：标识当前交通灯控制的路口为非优先通行状态
    >
    > 获取到停车线的距离`distance_adc_to_stop_line`，并获取交通信号灯颜色 `signal_color`
    >
    > 如果距离停止线小于0，调用 `FinishStage` 结束当前阶段
    >
    > 判断交通信号灯状态和车辆距离停车线的距离是否满足继续行驶的条件
    ```cpp
    if (signal_color != TrafficLight::GREEN || // 如果信号灯不是绿色
        distance_adc_to_stop_line >=
            scenario_config.max_valid_stop_distance()) { // 车辆到停车线的距离大于允许的最大有效停车距离
      traffic_light_all_done = false; // 则将标志位 traffic_light_all_done 设为 false 并跳出循环
      break;
    }
    ```
* 如果 `traffic_light` 为空，则调用 `FinishScenario()` 结束整个场景
* 如果所有交通灯任务已完成，则调用 `FinishStage(frame)` 结束当前阶段
* 否则，将阶段状态设置为运行中（`RUNNING`）并返回结果

### FinishStage 函数
* 获取上下文对象、访问配置信息
* 根据车辆速度决定是否进入“缓行”（creep）阶段
    > 若车速超过设定阈值，则跳过缓行，直接进入巡航阶段 `TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE`
    >
    > 否则，清空并更新交通灯相关的规划状态，并记录缓行开始时间，进入缓行阶段 `TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP`
* 调用 `LimitCruiseSpeed` 方法，将巡航速度限制为默认值
* 返回阶段执行结果，状态为“已完成”（`FINISHED`）

---

## stage_creep.cc
### Init 函数
初始化场景配置

### Process 函数
* 检查场景not null
* 获取当前场景的上下文对象、提取配置信息 `ScenarioTrafficLightUnprotectedLeftTurnConfig`
* 检查流水线配置是否启用 `pipeline_config_.enabled()`
* 遍历所有可行驶的参考线信息，并对每条参考线执行蠕行决策处理
    > 遍历 `frame` 中的所有参考线信息
    >
    > 若某条参考线不可行驶（`IsDrivable()` 返回 `false`），则记录错误并跳出循环
    >
    > 对可行驶的参考线调用 `ProcessCreep` 函数进行蠕行决策处理
    >
    > 若处理失败（`ret.ok()` 为 `false`），则记录错误信息并跳出循环
* 调用`ExecuteTaskOnReferenceLine`函数在参考线上执行规划任务，并检查错误情况
* 检查当前交通灯重叠ID是否为空，若为空则调用`FinishScenario`结束当前场景
* 检查当前交通信号灯是否在参考路径上存在重叠区域 `reference_line_info.GetOverlapOnReferenceLine`
* 未找到重叠区域，则调用 `FinishScenario()` 结束场景
* 标记当前交通灯控制的路口为无优先通行权状态 `reference_line_info.SetJunctionRightOfWay(current_traffic_light_overlap->start_s, false);`
* creep
    > 计算等待时间和超时时间：通过当前时间和起始时间差计算已等待时间，并获取配置的超时时间
    >
    > 确定缓行停止位置：调用 `GetCreepFinishS` 计算车辆应停止的位置，并与当前车位置比较距离
    >
    > 若车辆已到达或超过目标位置，则生成固定距离的缓行速度曲线 -> `SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);`
    >
    > 用 `CheckCreepDone` 判断是否满足结束条件，若满足则结束当前阶段
* 设置为“运行中”（`RUNNING`）

---

### stage_intersection_cruise.cc
* 检查场景not null
* 调用 `ExecuteTaskOnReferenceLine` 执行规划任务，若出错则记录错误日志
* 调用 `CheckDone` 函数，传入帧数据、规划上下文和布尔值，判断当前阶段是否已完成，结果存储在 `stage_done` 中

---

## traffic_light_unprotected_left_turn_scenario.cc
### IsTransferable 函数
* 检查是否有跟随车道等指令
* 检查参考线信息中是否存在首次遇到的重叠区域
* 筛选出信号灯相关的路径重叠区域
    > 遍历 `first_encountered_overlaps`，若遇到停车标志（`STOP_SIGN`）或让行标志（`YIELD_SIGN`），直接返回 `false`
    >
    > 若遇到信号灯标志（`SIGNAL`），记录其对应的重叠区域并跳出循环
    >
    > 未找到信号灯标志，则返回 `false`
* 获取交通信号灯相关的路径重叠信息，并计算自车前缘在参考线上的位置
* 筛选出与交通标志重叠区域距离在2米以内的交通灯重叠区域，并将它们存储到`next_traffic_lights`向量中
```cpp
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  for (const auto& overlap : traffic_light_overlaps) {
    // 遍历traffic_light_overlaps中的每个重叠区域overlap
    const double dist = overlap.start_s - traffic_sign_overlap->start_s; // 计算当前交通灯重叠区域与交通标志重叠区域起点的距离dist
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 若距离绝对值小于等于kTrafficLightGroupingMaxDist米，则将该交通灯重叠区域加入结果列表next_traffic_lights
      next_traffic_lights.push_back(overlap);
    }
  }
```
* 判断车辆是否进入交通灯场景
    > 遍历所有前方交通灯（`next_traffic_lights`），计算车辆与交通灯的距离
    >
    > 若距离小于等于0或超过检查范围（`start_check_distance`），跳过该交通灯
    >
    > 获取交通灯颜色，若为红色、黄色或未知，则标记为进入交通灯场景（`traffic_light_scenario = true`）并退出循环
* 若 `traffic_light_scenario` 为假，直接返回 `false`
* 获取路径在交通标志重叠起点处的转向类型`reference_line_info.GetPathTurnType(traffic_sign_overlap->start_s);`，若不是左转，则返回 `false` 
* 遍历下一个交通灯集合，将每个交通灯的ID添加到列表中

### Enter 函数
* 获取当前参考线信息中的第一个重叠对象集合
* 遍历这些重叠对象，查找类型为信号灯（`SIGNAL`）的对象，找到后，记录其ID并跳出循环
* 检查当前交通信号灯重叠区域ID是否为空
* 查找与当前交通信号灯在同一位置或组的所有交通信号灯
    > 取参考线上的所有交通信号灯重叠信息 `reference_line_info`
    >
    > 使用 `std::find_if` 查找与 `current_traffic_light_overlap_id` 匹配的信号灯
    >
    > 如果未找到匹配项，则清空规划状态中的交通信号灯信息并返回 `true`
* 将距离当前交通信号灯一定范围内的其他交通信号灯加入规划上下
    > 遍历所有交通信号灯重叠区域（`traffic_light_overlaps`）
    >
    > 计算每个信号灯与当前信号灯的距离（`dist`）
    >
    > 若距离小于等于最大分组距离（`kTrafficLightGroupingMaxDist`），则将其ID添加到规划状态中，并记录日志