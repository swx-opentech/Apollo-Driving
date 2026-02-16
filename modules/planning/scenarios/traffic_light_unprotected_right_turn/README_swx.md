# 没有路权保护的红绿灯->右转
### 由 `swx-opentech` 研究并翻译！

`TrafficLightUnprotectedRightTurnScenario` 是有路权保护的红绿灯右转场景，在该场景下可以实现在红绿灯路口前红灯停车，绿灯通过路口。

---
## stage_stop.cc.cc
### Process 函数
* 检查场景 not null
* 获取当前场景的上下文对象、提取配置信息
* 执行参考线上的规划任务 `ExecuteTaskOnReferenceLine`
* 初始化一些变量，用于交通灯状态的判断和路径重叠处理
* 遍历当前交通信号灯重叠区域，检查车辆与停止线的距离以及信号灯颜色，以判断是否满足通行条件
    > 遍历 `context->current_traffic_light_overlap_ids` 中的每个 `traffic_light_overlap_id`
    >
    > 通过 `id` 在参考线上查找对应的信号灯重叠区域 -> `GetOverlapOnReferenceLine` -> 没有则跳过当前循环
    >
    > 取消优先通行权，判断自车与交通信号灯停止线的距离是否在有效范围内，如果超过配置的最大停车距离，则标记 `traffic_light_all_stop` 为 `false` 并跳出循环
    > 
    > 检查交通信号灯颜色，如果信号灯颜色不是绿色（`TrafficLight::GREEN`），则设置 `false` ，并调用 `CheckTrafficLightNoRightTurnOnRed` 函数检查是否禁止右转红灯
* 当所有交通灯都处于绿灯状态，且车辆已经行驶到足够接近交通灯停止线的位置，结束当前阶段
* 判断当前是否禁止右转红灯，如车辆已经行驶到足够接近交通灯停止线的位置，且并非所有交通灯都是绿灯
    > 判断自车是否已越过停止线
    >
    > 判断是否启用右转红灯功能，若允许
    >
    > 记录停止开始时间，计算等待时间
    >
    > 若等待时间超过配置的最长等待时间（`red_light_right_turn_stop_duration_sec`），则调用 `FinishStage(false)` 结束当前阶段
* 无其他退出，返回 `StageStatusType::RUNNING`

### CheckTrafficLightNoRightTurnOnRed 函数
作用：检查指定交通信号灯是否禁止右转
* 通过 `traffic_light_id` 获取信号灯信息，若不存在则返回 `false`
* 遍历信号灯的标志信息，若存在 `NO_RIGHT_TURN_ON_RED` 标志，则返回 `true`
* 若未找到上述标志，再遍历子信号灯，若存在右转箭头（`ARROW_RIGHT`），也返回 `true`
* 若均不满足，则返回 `false`

### FinishStage 函数
* 根据 `protected_mode` 决定是否进入巡航阶段
* 若非保护模式，检查车辆速度是否超过阈值
    > 若超过阈值，则直接进入巡航阶段
    >
    > 未超过则进入蠕行（`creep`）阶段，并更新交通灯状态和开始时间

---

## stage_creep.cc
### Process 函数
* 检查场景 not null
* 获取当前场景的上下文对象、提取配置信息，如果 `pipeline_config` 不启用，则 `FinishStage()`
* 遍历所有可行驶的参考线信息，并对每条参考线执行蠕行决策处理
    > 若某条参考线不可行驶（`IsDrivable()` 返回 `false`），则记录错误并跳出循环
    >
    > 对可行驶的参考线调用 `ProcessCreep` 函数进行蠕行决策处理
    >
    > 若处理失败（`ret.ok()` 为 `false`），则记录错误信息并跳出循环
* 执行参考线上的规划任务、检查当前交通灯重叠ID是否为空
* 检查当前交通信号灯是否在参考路径上存在重叠区域，如果未找到重叠区域，则调用 `FinishScenario()` 结束当前场景
* 设置路权 `false`
* 开始蠕行
    > 通过当前时间和蠕行开始时间差值获取已等待时间
    >
    > 计算蠕行停止位置与自车当前位置的距离
    >
    > 若距离小于等于0，则生成固定距离的蠕行速度曲线 -> `GenerateFixedDistanceCreepProfile`
    >
    > 调用 `CheckCreepDone` 判断是否满足结束条件，若满足则调用 `FinishStage()` 结束阶段

---

## stage_intersection_cruise.cc
### Process 函数
* 检查场景not null
* 调用 `ExecuteTaskOnReferenceLine` 在参考线上执行规划任务
* 通过 `CheckDone` 判断当前阶段是否已完成
    > 若阶段完成，调用 `FinishStage()` 结束该阶段
    >
    > 否则返回运行状态（`RUNNING`）

---

## traffic_light_unprotected_right_turn_scenario.cc
### IsTransferable 函数
* 进行条件检查:包含车道跟随指令
* 调用 `IsTransferableOnReferenceLine` 检查当前场景是否可转移
* 检查参考路径上是否存在特定类型的交通标志，并返回相应的重叠信息
    > 若遇到停车标志或让行标志，直接返回 `false`
    > 
    > 若遇到信号灯标志，记录其重叠信息并跳出循环 `traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);`
    >
    > 如果未找到信号灯标志的重叠信息，返回 `false`
* 获取交通信号灯相关的路径重叠信息，并计算自车前缘在参考线上的位置
* 筛选出与交通标志重叠区域距离在 `kTrafficLightGroupingMaxDist` 范围内的交通灯重叠区域，并将它们存储到 `next_traffic_lights` 向量中
```cpp
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  for (const auto& overlap : traffic_light_overlaps) {    // 遍历 traffic_light_overlaps 中的每个重叠区域 overlap
    const double dist = overlap.start_s - traffic_sign_overlap->start_s; // 计算 overlap 起始位置与 traffic_sign_overlap 起始位置的距离 dist
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 若距离绝对值小于等于 2 米，则将该 overlap 添加到 next_traffic_lights 中
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
* 判断当前场景是否为右转场景，并更新交通信号灯重叠ID列表
    > 若traffic_light_scenario为空，则返回false
    >
    > 获取路径在信号灯重叠起点处的转向类型，若不是右转（`RIGHT_TURN`），则返回 `false`
    >
    > 清空当前交通信号灯重叠ID列表
    >
    > 遍历下一个交通信号灯集合 `next_traffic_lights`，将其对象ID添加到列表中

### Enter 函数
判断车辆是否进入“无保护右转通过红绿灯”场景
* 遍历参考线上的重叠信息，查找类型为信号灯（`ReferenceLineInfo::SIGNAL`）的第一个重叠对象
* 检查交通灯是否存在
* 查找与当前交通信号灯在同一位置或组的所有交通信号灯
* 遍历交通信号灯重叠区域，将距离当前信号灯在 `kTrafficLightGroupingMaxDist` 范围内的信号灯加入规划上下文
    > 获取当前信号灯的起始位置 `current_traffic_light_overlap_start_s`
    >
    > 遍历所有信号灯重叠区域，计算与当前信号灯的距离 `dist`
    >
    > 若距离小于等于2米，则将其ID添加到规划状态中，并记录日志