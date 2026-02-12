# 无保护停止
### 由 `swx-opentech` 研究并翻译！
`PullOverScenario`: 靠边停车场景

`StopSignUnprotectedScenario`场景可以在高精地图中有停止标记的路口时停车，观望周边车辆，等待周围车辆驶离后跛行，再快速通过路口。

---

## stage_pre_stop.cc
### Process 函数
* 检查场景not null
* 执行参考线任务 `ExecuteTaskOnReferenceLine(planning_init_point, frame);`
* 获取参考线信息、获取场景上下文、获取当前停止标志的ID `stop_sign_overlap_id`
* 检查当前路径上是否存在指定的停车标志重叠 有则 `FinishSenario`
* 计算自动驾驶车辆（ADC）前缘与停车标志之间的距离 `distance_adc_pass_stop_sign`
* 判断是否合法停车
    ``` cpp
    if (distance_adc_pass_stop_sign <= kPassStopLineBuffer) {
        // 距离小于等于缓冲距离 未越过停止线
        if (CheckADCStop(adc_front_edge_s, current_stop_sign_overlap->start_s)) {
            // CheckADCStop 函数检查车辆是否在指定位置（current_stop_sign_overlap->start_s）合法停车
            return FinishStage();
        }
    } else {
        // passed stop line
        return FinishStage();
    }
  ```
* 没有合法停车 -> 遍历 `watch_vehicles` 容器，将每个车道关联的车辆ID收集到 `watch_vehicle_ids`
    > 将当前车道的所有车辆ID追加到 `watch_vehicle_ids`
    >
    > 构造一个逗号分隔的字符串 `s`，表示当前车道关联的所有车辆 `ID` -> `s = s.empty() ? vehicle_id : s + "," + vehicle_id;`
* 在车辆接近停车标志时，将相关障碍物 `AddWatchVehicle` 监控列表
* 设置阶段状态为 `RUNNING`


### AddWatchVehicle 函数
* 检查 `watch_vehicles` 非空
* 获取障碍物及其枚举值 ， 后检查这些值
* 在地图中查找与给定位置和朝向最匹配的车道、检查感知到的障碍物是否在车道上
* 查找与障碍物车道ID匹配的关联车道、检查障碍物是否与当前停车标志关联的车道相关联
* 检查车辆是否在停止线前有效停车
    > 根据障碍物车道ID获取重叠信息 `over_lap_info`
    >
    > 检查当前车辆是否在指定的停止线之前有效停车 `distance_to_stop_line`
    >
    > 若距离超过配置的最大允许值，则认为停车无效，跳过处理。
* 向指定车道添加观测车辆ID
    > 从`watch_vehicles`中获取当前车道已记录的车辆列表
    >
    > 检查目标车辆ID是否已在列表中，若不存在则添加 `std::find(vehicles.begin(), vehicles.end(), perception_obstacle_id) == vehicles.end()`
    >
    > 更新`watch_vehicles`


### CheckADCStop 函数
检查车辆是否在停止线前有效停车

### FinishStage 函数
* 获取当前场景上下文 `StopSignUnprotectedContext`
* 记录停车开始时间 `stop_start_time`
* 设置下一阶段为 `"STOP_SIGN_UNPROTECTED_STOP"`
* 返回 `StageStatusType::FINISHED`

---

## stage_stop.cc
### Process 函数
* 检查场景not null
* 获取参考线、获取场景上下文，并在参考线上执行规划任务 `ExecuteTaskOnReferenceLine(planning_init_point, frame);`
* 验证停车标志是否存在于参考线上
    > 获取参考线信息 `reference_line_info`、获取停车标志ID `stop_sign_overlap_id`
    > 
    > 调用 `GetOverlapOnReferenceLine` 获取当前停车标志在参考线上的重叠信息
    >
    > 如果未找到该重叠区域（即指针为 nullptr），则调用 `FinishScenario()` 结束当前场景
* 把路口设置为无优先通行权
    > 获取参考线信息 `reference_line_info`、获取路口ID `junction_overlap_id`
    >
    > 调用 `SetJunctionRightOfWay` 函数，将该位置对应的路口设置为无优先通行权（`false`）
* 计算车辆前缘与停止标志起始位置的距离，如果该距离超过允许范围，则调用 `FinishStage()` 结束当前阶段
* 检查等待时间是否满足预设的停车时长要求
    > 获取当前上下文中的停车开始时间 `context->stop_start_time`
    >
    > 计算从停车开始到当前时刻的等待时间 `wait_time`
    >
    > 检查当前时间是否超过预设的停车时长，如果超过则设置 `StageStatusType::RUNNING`
* 检查 `watch_vehicles` 容器是否为空
* 遍历 `watch_vehicles` 容器，收集所有被监控的车辆ID
    > 将 `watch_vehicle.second` 中的所有元素复制到 `watch_vehicle_ids` 中
* 删除 `watch_vehicle_ids` 中相邻的重复元素（需先排序）
* 将被监控的车辆ID传递给 `DECIDER_RULE_BASED_STOP` 任务，用于可视化显示
* 时间检查：
    > 等待时间超过配置的超时时间且监控车辆数≤1，则结束当前阶段 `wait_time > scenario_config.stop_timeout_sec() && watch_vehicle_ids.size() <= 1`
    >
    > 从路径决策中移除不再需要监控的车辆 -> `RemoveWatchVehicle`
    >
    > 设置并返回当前阶段为“运行中”状态

### RemoveWatchVehicle 函数
* 检查场景not null
* 获取当前场景上下文 `context`
* 从 `vehicle.first` 获取关联车道 `ID`，并在 `context->associated_lanes` 中查找与该 `ID` 匹配的车道信息
* 获取与指定车道ID关联的停车标志重叠信息
* 遍历车辆列表 `vehicles`，检查每辆车是否在感知障碍物中存在，并根据距离判断是否将其标记为移除
    > 获取车辆信息 `vehicle_info`
    >
    > 查找指定 `ID` 的感知障碍物，加入待移除列表 `remove_vehicles`
    >
    > 获取障碍物类型，转为字符串；将障碍物的位置信息转换为ENU坐标系下的点
    >
    > 使用 `DistanceXY` 函数计算障碍物与停车标志点之间的水平距离
    >
    > 若距离超过10.0，则记录需移除的障碍物ID到 `remove_vehicles`
* 遍历 `remove_vehicles` ，从 `vehicles` 容器中删除对应ID的障碍物

### FinishStage 函数
* 将当前停车标志的ID标记为已完成，并清空等待障碍物的ID
* 保存当前时间为蠕行阶段的起始时间
* 设置下一阶段为 `"STOP_SIGN_UNPROTECTED_CREEP"`
* 返回阶段完成状态

---

## stage_creep.cc
### Init 函数
初始化场景

### Process 函数
* 检查场景not null
* 获取当前上下文对象 `StopSignUnprotectedContext` 的指针，检查 `pipeline_config_` 是否启用
* 遍历所有可行驶的参考线信息，并对每条参考线执行 `ProcessCreep` 函数进行蠕行决策处理
    > 遍历 `frame` 中的所有参考线信息
    >
    > 检查当前参考线是否可行驶 `!reference_line_info.IsDrivable()`
    >
    > 对可行驶的参考线调用 `ProcessCreep` 函数，若处理失败则记录错误并跳出循环
* 执行参考线上的规划任务并处理结果
* 调用 `GetOverlapOnReferenceLine` 函数，根据 `stop_sign_overlap_id` 查找停车标志重叠区域
* 处理车辆在停止标志前的缓行逻辑
    > 通过 `GetCreepFinishS` 获取车辆应在停止标志前完成缓行的位置
    >
    > 计算当前车辆后轴与缓行停止位置的距离 `distance`
    >
    > 若距离小于等于0（即已到达或超过缓行停止位置），则生成一个固定距离的缓行速度剖面，并更新参考线信息中的速度数据
* 检查是否满足结束当前阶段的条件，如果 `CheckCreepDone` 返回 `true`（表示条件满足），则调用 `FinishStage()`

### GetOverlapStopInfo 函数
* 如果ID不为空，则在参考线上查找对应的停车标志重叠对象
* 若找到该对象，将其结束位置（`end_s`）和ID返回，并返回`true`；否则返回 `false`

### FinishStage 函数
* 设置下一阶段为 `"STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE"`


---


## stage_intersection_cruise.cc
### Process 函数
* 检查场景not null
* 执行参考线任务
* 获取当前场景上下文 `StopSignUnprotectedContext`
* 调用 `CheckDone` 函数判断阶段是否完成
    > 如果阶段已完成（`stage_done` 为真），调用 `FinishStage()` 结束当前阶段
    >
    > 若未完成，则通过 `result.SetStageStatus()` 将阶段状态设为运行中（`RUNNING`）并返回

---


## stop_sign_unprotected_scenario.cc
### GetAssociatedLanes 函数
* 清空当前上下文中的关联车道列表。`context_.associated_lanes.clear();`
* 获取与输入停车标志关联的其他停车标志。 `HDMapUtil::BaseMap().GetStopSignAssociatedStopSigns`
* 遍历这些关联停车标志，提取其重叠的车道ID
* 若存在，则将该车道及其重叠信息添加到上下文的关联车道列表中，并记录调试日志。
```cpp
// 遍历当前车道（lane）上的所有停车标志重叠信息（stop_sign_overlap），
// 检查每个停车标志是否与指定停车标志（stop_sign）存在重叠。若存在，则将该车道与停车标志的关联信息存入上下文（context_）中
for (const auto& stop_sign_overlap : lane->stop_signs()) {
// 获取当前停车标志与目标停车标志的重叠信息
auto over_lap_info =
    stop_sign_overlap->GetObjectOverlapInfo(stop_sign.get()->id());
if (over_lap_info != nullptr) {
    // 若重叠信息存在，记录车道与停车标志的关联关系
    context_.associated_lanes.push_back(
        std::make_pair(lane, stop_sign_overlap));
    ADEBUG << "stop_sign: " << stop_sign_info.id().id()
            << "; associated_lane: " << lane_id.id()
            << "; associated_stop_sign: " << stop_sign.get()->id().id();
}
```

## IsTransferable 函数
* 检查frame中的规划命令是否包含车道跟随指令，若无则返回false
* 若other_scenario为空或参考线信息为空，则返回false
* 取第一条参考线信息及其首个遇到的重叠区域  `first_encountered_overlaps`
* 遍历first_encountered_overlaps容器，检查其中的重叠类型:
    > 如果遇到信号灯（`SIGNAL`）或让行标志（`YIELD_SIGN`），直接返回false
    >
    > 如果遇到停车标志（`STOP_SIGN`），将对应的重叠信息保存到 `stop_sign_overlap` 指针中，并跳出循环
* 计算车辆前缘到停车标志的距离 `adc_distance_to_stop_sign`
* 判断该距离是否在合理范围内（大于0且小于等于配置的距离），决定是否进入停车标志场景


## Enter 函数
* 查找最近的停车标志：遍历参考线上的重叠区域，找到第一个停车标志的ID。
* 检查当前停车标志重叠ID是否为空
* 获取参考线上的所有停车标志重叠区域（stop_sign_overlaps） ->  查找与当前停车标志 ID（`current_stop_sign_overlap_id`）匹配的重叠区域
* 更新规划上下文中的停车标志信息