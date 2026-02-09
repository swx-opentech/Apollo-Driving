# 车道跟随
### 由 `swx-opentech` 研究并翻译！

---
## lane_follow_stage.cc
### Process 函数
* 检查 `frame` 对象中的参考线信息是否为空 `if (!frame->reference_line_info().empty()) {...}`
* 遍历 `frame` 对象中所有可变的参考线信息
    > 使用 `count` 计数，参考线信息的数量 `if (count++ == frame->mutable_reference_line_info()->size()) {...}` 
    >
    > 判断是否存在可行驶的参考线，如果存在，则将当前参考线设置为不可行驶状态 `if (has_drivable_reference_line) {...}`
    >
    > 生成一条可行的行驶轨迹 `PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);`
    >
* 检查当前参考线是否存在错误 `if (!result.HasError()) {...}`
    > 没有错误，调用 `IsChangeLanePath()` 方法检查是否处于变道状态
    ``` cpp
    if (!reference_line_info.IsChangeLanePath()) {
        // 如果不是变道状态，则将当前参考线设置为可行驶状态
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true; // 表示存在可行驶的参考线
        continue;
    }
    ```
    >
    > 否则，检查这条参考线路的代价（`Cost`）是否小于不进行车道变更的代价（`kStraightForwardLineCost`）
    ``` cpp
    if (reference_line_info.Cost() < kStraightForwardLineCost) {
        // 如果 小于，则认为路径和速度优化成功，设置该参考线为可行驶状态
        has_drivable_reference_line = true;
        reference_line_info.SetDrivable(true);
    } else {
        // 否则，将当前参考线设置为不可行驶状态
        reference_line_info.SetDrivable(false);
        ADEBUG << "\tlane change failed";
    }
    ```
* 遍历完成，返回运行中的状态 (`StageStatusType::RUNNING` or `StageStatusType::ERROR`)

---
### PlanOnReferenceLine 函数
生成每一个参考线对应的代价，用于后续判断
* 判断是否为变道路径 `if (!reference_line_info->IsChangeLanePath()) {...}`，否则增加直行成本
* 遍历任务列表中的每一个任务
    > 获取系统时间 `const double start_timestamp = Clock::NowInSeconds();`...
    >
    > 调用task->Execute()执行任务，并将结果状态设置到ret对象中 `ret.SetTaskStatus(task->Execute(frame, reference_line_info));`
    >
    > 获取系统时间，并计算任务执行时间打印
    >
    > 检查任务执行结果，如果发生错误，则返回错误状态 `if (ret.IsTaskError()) {...}`
* 设置轨迹类型 正常模式；如果任务错误，则调用 `fallback_task`_ 的 `Execute` 方法，传入当前帧 `frame` 和参考线信息 `reference_line_info` 进行回退处理
* 调用 `CombinePathAndSpeedProfile` 方法尝试将规划起点的时间、路径点以及轨迹进行整合。
* 遍历路径决策中的所有障碍物
    > 获取障碍物信息 `const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()`
    >
    > 检查障碍物有纵向停车决策 停车原因代码为 `STOP_REASON_DESTINATION `
    ```cpp
    if (obstacle->LongitudinalDecision().has_stop()
            && obstacle->LongitudinalDecision().stop().reason_code() == STOP_REASON_DESTINATION) {
            // 调用 GetStopSL 函数计算该障碍物在参考线上的停车位置（SL坐标），并更新 dest_stop_s 为对应的 s 值。
            SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(), reference_line_info->reference_line());
            dest_stop_s = dest_sl.s();
        }
    ```
* 遍历路径上的障碍物
    > 检查障碍物是否有停车决策（`has_stop()`）
    >
    > 如果目标停车距离 `dest_stop_s` 小于 0，则直接设置 `add_stop_obstacle_cost` 为 `true`
    > 否则，通过 `GetStopSL` 获取障碍物的停车位置 `stop_sl`，若该位置小于目标停车距离且与自车后边界距离小于 `20` 米，则也设置 为 `true`
    >
    > 增加代价为真，向 `reference_line_info` 添加一个静态障碍物代价 `kReferenceLineStaticObsCost`，用于路径规划中的成本计算。
* 轨迹检查
* 赋值相应结果，返回函数
---
### GetStopSL 函数
将停车决策中的停车点坐标转换为SL坐标系下的坐标，调用reference_line的XYToSL方法，将stop_decision中的停车点（XY坐标）转换为SL坐标，并存入sl_point，返回转换后的SL坐标点。