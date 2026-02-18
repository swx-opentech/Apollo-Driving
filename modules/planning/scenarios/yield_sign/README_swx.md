# 让行标志
### 由 `swx-opentech` 研究并翻译！

`YieldSignScenario`场景可以在有让行标记的场景减速观望，然后慢速通过。

---
## stage_approach.cc
停让行标记前停车避让运动障碍物
### Process 函数
* 检查场景 not null
* 获取参考线、复制上下文到临时变量中，检查参考线非空
* 遍历参考线集合，对每条参考线上的路径决策进行处理
    > 获取当前路径上的所有让行标志重叠区域`GetOverlapOnReferenceLine`，检查重叠区域非空
    >
    > 设置优先通行权 `reference_line_info.SetJunctionRightOfWay(...)`
    >
    > 判断自车是否已通过停止线，若通过则结束当前阶段
    >
    > 若未通过停止线，则检查自车与停止线的距离是否足够近
* 当距离足够近时，进一步检测是否有障碍物影响通行
    > 获取障碍物ID和类型名称 `obstacle_id` / `obstacle_type_name`
    >
    > 若障碍物是虚拟的（IsVirtual()为真）或边界信息为空，则跳过该障碍物
    >
    > 若障碍物的最小时间超过阈值（6秒）则忽略
    >
    > 计算障碍物在ST图中左下角点与右下角点的s坐标差值，存储为 `obstacle_traveled_s`
    >
    > 过滤掉满足特定条件的障碍物，避免对其进行进一步处理
    ```cpp
    if (obstacle_traveled_s < kepsilon && // 障碍物在参考线上且正在向ADC移动
            obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT && // 障碍物的最小时间（min_t）小于阈值 kIgnoreMaxSTMinT（0.1秒）
            obstacle->reference_line_st_boundary().min_s() > kIgnoreMinSTMinS) { // 障碍物的最小距离（min_s）大于阈值 kIgnoreMinSTMinS（15米）
          continue;
        }
    ```
    >
    > 将当前障碍物的ID添加到规划状态中的让行标志等待列表中
* 如果 `yield_sign_done` 为真，则调用 `FinishStage()` 函数结束阶段 
* 否则，返回运行中状态

### FinishStage 函数
* 清空已完成的让行标志ID，并将当前场景中的让行标志ID添加到已完成列表中
* 清除等待障碍物的记录
* 将下一阶段设置为 `YIELD_SIGN_CREEP`，并返回阶段完成状态    

---

## stage_creep.cc
### Process 函数
* 检查场景 not null
* 获取参考线、复制上下文到临时变量中，检查->管道未启用，直接结束场景
* 遍历所有可行驶的参考线信息，并对每条参考线执行蠕行决策处理。
    > 遍历 `frame` 中的所有参考线信息
    >
    > 若某条参考线不可行驶`!reference_line_info.IsDrivable()`，则记录错误并跳出循环
    >
    > 对可行驶的参考线调用 `ProcessCreep` 函数进行蠕行决策处理，若处理失败则记录错误并跳出循环
* 执行参考线上的任务/获取当前参考线上的可识别对象信息
* 在参考线上查找指定的让行标志重叠区域`GetOverlapOnReferenceLine`，如果未找到该重叠区域，则结束当前场景
* 处理车辆在让行标志处的行驶逻辑
    > 通过 `SetJunctionRightOfWay` 将让行标志起点标记为无优先通行权
    >
    > 获取当前时间与开始蠕行时间的差值，并判断是否超时
    >
    > 调用 `GetCreepFinishS` 计算蠕行结束位置，若车辆已到达或超过该位置，则生成固定距离的蠕行速度曲线->`GenerateFixedDistanceCreepProfile`
* 判断是否完成让行操作并决定下一阶段状态
    > 调用 `CheckCreepDone` 检查让行条件是否满足
    >
    > 若满足，则调用 `FinishStage()` 结束当前阶段
    >
    > 否则，设置阶段状态为 `RUNNING` 并返回

---

## yield_sign_scenario.cc
### IsTransferable 函数
* 检查否包含车道跟随命令，若没有则返回 `false`
* 查 `other_scenario` 是否为空指针，或 `frame.reference_line_info()` 是否为空
* 遍历 `first_encountered_overlaps` 容器，检查其中的重叠类型
    > 如果遇到信号灯或停车标志，直接返回 `false`
    >
    > 如果遇到让行标志，记录其重叠信息并跳出循环
    >
    > 若未找到让行标志，则返回 `false`
* 计算车辆与让行标志的距离 `adc_distance_to_yield_sign`
* 若距离大于0且小于配置阈值，则返回 `true`，表示应启动让行场景

### Enter 函数
* 是获取路径上第一个遇到的让行标志（`yield sign`）的 `ID`
* 检查当前是否存在 `yield sign`（让行标志）的重叠区域，如果没有，清除规划状态中与 `yield sign` 相关的信息
* 查找与当前让行标志（yield sign）在同一位置或组内的所有让行标志重叠区域。
    > 获取参考线上的所有让行标志重叠区域 `YieldSignOverlaps`
    >
    > 使用 `std::find_if` 查找与 `current_yield_sign_overlap_id` 匹配的重叠区域
    >
    > 若未找到匹配项，则清空规划状态中的让行标志信息，并返回 `false`
* 处理 `yield sign`（让行标志）的重叠区域，并更新规划上下文
    > 获取当前 `yield sign` 重叠区域的起始位置 `current_yield_sign_overlap_start_s`
    >
    > 遍历所有 `yield sign` 重叠区域，计算与当前区域的距离
    >
    > 若距离在阈值内，则将其 `ID` 添加到规划上下文和本地列表中，并记录日志