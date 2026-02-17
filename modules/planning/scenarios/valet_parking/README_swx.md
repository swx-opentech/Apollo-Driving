# 代客泊车‌
### 由 `swx-opentech` 研究并翻译！

`ValetParkingScenario`可以在停车区域泊入指定的车位。

---
## stage_approaching_parking_spot.cc
### Init 函数
函数的作用是初始化场景，并复制上下文到临时变量中。

### Process 函数
* 检查场景 not null、获取参考线
* 检查目标停车点 ID 是否为空
* 设置目标停车位ID；设置预停标志位；设置预停位置点
* 遍历参考线集合，对每条参考线上的路径决策进行处理
    > 获取当前参考线的路径决策对象 `reference_line.path_decision();`
    >
    > 若路径决策为空/未找到目标障碍物则跳过
    >
    > 创建一个"忽略"决策并应用到目标障碍物上 `AddLongitudinalDecision("ignore-dest-in-valet-parking",`
* 基于参考线执行规划任务
* 获取预停标志和预停位置点
* 调用 `CheckADCStop` 函数判断车辆是否已停止，若满足条件则进入下一阶段（`VALET_PARKING_PARKING`）并返回完成状态
* 若无异常且未满足停止条件，则返回运行中状态

### CheckADCStop 函数
检查自动驾驶车辆（ADC）是否在停车点附近正确停止
* 获取场景上下文允许的最大速度和停止距离
* 获取当前速度，判断车辆当前速度是否低于最大允许停止速度，若超过则返回 `false`
* 计算车辆前缘与停车线的距离，若距离过大（超过配置的最大有效停车距离），则认为未正确停车，返回 `false`

---
## stage_parking.cc
### Process 函数
核心是完成停车场景下的开放空间轨迹规划与状态管理
* 将帧（`frame`）标记为处于开放空间轨迹状态
* 从场景上下文获取目标停车位ID并写入帧信息
* 调用 `ExecuteTaskOnOpenSpace` 处理开放空间规划逻辑
* 任务执行出错，记录错误日志并返回错误状态；否则返回运行中状态

---

## valet_parking_scenario.cc
### IsTransferable 函数
* 前置工作：检测可用停车位并获取相关指令信息 -> 获取配置参数、检查规划命令
* 检查 `frame.reference_line_info()` 是否为空
* 检查泊车指令：当存在泊车指令且该指令包含有效的停车位ID时，将目标停车位ID赋值给变量 `target_parking_spot_id`
* 若无泊车指令，根据路径终点和车辆位置判断是否进入泊车范围 ↓
* 前置检查：检查 `routing_end` 是否为空指针，不是则可设置终点位置坐标
* 计算自车（ADC）前缘到目标点的距离，用于路径决策判断，并判断车辆距离终点的距离 `adc_distance_to_dest` 是否超过停车点启动范围 `parking_spot_range_to_start`，超过则不满足泊车条件
* 寻找停车位：
    > 通过高精地图接口获取车辆终点附近的停车位列表 `hdmap_->GetParkingSpaces(...)`
    >
    > 首先收集所有障碍物位置信息（看代码注释）
    >
    > 检查每个停车位是否符合条件（看代码注释）
    >
    > 如果有有效停车位，选择距离当前位置最近的
* 检查目标停车点ID是否为空
* 调用 `SearchTargetParkingSpotOnPath` 函数在路径上查找指定 ID 的停车位。若未找到，则记录日志并返回 `false`
* 若找到停车位，调用 `CheckDistanceToParkingSpot` 检查车辆与停车位的距离是否超过预设范围。若距离过远，则记录日志并返回 `false`
* 若上述条件均满足，将目标停车位 ID 保存到上下文变量 `context_.target_parking_spot_id` 中，并返回 `true`


### CheckDistanceToParkingSpot 函数
是检查车辆是否接近停车点
* 通过 `parking_space_overlap.object_id` 从高精地图中获取目标停车位的几何信息
* 取停车位四个角点的平均值作为中心点 -> `Vec2d center_point = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;`
* 停车点中心和车辆位置分别投影到附近路径上，得到对应的路径参数`s`
* 比较车辆与停车点在路径上的距离是否小于设定阈值 `parking_start_range`，若满足则返回`true`，否则返回`false`
