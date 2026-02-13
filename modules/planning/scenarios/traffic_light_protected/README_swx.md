# 有路权保护的红绿灯
### 由 `swx-opentech` 研究并翻译！

`TrafficLightProtectedScenario` 是有路权保护的红绿灯场景，在该场景下可以实现在红绿灯路口前红灯停车，绿灯通过路口。

---

## stage_approach.cc
### Process 函数
* 检查场景 not null
* 执行交通灯保护场景下的规划任务
* 访问交通信号灯重叠区域的ID列表
    > 获取参考线上与交通信号灯相关的重叠区域信息 `current_traffic_light_overlap` -> 没有跳过本次循环
    >
    > 设置路权状态 `SetJunctionRightOfWay`
    >
    > 获取自车前缘位置 `adc_front_edge_s`，并计算其到停车线的距离
    >
    > 通过 `frame->GetSignal` 获取指定信号灯的颜色
    >
    > 判断车辆与停止线的距离是否超过最大有效停车距离 -> break
    >
    > 检查交通信号灯的颜色 -> 非 `rafficLight::GREEN` 时 `traffic_light_all_done` false
* `traffic_light_all_done` 条件为真时，调用 `FinishStage()` 函数并返回其结果
* 否则，设置 `StageStatusType::RUNNING`

### FinishStage 函数
* 获取当前上下文 `TrafficLightProtectedContext`
* 清除已完成的交通信号灯重叠区域 ID
* 将当前上下文中的交通信号灯重叠区域ID添加到已完成列表中
* 设置下一阶段为 `"TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE"`

---
## stage_intersection_cruise.cc
### Process 函数
* 调用`ExecuteTaskOnReferenceLine`在参考线上执行规划任务
* 检查阶段是否完成
    > 完成 -> `FinishStage()` -> `FinishScenario()`
    >
    > 未完成 -> `StageStatusType::RUNNING`

---

## traffic_light_protected_scenario.cc
### Init 函数
初始化场景

### IsTransferable 函数
* 判断当前场景是否满足继续执行的条件
* 遍历路径上的重叠信息，检查是否存在停车标志、让行标志或信号灯
* 获取交通信号灯相关的路径重叠信息，并计算自车前缘在参考线上的位置
* 查找与第一个遇到的交通信号灯属于同一组的所有交通信号灯
```cpp
  // 查找与第一个遇到的交通信号灯属于同一组的所有交通信号灯
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  for (const auto& overlap : traffic_light_overlaps) {
    // 遍历 traffic_light_overlaps 中的所有交通信号灯重叠区域
    const double dist = overlap.start_s - traffic_sign_overlap->start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      // 距离小于等于阈值 kTrafficLightGroupingMaxDist（2米），则将其加入 next_traffic_lights 向量
      next_traffic_lights.push_back(overlap);
    }
  }
```
* 判断车辆是否进入交通灯场景
    > 遍历所有前方交通灯（next_traffic_lights）
    >
    > 计算车辆与交通灯的距离，若距离不在有效范围内则跳过(不进入交通灯场景)。
    >
    > 检查交通灯颜色，若为红色、黄色或未知，则标记为交通灯场景
    >
    > 若未检测到需响应的交通灯，返回 false
* 更新当前交通灯的重叠ID列表