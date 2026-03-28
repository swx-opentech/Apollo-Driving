# 发摄像头



# 传统CV简介

#### 介绍：

​	OpenCV是什么？基于数学(矩阵)运算 的 图像处理算法 **库**（如边缘检测、色域转换等）。

​	传统CV：处理 **规则物体识别**（圆，矩形等），性能要求小（相对的，单片机跑不了）实时性高。

​		25-E自瞄，25-C单目测距，24-E机械臂下棋，**23-E（靶纸、激光点）**

​	对比yolo，处理复杂问题（数字，人脸等），性能要求高。

​		21送药小车。电赛用模型(？)

#### 开发语言选择：

​	python：语法简洁，库生态丰富，运行方便

​	c++：**快（真的很快）**，大家学过C

​	以python为例。语法简单（不用担心，用着用着就会了），要了解一下面向对象部分（直接类比int）。



# 环境配置（装python && cv）

#### 使用uv版本管理，装uv：

​	linux：`curl -LsSf https://astral.sh/uv/install.sh | sh`

​	win：`powershell -c "irm https://astral.sh/uv/install.ps1 | iex"`

​	重启终端，`uv --version` 确认安装成功

​	`uv python install 3.12` 安装python

#### 基于uv，一个传统CV项目的工作流：

```bash
mkdir 你的项目名称 && cd 你的项目名称	# 创建并进入项目文件夹
uv init		# uv初始化项目，会自动创建 pyproject.toml 文件
uv add opencv-python numpy		# 为该项目安装 OpenCV 和 NumPy

# 进行开发

uv run xx.py 	# 运行

# 部署 / 团队开发 同步
uv sync
```

麻烦，针对每个项目。复杂（依赖多、手动编译、古早库等等）方便管理 && 同步。良好的工程习惯（部署反例）

​	注释依赖声明（资料里所有例程都有依赖声明，直接 `uv run 对应脚本.py` 即可）

或者全局装个cv测试用。



# 终于，我们可以来写点东西了

#### 核心逻辑：当成手册

​	本质来说，它是一个库，只要熟悉功能调用即可。

​	底层实现：把图像存成 NumPy 数组（二维或三维矩阵），对矩阵进行数学运算实现其他所有功能。了解这些就够了，剩下的有兴趣。

​	关于它怎么用，举个最熟悉的例子：stm32。`HAL_UART_init, Receive...`，底层...

#### 常用函数手册（基本介绍）：

#### 1. 基础 I/O 与显示。视频处理基本工作流

- **`cv2.VideoCapture(index)`**: 初始化编号index的摄像头对象。**返回实例对象**（调用摄像头内部方法）
  - `cap = cv2.VideoCapture(0) `
  - `cap.set(cv2.CAP_PROP_FRAME_WIDTH, xx)` 设置宽度
  - `cap.set(cv2.CAP_PROP_FRAME_HEIGHT, xx)` 设置高度
  - `cap.set(cv2.CAP_PROP_FPS, xx)` 设置帧率
  - `ret, frame = cap.read()` 调用方法读取图像。ret是bool是否成功读取，**frame是实际数据**
  - `cap.release()` 释放资源

- **`cv2.imread(path)`**: 读入图片文件。**返回 NumPy 数组**
- **`cv2.imshow(name, img)`**: 在名为 `name` 的窗口显示图像
  - `cv2.imshow("Camera_View", img)` 在名为 Camera_View 的窗口显示 img 图像。

- **`cv2.waitKey(delay)`**: 等待按键。用于循环显示
  - `key = cv2.waitKey(1) & 0xFF` 等待按键，存到key里（堵塞1ms）
- **`cv2.createTrackbar(trackbarName, windowName, value, count, onChange)` **:创建滑块调参

#### 2. 图像预处理（变换、滤波、二值化）

- **`cv2.cvtColor(img, code)`**: 色彩空间转换。图像本身不变，计算机内部换一种描述方式。
  - 常用code：`cv2.COLOR_BGR2GRAY`（灰度）、`cv2.COLOR_BGR2HSV`（HSV 色域）。
- **`cv2.GaussianBlur(src, ksize, sigmaX)`**: 高斯模糊滤波。还有均值、中值等滤波等。原始图像降噪。
  - `src`: 源图像。
  - `ksize`: 权重区域的大小，如(3, 3)（奇数），区域越大越模糊。
  - `sigmaX`: 高斯核在 x 方向的标准差。通常0即可，越大越模糊。

- **`cv2.threshold(src, thresh, maxval, type)`**: 二值化。将灰度图按照规定阈值分割为纯白/黑。
  - `src`: 源图像，灰度图。
  - `thresh`: 阈值。像素值大于此值时，按 `type` 执行。
  - `maxval`: 最大值。通常设为 255（纯白）。
  - `type`: 操作类型。`cv2.THRESH_BINARY` :大于阈值变白，其余变黑。

- **`cv2.adaptiveThreshold`**: 自适应二值化函数。按区域大小自动确定阈值，处理光照不均的情况。
- **`cv2.inRange(img, lower, upper)`**:区域提取二值化。
  - `lower, upper`为色彩空间的参数：e.g. `lower = [H_min, S_min, V_min]` ，实现颜色分割。

- **`cv2.morphologyEx()`**:形态学运算，膨胀腐蚀，开、闭等。修饰二值化后的图像。

#### 3. 特征提取与绘制

- **`cv2.Canny(img, threshold1, threshold2)`**: 边缘检测canny算子。
  - `img`: 输入图像，必须是 8 位单通道 **灰度图**。
  - `threshold1` (低阈值): 像素梯度低于此值一定不是边缘。
  - `threshold2` (高阈值): 像素梯度高于此值一定是边缘。
    - 介于两者之间的像素认为是弱边缘，弱边缘与强边缘相连才会被确认为边缘。
  - 返回值为二值图，白色为边缘，黑色为非边缘
- **`contours, _ = cv2.findContours(img, mode, method)`**: 寻找轮廓，用于提取物体形状。
  - `image`: 输入图像
  - `mode`: 检测模式，常用 `cv2.RETR_EXTERNAL` 只检测最外层轮廓。识别时忽略内部杂点。
  - `method`: 近似方法，常用`cv2.CHAIN_APPROX_SIMPLE`压缩水平、垂直和对角线方向的元素(只留顶点)
  - 返回 **`contours`**: 点集列表，每个元素都是 (N, 1, 2) 的 NumPy 数组，代表一条独立的轮廓。第二个返回值为轮廓嵌套关系，一般不用。
- **`approx = cv2.approxPolyDP(curve, epsilon, closed)`**: 多边形拟合，将复杂轮廓简化为四边形。
  - `curve`: 输入的轮廓点集。
  - `epsilon`: 原轮廓到近似多边形边界的最大允许距离(0.x * 周长)。越小越精细，顶点越多；越大越粗糙，顶点越小。
  - `closed`: 布尔值。如果为 True，则近似曲线是封闭的。
  - 返回值同上，列表长度3 -> 三角形，4 -> 矩形，更多 -> 圆 ，包含轮廓嵌套维度 和 顶点坐标。
- **绘制函数**: `cv2.line`, `cv2.rectangle`, `cv2.circle`, `cv2.putText`等等。用于在图像上标注识别结果。

#### 4. 其他

* 几何分析、透视变换、可视化、交互……

今天只是简单介绍，只靠讲一定是讲不完的，边用边学， 用一次就知道有什么功能了



#### 模块封装：

**逻辑与 I/O 解耦 **：

​	应用层：负责初始化硬件（摄像头、串口）、循环读取图像帧，并负责最终结果的下发。

​	处理层：CV 模块，通常封装成纯函数即可。不包含视频采集。接收一个图像，并返回处理后的特征数据。

一般情况模块返回像素坐标，左上角第一个像素是 `(0, 0)`，行列均向右下递增

​	此处注意：`roi = frame[y1:y2, x1:x2]` 截取图像时，实际坐标不会变化（去掉的还在）。



# 任务：

* GPT / Gemini账号日常使用
* claude code插件
* 完成23-e视觉部分：激光笔，矩形框









### **能看懂 且 能调试**

<img src="C:\Users\a7743\AppData\Roaming\Typora\typora-user-images\image-20260326214350313.png" alt="image-20260326214350313" style="zoom:200%;" />