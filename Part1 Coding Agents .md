## 介绍agent之前，主流ai使用

#### 网页chat推荐：

gpt，gemini，claude网页chat使用。网页有深度研究，（尤其是gpt的pro）处理复杂问题。

国产glm，minimax在编码方面都还不错，对话相比上述还有差距，建议仅作为可选项在agent中使用，成本极低

ps：无论如何不要再用豆包。

#### gpt与gemini获取渠道：

[Google Gemini](https://gemini.google.com/app?zh=cn)：学生认证（搜），或淘宝50买一年学生认证号（质保）。日常使用。

[ChatGPT](https://chatgpt.com/)：咸鱼搜“business”，9.9/月。thinking和pro处理gemini做不了的事情。

​       	<img src="C:\Users\a7743\AppData\Roaming\Tencent\TIM\Temp\685735d732bbad576c911514a15783be.jpg" alt="685735d732bbad576c911514a15783be" style="zoom:15%;" />			<img src="C:\Users\a7743\AppData\Roaming\Tencent\TIM\Temp\6cb2e4587063d961fe94bfd52ffc8ed1.jpg" alt="6cb2e4587063d961fe94bfd52ffc8ed1" style="zoom:15%;" />

关于 [Claude](https://claude.com/)：很强，但是基本包封。

#### 使用提示：

节点干净，保证正常访问且不封号。访问不了换节点。

反例<img src="C:\Users\a7743\AppData\Roaming\Typora\typora-user-images\image-20260326180142107.png" alt="image-20260326180142107" style="zoom:50%;" />

应该不用我介绍如何翻墙。

​	资料有clash verge 安装包(win)，推荐机场 [赔钱机场 - 超便宜低价高速机场](https://xn--mes358aby2apfg.com/dashboard)。真有问题私信。

__培训结束后gpt/gemini二选一作为日常使用（请当作一个任务，常用用就知道了）__



## 前置部分结束。1.coding agents介绍

#### 能力：

处理复杂工程（读整个仓库）

且对比AI ide：具有 __直接操作终端的能力__，直接操作电脑，读取运行结果。且生态更丰富。

演示

#### 说明：

**coding agents本身只是一种工具**，需要模型接入。

选择：一般情况，用gemini / gpt直接用对应gemini-cli / codex（见后）。

claude code作为工具本身最先进（我认为）

​	如果使用国产模型 / claude，建议都用claude code。当然本质上说用什么模型都可以。

#### 额外说明，安全：

网络安全：大可不必担心。相比openclaw24/7运行的守护进程，普通agent网络安全问题低。

系统安全：**知道它在做什么**（小心比赛祖传的东西）。平时自用：wsl（仅环境隔离），docker（绝对安全）

​	**不能把它用成黑箱**



## 2.Claude Code部署

今天演示装在wsl里，win的文件会挂载成设备节点，可以直接访问。环境隔离、unix指令

![image-20260326184747743](C:\Users\a7743\AppData\Roaming\Typora\typora-user-images\image-20260326184747743.png)

#### 安装、运行（工具本身）：

0. __(可选)__  推荐一个终端 [Warp: The Agentic Development Environment](https://www.warp.dev/)，资料有win安装包。

1. 安装并进入WSL：安装完成后打开终端(win: win + r 输 cmd)输入`wsl`运行

2. ```bash
   sudo apt update # 更新本地索引
   
   # 安装git
   sudo apt install git -y 
   # 安装nvm(nodejs版本管理工具)
   curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
   source ~/.bashrc
   # 使用nvm安装nodejs长期支持版
   nvm install --lts
   
   # 安装claude code
   npm install -g @anthropic-ai/claude-code
   claude --version # 成功显示版本为安装成功
   claude # 运行claude code, 首次进入选主题颜色等, 到选择登陆方式时两次ctrl+C退出, 进行下方API配置
   
   # 如果安装成功但是claude命令不识别，修改环境变量（已经成功则无视）：
   nano ~/.bashrc
   # 在文件末尾添加 export PATH=$PATH:/你的/目标/路径
   # 通常路径为/home/<用户名>/.nvm/versions/node/v24.x.x/bin
   source ~/.bashrc
   ```

#### API配置（模型接入）：

cc，codex，geminicli都支持直接登陆对应账号 / 使用API接入模型

claude code常用模型：

​	国产 GLM [智谱AI开放平台](https://bigmodel.cn/glm-coding?utm_source=BING&utm_campaign=BING&_channel_track_key=8JOkYuEj&msclkid=27c929b14a341700a3a4407337be4bde) / minimax [MiniMax](https://www.minimaxi.com/)

​	claude：同上文，不推荐官网注册账号（包封）。 API 中转站推荐：[OpenRouter](https://openrouter.ai/) / [仪表板 - NEW CLI](https://foxcode.rjj.cc/dashboard)  / [Kit Coding](https://kitcoding.com/) / [0011.ai](https://0011.ai/)

CC Switch：快速切换claude code API， [Release CC Switch v3.12.2 · farion1231/cc-switch](https://github.com/farion1231/cc-switch/releases/tag/v3.12.2)，安装包有

​	(1) 装在wsl中，修改目录

<img src="C:\Users\a7743\AppData\Roaming\Typora\typora-user-images\image-20260325154907552.png" alt="image-20260325154907552" style="zoom:80%;" />

​	(2) 添加API

​	添加供应商 -> 自定义配置 -> 下划填入api 和 请求地址

​	体验：baseURL：https://kitcoding.com，API：sk-1Jzm8UhcoHxiuf6O1WiK4m1SGyVjexB1oqnP0YALeQs7CNQd

<img src="C:\Users\a7743\AppData\Roaming\Typora\typora-user-images\image-20260325155034198.png" alt="image-20260325155034198" style="zoom:80%;" />

__梯子换成TUN模式__，在wsl中 `claude` 运行claude code

#### codex && gemini-cli

codex：`npm install -g @openai/codex` 

gemini-cli：`npm install -g @google/gemini-cli` 

使用：直接登录对应gpt / gemini账号即可，普通plus账号额度比较大方。



## 3.Claude Code基本操作：

### 基本命令：

​	演示

​	运行前 `claude -r` 回到历史对话。(注意在同一工作目录下)

​	`/memory + 内容`  claude.md中添加记忆：系统级（home/用户名/.claude下） / 项目 / 项目且不传git

​	`/model` 切换模型，左右键选择思考深度

​	`/init` 初始化：自动浏览仓库，生成项目claude.md

​	`/btw + 问题` 运行过程中问问题且不打断

​	`/compact + 指定重点` 手动压缩上下文：上下文用尽前会有百分比提醒，手动压缩。不要等到自动压缩

​	`shift + tab` 切换模式：默认 / 全自动 / plan mode

​	对话框双击ese 回退到之前



## 4.强力工具——武装你的agent

只介绍最必要的，**要装，会强很多**

### everything-claude-code:

github 109k stars，集成了众多自定义Rules、skills、command

[affaan-m/everything-claude-code: The agent harness performance optimization system. Skills, instincts, memory, security, and research-first development for Claude Code, Codex, Opencode, Cursor and beyond.](https://github.com/affaan-m/everything-claude-code)

### skills：

本质来说，skills只是__针对特定场景__写好的提示词.md + 一些供AI调用的脚本。

对话内容自动触发或 进claude /名称 + 对话。

market ：[The Agent Skills Directory](https://skills.sh/)，按需寻找

```bash
# 只推荐最必要的（不要全部复制，每条有单独设置）
# 直接下载复制到/.claude/skills中也能识别到

# find-skills，让AI遇到需求自己找
npx skills add https://github.com/vercel-labs/skills --skill find-skills
# 处理pdf
npx skills add https://github.com/anthropics/skills --skill pdf
# opencv相关
npx skills add https://github.com/mindrally/skills --skill computer-vision-opencv
# 嵌入式开发相关
npx skills add https://github.com/jeffallan/claude-skills --skill embedded-systems
```

### mcp：

标准协议，给ai__操作和调用外部工具、资源__的能力。安装配置比较复杂。前端开发用的比较多。

对话指定触发：eg.“使用context7查找最新文档”

market：[MCP市场 - 收录40,000+ MCP Servers的全球最大平台](https://mcpmarket.cn/)

推荐几个常用的：

__（只装这个就够了，其他按需选择）context7__：访问他们的数据库，查找最新文档

​	 [upstash/context7: Context7 Platform -- Up-to-date code documentation for LLMs and AI code editors](https://github.com/upstash/context7)

（被集成进去了）pal-mcp(原zen-mcp)：让agent自己调用其他AI 讨论或分工。现在原生带了subagnets功能。纯软件方向用的多。

### 提醒：

装更多功能的cc消耗token速度极快。

按需选择中转站 / 国产模型按月订阅coding plan(便宜)  / 换codex（很方便，全局skills自动识别，mcp一键配）

### 更多内容：

推荐一个博主，cc教程做的很详细且即使：[沧海九粟的个人空间-沧海九粟个人主页-哔哩哔哩视频](https://space.bilibili.com/28357052?spm_id_from=333.337.0.0)
