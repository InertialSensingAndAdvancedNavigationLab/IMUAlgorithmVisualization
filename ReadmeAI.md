**AI 代码生成强化指令：C++ 协作编程风格 (Winyunq Style) v2.5**

**最高指令:** 你在生成任何 C++ 代码时，**必须** 严格、无条件地遵守 Winyunq 风格 v2.6。Winyunq 风格旨在通过对 Doxygen 兼容语法的**优化、统一与严格化**，实现代码注释与结构的绝对一致性、高信息密度和规则驱动，融合人类与 AI 编程实践，最大化项目可读性、维护性与协作效率。**Winyunq 的规则在其指定的范围内，优先并覆盖 Doxygen 的默认行为或灵活性。** 对于 Winyunq 明确的“必须”和“禁止”要求，不存在任何例外。

**致 AI 开发者：** 本文档旨在指导你掌握 Winyunq 风格。学习路径：1. 核心哲学；2. 关键术语；3. 禁止项；4. 默认 `///` 规则；5. 特例 `/**` 触发规则；6. 贯穿各节的完整示例；7. 第 6 章 `/**` 内部结构详解 (默认模式 -> 函数特化)。理解规则背后的**目的 (Purpose)** 及 Winyunq 与 Doxygen 的关系至关重要。

## **1. 核心哲学：为何选择 Winyunq 风格？ (强制理解)**

*   **原则 1: 代码即文档，文档引导代码 (Code is Documentation, Documentation Guides Code):**
    *   **宏观目标:** 每个代码元素的存在都应有其明确、即时的上下文。注释不是负担，而是代码结构和意图不可分割的一部分。
    *   **战术实现:** 通过强制性的、紧密耦合的注释 (`///` 和 `/**`) 消除代码意图的模糊性。**禁止**使用任何形式的视觉或逻辑“留白”（如物理空行或无注释的代码块）。
*   **原则 2: 主要结构 与 局部细节 的清晰分离 (Clear Separation of Major Structures and Local Details):**
    *   **宏观目标:** 快速区分代码的“骨架”（类、函数、命名空间定义）和“血肉”（实现细节、局部逻辑），优化宏观理解与微观分析效率。
    *   **战术实现:** 使用两种视觉和语义上截然不同的注释风格：`/** ... **/` (主要结构定义) 和 `///` (所有其他元素)。
*   **原则 3: 绝对一致性 和 认知负荷 最小化 (Minimizing Cognitive Load through Absolute Consistency):**
    *   **宏观目标:** 风格的严格性旨在**解放**认知资源。掌握规则后，开发者（人或 AI）无需猜测或选择风格，可专注于问题本身。
    *   **战术实现:** 对格式、命名和注释应用**强制性、无歧义**的规则。特别是 `/** ... **/` 的严格对齐，旨在创建**视觉上可快速扫描**的信息列。
*   **原则 4: 信息密度最大化 (Maximizing Information Density):**
    *   **宏观目标:** 在有限的文本空间内传递最多的有效信息。简洁规则 + 完整示例 = 高效学习。
    *   **战术实现:** 规则描述精炼，示例完整且直接对应规则，并通过“目的”阐释原理。

## **2. 关键术语 (强制学习)**

*   **声明 (Declaration):** 指仅提供接口、引入名称和类型，但**不**包含完整实现或分配存储的代码。例如：函数原型 (`void func();`)，类前向声明 (`class Widget;`)，`extern` 变量 (`extern int count;`)，类/结构体内的成员函数声明 (`void method();`)。**必须**使用 `///` 注释。
*   **定义 (Definition):** 指提供**完整实现**或分配存储的代码。例如：带 `{}` 函数体的函数 (`void func() { /* ... */ }`)，带 `{}` 的类/结构体/枚举/命名空间 (`class Widget { /*...*/ };`)，已初始化的全局/静态变量 (`int count = 0;`)。**特定主要结构的定义必须**使用 `/** ... **/` 注释（见第 4 节）。
*   **主体 `{}` 块:** 指 `class`, `struct`, `enum`, `namespace`, 函数定义中包含其主要内容或实现的 `{}`。**注意：** 控制流语句（`if {}`, `for {}`）或 Lambda 表达式 (`[]{}`) 的 `{}` **不是**主体块，其内部适用 `///` 规则。
*   **WinyunqDoxygen Block:** 特指本风格中使用的 `/** ... **/` 块注释。其格式严格，**禁止**与标准或其他 Doxygen 风格混淆。**必须**顶格。
*   **基准列:** 在 `/** ... **/` 块内用于对齐文本的固定列号。Winyunq 使用三个基准列（详见 6.4 节）。

## **3. 禁止项 (强制规则)**

*   **3.1 禁止非 Winyunq 注释:**
    *   **绝对禁止**使用行尾注释 (`// comment`)。
        *   **目的 (Purpose):** 消除视觉噪音，确保 `///` 和 `/**` 是唯一的注释来源。
    *   **绝对禁止**在结束花括号 `}` 之后的同一行或后续行添加任何注释或文本。
        *   **目的 (Purpose):** 保持块结束符的绝对清晰。
    *   **绝对禁止**使用 Doxygen 的其他行注释风格 (`//!`, `///<`) 或任何非 Winyunq 风格的块注释。
        *   **目的 (Purpose):** 强制统一性。
*   **3.2 禁止物理空行:**
    *   **绝对禁止**在代码块之间或任何 `{}` 内部（WinyunqDoxygen Block 内特定分隔符除外）使用物理空行进行视觉分隔。
        *   **目的 (Purpose):** 核心原则！`///` 和 `/**` 是唯一的、主动携带信息的视觉和逻辑分隔符。空行引入歧义。

## **4. `///` - 默认的本地上下文提供者 (强制基础规则)**

*   **4.1 核心作用:** 为**所有声明**，以及**定义体内部**的语句、成员等提供即时、简洁的上下文。这是 Winyunq 风格的**默认**注释方式。凡是未加以说明的地方，均**默认**使用`///`
*   **4.2 强制应用场景:**
    *   预处理指令 (`#include`, `#pragma once` 等)。
    *   **所有声明:** 函数原型 (`/// @brief`)，类/结构体前向声明 (`/// @brief`)，外部变量声明 (`extern`)，`using` 声明/指令, `typedef` 等。
    *   **类/结构体/枚举 定义体 `{}` 内部:** 访问控制符 (`public:`), 成员变量声明, 成员函数**声明** (`/// @brief`)，枚举值。
    *   **函数 定义体 `{}` 内部:** 局部变量声明, 控制流语句 (`if`, `for`, `while`, `switch` 等的条件和主体语句), 可执行语句 (函数调用, 赋值等), `return` 语句。
*   **4.3 格式规则 (强制):**
    *   **位置:** 必须在被描述代码行的**正上方**。
    *   **间距:** 与代码行之间**禁止**任何空行。
    *   **缩进:** 必须与代码行**完全相同的缩进**。
        *   **目的 (Purpose for 上方/无空行/同缩进):** 创建代码与其解释之间最紧密的视觉和逻辑绑定。
    *   **内容:**
        *   **必须简洁，且必须为单行。** 任何需要多行解释或详细说明的情况，都应将信息整合到包含该元素的**定义**的 `/**` 文档中。
        *   函数**声明**(原型) **必须**使用 `/// @brief [描述]`，且此 `@brief` **必须是单行**。
        *   **绝对禁止**在 `///` 注释中使用 `@details`, `@param`, `@return`, `@note`, `@warning`, `@tparam`, `@extends` 等**任何块级 Doxygen 标签**。
            *   **目的 (Purpose for 单行强制 & 禁止块标签):** 强制 `///` 只承担最核心、最本地的上下文描述。确保极致简洁，将结构化、详细文档责任完全推向 `/**` 定义块。维护 `///` 与 `/**` 的严格功能分离。
        *   **禁止**在仅包含 `{` 或 `}` 的行上方使用 `///`。
*   **4.4 示例 (完整且符合规则):**

*   **示例 4.4.1: 文件级与声明 `///`**
```cpp
/// 使用 #pragma once 防止头文件重复包含
#pragma once
/// 包含标准库字符串
#include <string>
/// @brief 日志记录器类 (前向声明)
class Logger;
/// @brief 全局日志记录器实例 (外部链接)
extern Logger* g_globalLogger;
/// @brief 初始化日志系统的函数原型
bool initializeLogging();
```
*   **示例 4.4.2: 类定义内部的 `///`**
```cpp
class UserSession
{
/// 公共接口
public:
    /// @brief 用户登录 (静态成员函数声明)
    static bool login(const std::string& username, const std::string& password);
    /// @brief 获取当前会话的用户 ID (成员函数声明)
    int getUserId() const;
    /// @brief 检查会话是否仍然有效 (成员函数声明)
    bool isActive() const;
/// 受保护成员
protected:
    /// @brief 会话令牌
    std::string sessionToken;
/// 私有实现细节
private:
    /// @brief 内部验证令牌有效性 (成员函数声明)
    bool validateToken() const;
    /// @brief 用户唯一标识符
    int userId;
    /// @brief 会话激活状态标志
    bool active;
};
```
*   **示例 4.4.3: 函数定义内部的 `///`**
```cpp
bool UserSession::login(const std::string& userName, const std::string& password)
{
    /// 记录尝试登录的用户名
    globalLogger->log("Login attempt for user: " + userName);
    /// 验证用户名和密码
    bool credentialsValid = verifyCredentials(userName, password);
    /// 如果凭证无效则返回失败
    if (!credentialsValid)
    {
        /// 记录登录失败事件
        globalLogger->log("Login failed for user: " + userName);
        /// 返回失败状态
        return false;
    }
    /// 创建新的会话实例
    UserSession newSession;
    /// 设置会话的用户 ID
    newSession.userId = getUserIdFromCredentials(userName);
    /// 生成并设置会话令牌
    newSession.sessionToken = generateSessionToken();
    /// 标记会话为活动状态
    newSession.active = true;
    /// 存储或注册新会话
    registerSession(newSession);
    /// 记录登录成功事件
    globalLogger->log("Login successful for user: " + userName);
    /// 返回成功状态
    return true;
}
```
*   **示例 4.4.4: 枚举定义内部的 `///`**
```cpp
enum class EProcessingState
{
    /// @brief 尚未开始处理
    IDLE,
    /// @brief 正在进行预处理
    PREPROCESSING,
    /// @brief 正在执行核心计算
    COMPUTING,
    /// @brief 处理完成，等待结果获取
    COMPLETED,
    /// @brief 处理过程中发生错误
    ERROR
};
```
这些列举的案例，说明了**默认**采用`///`，凡是未被`/** ... **/`覆盖命令，均为`///`注释

## **5. `/** ... **/` - 主要结构的文档中心 (强制覆盖规则 - 特例)**

*   **5.1 核心作用:** 标记一个**主要结构单元的定义体** (`{}` 块)，提供全面、结构化的文档。这是对默认 `///` 规则的**特例覆盖**。
*   **5.2 强制触发条件 (必须取代 `///`):**
    *   **必须**应用于以下元素的**定义 (Definition)**：
        *   `class` 定义 (带 `{...}`)
        *   `struct` 定义 (带 `{...}`)
        *   `enum class` / `enum` 定义 (带 `{...}`)
        *   `namespace` 定义 (带 `{...}`)
        *   **所有函数定义** (带 `{...}` 实现体)，包括：
            *   普通成员函数、静态成员函数、虚函数。
            *   **构造函数、析构函数**。
            *   **模板函数定义**。
            *   重载运算符函数定义。
            *   全局函数、命名空间内的非成员函数定义。
    *   **重要说明:** 触发条件**仅基于代码元素的语义类型（是否为主要结构定义）**，**与注释内容的长度或复杂度无关**。
        *   **目的 (Purpose):** 体现核心原则 2，强制区分代码的“骨架”与“其他部分”。定义处是提供详尽文档的逻辑位置，与 Doxygen 等工具期望一致，但 Winyunq 规则优先。
*   **5.3 禁止应用:**
    *   **绝对禁止**对任何**声明** (函数原型、类前向声明等) 使用 `/** ... **/`。
    *   **绝对禁止**对**函数体内部**的语句或局部声明使用 `/** ... **/`。
    *   **Lambda 表达式** (`[]{}`) **禁止**使用 `/** ... **/`，其实现逻辑应足够简单，或通过其上下文函数的 `/**` 解释。
        *   **目的 (Purpose):** 严格维护 `/**` 作为**主要结构定义**文档标记的纯粹性。

*   **5.4 格式规则 (强制):**
    *   **位置:** `/** ... **/` 块本身**必须顶格** (从第 1 列开始)，**禁止**任何缩进。
        *   **目的 (Purpose):** 使其在代码中成为醒目的视觉地标，标记主要逻辑单元。
    *   **内部结构:** 遵循**第 6 章**的“默认模式”或“函数特化模式”。
    *   在`/** ... **/`中，不会使用`Tab`，可用理解为在`/**`注释中，`Tab`是0个空格，从而`/**`注释**不会参与缩进**。

*   **5.5 示例 (展示 `/**` 应用于主要结构定义的基本形式):**

*   **示例 5.5.1: 简单的类定义 `/**`** (同示例 2.3.1)
正确案例：在class定义时，形式上有`{`使用，**注释不参与缩进**
```cpp
/**
 * @class       SimpleWidget
 * @brief       一个极其简单的小部件示例类定义
 **/
    class SimpleWidget
    {
    /// 公共接口
    public:
        /// @brief 默认构造函数声明
        SimpleWidget();
    /// 私有数据
    private:
        /// @brief 小部件 ID
        int widgetId;
    };
```
错误案例：前向声明使用了`/**`，形式上没有`{`
```cpp
/**
 * @class       SimpleWidget
 * @brief       一个极其简单的小部件示例类定义
 **/
    class SimpleWidget;
```
错误修正：前向声明没有特殊说明**默认使用`///`**注释，且**只保留`@brief`**。
```cpp
    /// @brief       一个极其简单的小部件示例类定义
    class SimpleWidget;
```
*   **示例 5.5.2: 简单的结构体定义 `/**`**
```cpp
/**
 * @struct      Point2D
 * @brief       表示二维空间中的一个点
**/
struct Point2D
{
    /// @brief X 坐标
    float x;
    /// @brief Y 坐标
    float y;
};
```
*   **示例 5.5.3: 简单的枚举定义 `/**`** 
```cpp
/**
 * @enum        EColorChannel
 * @brief       定义颜色通道的枚举
**/
enum class EColorChannel
{
    /// @brief 红色通道
    RED,
    /// @brief 绿色通道
    GREEN,
    /// @brief 蓝色通道
    BLUE
};
```
*   **示例 5.5.4: 简单的函数定义 `/**`**
```cpp
/**
 * @brief       一个空函数的定义示例
 *  @details     演示 /** 的基本顶格要求和函数定义应用。
**/
void doNothing()
{
    /// 函数体代码省略
    return;
}
```
*   **示例 5.5.5: 构造函数定义 `/**`**
正确案例：在函数定义时使用，**注释不参与缩进**，即便是构造函数，因为形式上使用了`{`，所以**视为定义**，触发`/**`替代`///`
```cpp
/**
 * @brief       SimpleWidget 类的默认构造函数
 *  @details     初始化 widgetId 为默认值 -1。
 **/
    SimpleWidget::SimpleWidget() : widgetId(-1)
    {
        /// 构造函数体可以为空或包含初始化逻辑，无论如何，使用{}即视为定义
    }
```
错误案例：**函数声明不允许采用`/**`注释**
```cpp
/**
 * @brief       SimpleWidget 类的默认构造函数
 *  @details     初始化 widgetId 为默认值 -1。
 **/
    SimpleWidget::SimpleWidget();
```
错误案例修正：函数声明没有特殊说明**默认使用`///`**注释，且**只保留`@brief`**。
```cpp
    ///@brief       SimpleWidget 类的默认构造函数
    SimpleWidget::SimpleWidget();
```
*   **示例 5.5.6: 模板函数定义 `/**`**
```cpp
/**
 * @brief       查找并返回容器中第一个匹配元素的迭代器
 * 
 * @tparam      Container                       容器类型 (如 std::vector, std::list)
 * @tparam      ValueType                       要查找的值的类型
 * @param       参数名称: container                     数据类型:        const Container&
 * @param       参数名称: valueToFind                   数据类型:        const ValueType&
 * @return      指向找到元素的迭代器，未找到则返回 end() 数据类型:        typename Container::const_iterator
 **/
template<typename Container, typename ValueType>
typename Container::const_iterator findInContainer(const Container& container, const ValueType& valueToFind)
{
    /// 使用标准库算法查找
    return std::find(container.begin(), container.end(), valueToFind);
}
```
总结：`/** ... **/`替换`///`的实质是此处是否进行了定义，是否是**Dxoygen生成文档最优先参考处**，形式上为是否有使用`{`。

**凡是形式上没有`{`，实质上也不是定义，不会触发`/** ... **/`替换`///`，请默认使用`///`且单行注释**

## **6. `/** ... **/` (WinyunqDoxygen Block) 规则 (信息中心的构建 - 强制格式)**

*   **6.0 Winyunq 与 Doxygen 的关系:**
    *   Winyunq 风格**使用** Doxygen 兼容的 `/** ... */` 块注释和 `@tag` 语法作为基础。
    *   Winyunq 的**核心价值**在于对 Doxygen 的灵活性进行了**严格的约束、优化和统一**，旨在实现极致的**一致性**和**可读性**。
    *   **规则优先级:** 当 Winyunq 对格式、标签使用、触发条件、内部结构等做出明确规定时，这些规定**必须**遵守，并**优先于** Doxygen 的默认行为或其他风格。
    *   **目的 (Purpose):** 借助 Doxygen 的生态和工具兼容性，同时强制执行 Winyunq 的高标准，实现风格统一和文档质量提升。

*   **6.1 核心目标：可扫描性、信息密度、结构化**
    *   **核心对齐基准列 (强制定义):** Winyunq 定义了三个核心基准列，用于创建可扫描的视觉结构。列号从行首第 1 个字符开始计算，`Tab` 字符在`/**`环境**不会使用**，对齐**必须**通过添加**空格**实现精确控制。
        *   左对齐的核心原则：**绝对列位置**，对齐是基于代码编辑器中的绝对列，不受前导文本长度影响。工具辅助：建议使用支持列标尺的编辑器（如VS Code的插件），确保对齐到15和35列。
        *   **基准列 1 (Col 3):** `*` 后的第一个空格之后的位置。这是所有**后续**描述文本行（非首行，所在行**没有**`@`）的起始列。
        *   **基准列 2 (Col 15):** Doxygen命令 (`@brief`, `@param`, `@return` 等) 后的**首行描述文本**的起始列，注意，**受到前导空格影响**
        *   **基准列 3 (Col 35):** `@param`/`@return` 的 `"数据类型:"` 标签后的 **实际数据类型文本** 的起始列。其**左对齐固定在第35列**（不足35则补充空格使得**视觉上**对齐35列，超过35则**补充一个空格**）**不守前导空格影响**
        *   **目的 (Purpose):** 创建稳定、可预测、易于机器和人类解析的视觉结构，最大限度降低阅读和理解成本。
        *   注意：基准**禁止缩进**，基准始终以最左侧为起点，完全遵循左对齐
```cpp
/**
 * @brief       黄金标准示例：执行复杂数据处理与验证
 *  @details     演示 WinyunqDoxygen Block 的所有主要特性，包括标签层级、对齐、
 * Markdown、LaTeX 和内部空行规则。此函数根据 `options` 处理 `inputData`。
 * **主要步骤:**
 * 1. 输入验证 (Input Validation)
 * 2. 基于 `options.mode` 的预处理 (Preprocessing based on mode)
 * 3. 核心算法执行 (Core Algorithm Execution):
 *    - Mode A: 快速傅里叶变换 ($O(N \log N)$)
 *    - Mode B: 动态规划 ($O(N^2)$)
 *    - Mode C: 查找表 ($O(1)$)
 * 4. 后处理与结果封装 (Postprocessing)
 * 5. 错误处理与日志记录 (Error Handling)
 *   @note        `inputData` 的大小可能显著影响性能。 Mode B 非常耗时。
 *   @warning     如果 `options.calibrationFactor` 无效 (e.g., NaN)，结果可能不可靠。
 *   @throws      std::runtime_error 如果发生不可恢复的内部错误。
 *   @todo        为 Mode B 添加并行化选项。
 *
 * @param       参数名称: inputData                     数据类型:const std::vector<double>&
 *  @details     包含原始输入数据的向量。**禁止**为空。
 * @param       参数名称: outputResult                  数据类型:ProcessingResult&
 *  @details     用于存储处理结果的输出参数（引用）。函数将填充此结构。
 * @param       参数名称: options                       数据类型:const ProcessingOptions&
 *  @details     包含处理模式、校准因子等配置选项的结构体。
 *   @note        `options.mode` 决定了核心算法的选择。
 * @param       参数名称: progressCallback              数据类型:std::function<void(float)>
 *  @details     可选的回调函数，用于报告处理进度 (0.0 到 1.0)。
 *
 * @return      操作成功状态                          数据类型:bool
 *  @retval      true  处理成功完成，`outputResult` 已填充。
 *  @retval      false 处理失败（例如输入无效或内部错误）。`outputResult` 的状态未定义。
 **/
    bool SomeClass::processDataAdvanced(const std::vector<double>& inputData,
	ProcessingResult& outputResult,
	const ProcessingOptions& options,
	std::function<void(float)> progressCallback) {
        /// 省略的代码...
    }
```
遵循**基准列 1 (Col 3):** 的部分:
```cpp
/**
 * Markdown、LaTeX 和内部空行规则。此函数根据 `options` 处理 `inputData`。
 * **主要步骤:**
 * 1. 输入验证 (Input Validation)
 * 2. 基于 `options.mode` 的预处理 (Preprocessing based on mode)
 * 3. 核心算法执行 (Core Algorithm Execution):
 *    - Mode A: 快速傅里叶变换 ($O(N \log N)$)
 *    - Mode B: 动态规划 ($O(N^2)$)
 *    - Mode C: 查找表 ($O(1)$)
 * 4. 后处理与结果封装 (Postprocessing)
 * 5. 错误处理与日志记录 (Error Handling)
 **/
```
这部分内容的**形式特点**是最前方没有`@`开头的Doxygen命令，**逻辑特点**是所处`@`命令**已使用换行符**

遵循**基准列 2 (Col 15):** 的部分:
```cpp
/**
 * @brief       黄金标准示例：执行复杂数据处理与验证
 *  @details     演示 WinyunqDoxygen Block 的所有主要特性，包括标签层级、对齐、
 *   @note        `inputData` 的大小可能显著影响性能。 Mode B 非常耗时。
 *   @warning     如果 `options.calibrationFactor` 无效 (e.g., NaN)，结果可能不可靠。
 *   @throws      std::runtime_error 如果发生不可恢复的内部错误。
 *   @todo        为 Mode B 添加并行化选项。
 *
 * @param       参数名称: inputData                     数据类型:const std::vector<double>&
 *  @details     包含原始输入数据的向量。**禁止**为空。
 * @param       参数名称: outputResult                  数据类型:ProcessingResult&
 *  @details     用于存储处理结果的输出参数（引用）。函数将填充此结构。
 * @param       参数名称: options                       数据类型:const ProcessingOptions&
 *  @details     包含处理模式、校准因子等配置选项的结构体。
 *   @note        `options.mode` 决定了核心算法的选择。
 * @param       参数名称: progressCallback              数据类型:std::function<void(float)>
 *  @details     可选的回调函数，用于报告处理进度 (0.0 到 1.0)。
 *
 * @return      操作成功状态                            数据类型:bool
 *  @retval      true  处理成功完成，`outputResult` 已填充。
 *  @retval      false 处理失败（例如输入无效或内部错误）。`outputResult` 的状态未定义。
 **/
```
这部分内容的**形式特点**是最前方有`@`开头的Doxygen命令，**逻辑特点**是所处`@`命令**未使用换行符**

遵循**基准列 3 (Col 35):** 的部分:
```cpp
/**
 * @param       参数名称: inputData                     数据类型:const std::vector<double>&
 * @param       参数名称: outputResult                  数据类型:ProcessingResult&
 * @param       参数名称: options                       数据类型:const ProcessingOptions&
 * @param       参数名称: progressCallback              数据类型:std::function<void(float)>
 * @return      操作成功状态                            数据类型:bool
 **/
```
这部分内容的**形式特点**是具有数据类型:[Type]，**逻辑特点**是出现字段`数据类型:[Type]`

`/** ... **/`中的三个基准始终以最左侧为起点，**基准列 2 (Col 15):**只受前导空格影响，**基准列 1 (Col 3):**，**基准列 3 (Col 35):**，完全遵守左对齐，**不参与任何缩进**。

*   **6.2 默认块结构 (适用于 Class/Struct/Enum/Namespace 定义):**
    *   **结构:**
        *   1.  **禁止空行:** `/** ... **/`最终目的用于Doxygen生成手册，只考虑Doxygen排版效果，**不允许**使用物理空行分割。 
        *   2.  **结构类型标签:** `@class Name`, `@struct Name`, `@enum Name`, `@namespace Name`。
        *   3.  **摘要:** `@brief` (必须)。
        *   4.  **继承关系 (若有, 仅限 Class/Struct):** `@extends [可见性] [基类名]` (每个基类一行，Winyunq 约定，使用 `@extends`)。
        *   5.  **详细描述 (可选但推荐):** `@details`。
        *   6.  **补充说明 (可选，挂载于 @details 下):**`其他doxygen命令`(`@note`, `@warning`, `@todo` 等)**未提及的Doxygen命令均为补充说明**。
    *   **标签说明与层级:**
        *   `@class`/`@struct`/`@enum`/`@namespace`: (0 空格) 标识结构类型。
        *   `@brief`: (0 空格) 核心功能摘要。
        *   `@extends`: (0 空格) 描述继承关系。可见性 (`public`, `protected`, `private`) 和基类名。
        *   `@details`: (1 空格) 详细解释、背景、设计原理等。
        *   `其他doxygen命令`(`@`开头的Doxygen命令，例如`@note`/`@warning`/`@todo`): (2 空格) 从属于 `@details`，即凡是未提及的Doxygen命令，**默认**挂载于`@details`下。
            *   **目的 (Purpose for nesting under @details):** 补充信息通常需要详细背景。
        *   **一旦换行**，换行后的内容遵循**基准列 1 (Col 3):**，即任何左侧没有`@`开头的Doxygen命令，则遵循**基准列 1 (Col 3):**
    *   **示例 6.2.1: 带有继承和详细信息的类定义 `/**` (使用 @extends)**
```cpp
/**
 * @class       AdvancedWidget
 *  @extends     public BaseWidget
 *  @extends     private ISerializable
 * @brief       一个带有配置和状态的高级小部件
 *  @details     此类继承自 BaseWidget 和 ISerializable，增加了配置加载、运行时状态管理以及序列化功能。
 *   @warning     序列化格式可能在未来版本中更改。
 *   @todo        实现对 XML 格式的序列化支持。
 **/
class AdvancedWidget : public BaseWidget, private ISerializable
{
    // ... class members with /// comments ...
};
```

*   **6.3 函数块结构 (特化模式):**
    *   **结构:** 基于默认结构 (`@brief`, `@details`，`其他doxygen命令`等)，**在它们之后**，按顺序添加函数特有的部分。
        *   1.  **基础信息** : `@brief`, `@details`, `其他doxygen命令`等从属标签
        *   2.  **函数参数 (若有):** `@param 参数名称: [Name] 数据类型:[Type]` (每个一行，可带 `@details`, `@note` 子标签)。**函数信息**与**基础信息**存在**换行**
        *   3.  **模板参数（若有）**模板参数被视为特殊参数，仅将`@param`改为`@tparam`，视为第一个参数。
        *   4.  **返回值 (若非 `void`):** `@return [含义描述] 数据类型:[Type]` (可带 `@details`, `@retval` 子标签)。返回值与前面的内容存在**换行**
    *   **标签说明:**
        *   `@brief`: (0 空格) 函数的核心功能摘要。**覆盖** `@fn`。
        *   `@details`/`@note`/`@warning`/`@throws`/`@todo`: 同默认模式。
        *   `@tparam`: (0 空格) 描述模板参数。除Doyxgen命令改为`@tparam`，其余与`@param`**一致**
            *   **目的 (Purpose for placing before @param):** 模板参数是函数签名的一部分，逻辑上视为第一个普通参数。
        *   `@param`: (0 空格) 描述函数参数。`参数名称:` 对齐到**基准列 2 (Col 15):**，`数据类型:` 对齐到**基准列 3 (Col 35):**。可嵌套 `@details`/`@note` (1/2 空格)。
        *   `@return`: (0 空格) 描述返回值。含义描述和 `数据类型:` 后的类型文本需对齐到**基准列 3 (Col 35):**。可嵌套 `@details` (1 空格) 或 `@retval` (2 空格)。
    *   **示例 6.3.1: 完整的函数定义 `/**` (黄金标准 - 同 v2.5 示例 5.3.1)**
    *   **示例 6.3.2: 使用 @retval (非布尔)**
```cpp
/**
 * @brief       根据文件扩展名确定文件类型
 * 
 * @param       参数名称: filePath                      数据类型:const std::string&
 * 
 * @return      表示文件类型的枚举值                  数据类型:EFileType
 *  @retval      EFileType::IMAGE   如果是图片文件 (jpg, png, gif)。
 *  @retval      EFileType::TEXT    如果是文本文件 (txt, md)。
 *  @retval      EFileType::UNKNOWN 如果无法识别或无扩展名。
 **/
EFileType determineFileType(const std::string& filePath)
{
    /// 示例返回值
    return EFileType::UNKNOWN;
}
```

*   **6.4 内部格式化规则 (强制):**
    *   **禁止默认对齐:** **绝对禁止**简单地将所有 `@tag` 或文本顶格 (`* @tag`) 或随意缩进。**必须**遵循 Winyunq 的基准列对齐系统。
    *   **标签行缩进层级 (强制):**
        *   `@brief`, `@class`/`@struct`/`@enum`/`@namespace`, `@extends`, `@tparam`, `@param`, `@return`: **0 空格** (`* @tag`) - 顶层信息。
        *   `@details`: **1 空格** (`*  @details`) - 从属于上方主标签，提供详细阐述。
        *   `@note`, `@warning`, `@throws`, `@todo`, `@retval`: **2 空格** (`*   @tag`) - 从属于 `@details` (或 `@param`/`@return` 的 `@details`)，提供特定角度的补充信息。
        *   **目的 (Purpose):** 通过缩进反映信息的层级和从属关系。
    *   **文本对齐 (规则覆盖 - 强制):**
        *   **规则 A (主文本/名称 - 列 15):** 标签后的首行描述、`@param` 的 `"参数名称:"`、`@return` 的含义描述，**必须**从**基准列 2 (Col 15)** 开始。
        *   **规则 B (类型文本 - 列 35):** `@param`/`@return` 的 `"数据类型:"` 标签本身从列 15 开始，其后的**实际类型**文本**必须**从**基准列 3 (Col 30)** 开始。
        *   **规则 C (后续行 - 列 3):** **任何**描述文本（无论层级）的**第二行及之后**，**必须**从**基准列 1 (Col 3)** 开始。**禁止**悬挂缩进或其他对齐方式。
        *   **目的 (Purpose):** 规则 A/B 确保列对齐。规则 C 确保多行文本块左边界统一，提高段落可读性。
    *   **内部空行 (严格限制):** 空 `*` 行**仅允许且必须**用于分隔不同的**顶级标签组** (brief/details 组, tparam/param 组, return 组)。**禁止**在组内（如多个 `@param` 之间）或多行描述内部使用。
        *   **目的 (Purpose):** 保持主要信息块间视觉分隔，同时确保块内部紧凑连续。
    *   **标签顺序 (强制，函数特化):** `@brief`,`@details` (及其从属) -> `@tparam`/`@param` -> `@return`。
        *   **目的 (Purpose):** 提供一致的文档结构，便于查找。

*   **6.5 最小化与省略规则:**
    *   **必须包含:** `@brief` (及对应的 `@class` 等)。
    *   **条件性包含:** `@extends` (若有继承), `@tparam`, `@param`, `@return` (函数)。
    *   **推荐包含:** `@details` 及从属标签。
    *   **格式不变性:** 省略可选部分时，剩余部分的**所有格式规则（对齐、缩进、内部空行、基准列）必须严格保持不变**。
        *   **目的 (Purpose):** 即使内容简化，也要维护结构的稳定性和可预测性。
    *   **默认处理方式** 凡是未明确说明的Doxygen命令，挂载于属于`@brief`下的`@details`
        *   **目的 (Purpose):** 针对未说明情况采用统一处理。

## **7. 文件与命名规范 (强制)**

*   **文件编码:** **必须** UTF-8 无 BOM。
*   **头文件保护:** **必须** `#pragma once`。
*   **命名约定 (强制):**
    *   **类型 (Class, Struct, Enum Type):** `UpperCamelCase` (内部无 `_`)。
    *   **模板参数:** `UpperCamelCase` (例如 `DataType`, `IteratorType`)。**禁止**使用无意义的单字母（如 `T`, `U`），除非在极简单、上下文极明确的泛型函数中（如 `std::swap`），但优先使用描述性名称。
    *   **变量 (Member, Global, Local), 参数, 函数/方法:** `lowerCamelCase` (内部无 `_`)。**必须清晰有意义**，避免模糊或过度简短的缩写。
    *   **常量 (Compile-time):** `SCREAMING_SNAKE_CASE` (必须用 `_`)。
    *   **枚举值:** `SCREAMING_SNAKE_CASE` (必须用 `_`, 例如 `ERROR_FILE_NOT_FOUND`)。
    *   **命名空间:** `lowercase` 或 `lower_snake_case`。
    *   **宏 (`#define`):** 强烈不推荐。若必须使用，采用 `SCREAMING_SNAKE_CASE`，其定义前**必须**有 `/// @brief` (单行)。
    *   **目的 (Purpose):** 通过大小写和下划线明确区分元素种类。清晰的命名是代码可读性的关键。强制模板参数和枚举值使用描述性名称以提高理解性。

*   **示例 (命名规范):**
```cpp
/// 使用 #pragma once 防止头文件重复包含
#pragma once
/// 包含标准库字符串
#include <string>
/// 包含标准库映射
#include <map>
/**
 * @namespace   ConfigManager
 * @brief       提供加载和访问应用程序配置的功能
 *  @details     该命名空间包含用于解析配置文件和检索设置项的工具。
 **/
namespace ConfigManager
{
    /// @brief 配置项的最大长度 (常量)
    const int MAX_CONFIG_VALUE_LENGTH = 256;
/** 
 * @enum        ELoadResult
 * @brief       配置文件加载操作的结果状态
 **/
    enum class ELoadResult
    {
        /// @brief 加载成功
        LOAD_SUCCESS,
        /// @brief 文件未找到
        ERROR_FILE_NOT_FOUND,
        /// @brief 文件格式错误
        ERROR_INVALID_FORMAT
    };
/**
 * @struct      ConfigData
 * @brief       存储所有已加载的配置项
 **/
    struct ConfigData
    {
        /// @brief 存储键值对的映射 (成员变量)
        std::map<std::string, std::string> settingsMap;
    };
    /// @brief 从指定路径加载配置文件
    ELoadResult loadConfigurationFromFile(const std::string& filePath);
    /// @brief 获取指定键的配置值 (若存在)
    bool getConfigurationValue(const std::string& key, std::string& outValue);
/**
 * @brief       从指定路径加载配置文件
 *  @details     尝试打开并解析配置文件，将结果存储在内部。
 * 
 * @param       参数名称: filePath                      数据类型:const std::string&
 *  @details     配置文件的完整路径。
 * 
 * @return      加载操作的结果状态                    数据类型:ELoadResult
 **/
    ELoadResult loadConfigurationFromFile(const std::string& filePath)
    {
        /// 加载成功
        return ELoadResult::LOAD_SUCCESS;
    }
/**
 * @brief       获取指定键的配置值 (若存在)
 * 
 * @param       参数名称: key                           数据类型:        const std::string&
 *  @details     要检索的配置项的键。
 * @param       参数名称: outValue                      数据类型:        std::string&
 *  @details     用于接收配置值的输出参数。
 * 
 * @return      是否成功找到并获取了值                数据类型:        bool
 *  @retval      true  键存在，`outValue` 已被设置。
 *  @retval      false 键不存在。
 **/
    bool getConfigurationValue(const std::string& key, std::string& outValue)
    {
        /// 未找到，返回失败
        return false;
    }

}
```
## **8. 代码布局与格式化 (强制 - 实现视觉一致性)**

*   **8.1 缩进:** **必须** Tab 字符，视觉宽度 4 空格。**禁止**使用空格进行缩进。
    *   **目的 (Purpose):** 强制统一缩进方式。
*   **8.2 空格:** 运算符两侧、逗号/分号后、控制关键字后括号前加空格。函数名与括号间、一元运算符与操作数间、类型与`&`/`*`间、括号内部不加空格。
    *   **目的 (Purpose):** 遵循广泛接受的 C++ 编码习惯，提高可读性。
*   **8.3 类/结构体内部布局:**
    *   **禁止**任何物理空行。
    *   成员（变量、函数声明/定义）之间**必须**由其上方的 `///` (单行) 注释分隔。
    *   访问控制符 (`public:`, `private:`, `protected:`) **必须**独占一行，**允许**缩进，其上方**必须**有 `///` (单行) 注释描述该区域用途 (e.g., `/// 公共接口`, `/// 私有实现细节`)。
        *   **目的 (Purpose):** 强制使用 `///` 作为唯一分隔符，保持信息密度。清晰标记访问区域。

## **9. 快速验证指南 (核心检查点)**

*   [ ] **术语:** 文档是否正确区分并使用了“声明”和“定义”？
    *   [ ] 是否确定对于每一个函数或类的定义，存在唯一一个使用`/** ... **/`注释，用于Doxygen生成文档，而其他地方仅仅是`/// @brief`简单介绍
*   [ ] **禁止项:** 是否**完全没有** `//` 行尾注释、`}` 后注释、非 Winyunq 注释、物理空行？ **(核心!)**
*   [ ] **`///` 规则:**
    *   [ ] 是否应用于所有声明和块内元素？
    *   [ ] 是否**严格为单行**？ **(核心!)**
    *   [ ] 格式（正上方、无空隙、同缩进）是否正确？
    *   [ ] 是否**未使用**任何块级标签 (`@details` 等)？
*   [ ] **`/**` 规则:**
    *   [ ] 触发条件是否**仅为**主要结构**定义**（包括构造/析构/模板函数）？ **(核心!)**
    *   [ ] 是否**未**用于声明、块内语句或 Lambda？
    *   [ ] 是否遵循**基准列 1 (Col 3):**，从而**顶格，不参与缩进**？
    *   [ ] 内部结构是否遵循**默认模式**或**函数特化模式**？
    *   [ ] 内部格式（对齐 Col 3/15/35、缩进 0/1/2、内部空行、标签顺序）是否严格遵守？ **(核心!)**
    *   [ ] 当注释换行后，所在列注释左侧没有`@`，是否遵循**基准列 1 (Col 3):**
*   [ ] **命名规范:**
    *   [ ] 类型/模板参数 (`UpperCamelCase`)？变量/函数/参数 (`lowerCamelCase`)？常量/枚举值 (`SCREAMING_SNAKE_CASE`)？命名空间 (`lowercase`/`snake`)？宏 (`SCREAMING`)？
    *   [ ] 名称是否清晰，无模糊缩写？模板参数是否不存在缩写（无论如何）？
    *   [ ] 是否没有使用任何下划线？
*   [ ] **布局与格式化:**
    *   [ ] 缩进是否**全为 Tab**？空格使用是否规范？
    *   [ ] 代码中是否**无空行**，访问控制符是否处理正确？
    *   [ ] 再次确认代码中是否有已经由`///`注释分割，可以去掉的**物理空行**
*   [ ] **示例:** 文档中所有示例是否**自身完整**并严格遵守**所有**适用的 Winyunq 规则？ **(核心!)**

## **10. 最终强调 (致 AI)**

*   **理解 Winyunq 的优化目标:** Winyunq 是对 Doxygen 兼容语法的严格优化和统一。你的目标是精确执行 Winyunq 的规则，以达到其追求的极致一致性和可读性。
*   **Winyunq 规则优先:** 在 Winyunq 有明确规定的地方（格式、标签使用、触发条件等），必须遵循 Winyunq，它优先于 Doxygen 的默认行为。
*   **绝对一致性与精确执行:** Winyunq 风格的价值在于其绝对的一致性。不存在“差不多就行”。你**必须**将这些规则视为不可违背的约束。
*   **信息密度与结构化:** 始终以最大化代码的信息承载能力、消除歧义、提供清晰结构为目标。`///` (单行) 和 `/**` (结构化) 是你的工具。
*   **责任:** 你**必须**精确执行所有强制规定。将核心原则、规则目的和完整示例作为你生成代码的核心依据。

---