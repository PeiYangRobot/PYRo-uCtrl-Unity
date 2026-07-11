# PYRo Module Migration Guide

本文档说明如何从旧版 `module_base_t<Derived, CmdType, ModuleDeps>` 更新到新版
`module_base_t<Derived, ModuleParams>`。

新版 Module 的核心变化是：模块基类只接收两个模板参数，第二个参数是一个聚合参数类型，
用于统一声明 `CmdType`、`ModuleDeps` 和 `ModuleCtx`。同时，模块上下文由基类统一持有，
并通过继承得到的 `get_ctx()` 访问。

## 1. 旧版写法

旧版 Module 通常直接把命令类型和依赖类型传给基类，并在派生类里自己定义、持有和暴露
context。

```cpp
struct ExampleCmd final : public pyro::cmd_base_t
{
    bool enable{false};
};

struct ExampleDeps
{
    DriverType *driver{nullptr};
};

class ExampleModule final
    : public pyro::module_base_t<ExampleModule, ExampleCmd, ExampleDeps>
{
    friend class pyro::module_base_t<ExampleModule, ExampleCmd, ExampleDeps>;

  public:
    struct DataCtx
    {
        float value{0.0f};
    };

    struct Context
    {
        ExampleDeps deps{};
        DataCtx data{};
        ExampleCmd *cmd{nullptr};
    };

    [[nodiscard]] Context &get_ctx()
    {
        return _ctx;
    }

  private:
    ExampleModule();

    pyro::status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    Context _ctx{};
};
```

## 2. 新版写法

新版需要先定义模块参数聚合类型，例如 `ExampleModuleParams`。这个类型至少包含三个别名：

- `CmdType`：模块命令类型。
- `ModuleDeps`：模块依赖类型。
- `ModuleCtx`：模块上下文类型。

由于新版基类会直接持有 `ModuleCtx _ctx`，所以 `ModuleCtx` 必须在继承
`module_base_t` 之前是完整类型。推荐把 context 相关结构体放在模块类外部定义。

```cpp
struct ExampleCmd final : public pyro::cmd_base_t
{
    bool enable{false};
};

struct ExampleDeps
{
    DriverType *driver{nullptr};
};

struct ExampleDataCtx
{
    float value{0.0f};
};

struct ExampleCtx
{
    ExampleDeps deps{};
    ExampleDataCtx data{};
    ExampleCmd *cmd{nullptr};
};

struct ExampleModuleParams
{
    using CmdType    = ExampleCmd;
    using ModuleDeps = ExampleDeps;
    using ModuleCtx  = ExampleCtx;
};

class ExampleModule final
    : public pyro::module_base_t<ExampleModule, ExampleModuleParams>
{
    friend class pyro::module_base_t<ExampleModule, ExampleModuleParams>;

  public:
    using data_ctx_t = ExampleDataCtx;
    using context_t  = ExampleCtx;

  private:
    ExampleModule();

    pyro::status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;
};
```

## 3. 需要更新的点

### 3.1 修改继承参数

旧版：

```cpp
class ExampleModule final
    : public pyro::module_base_t<ExampleModule, ExampleCmd, ExampleDeps>
{
};
```

新版：

```cpp
class ExampleModule final
    : public pyro::module_base_t<ExampleModule, ExampleModuleParams>
{
};
```

### 3.2 修改 friend 声明

旧版：

```cpp
friend class pyro::module_base_t<ExampleModule, ExampleCmd, ExampleDeps>;
```

新版：

```cpp
friend class pyro::module_base_t<ExampleModule, ExampleModuleParams>;
```

### 3.3 新增参数聚合类型

为每个模块新增一个参数聚合类型：

```cpp
struct ExampleModuleParams
{
    using CmdType    = ExampleCmd;
    using ModuleDeps = ExampleDeps;
    using ModuleCtx  = ExampleCtx;
};
```

### 3.4 移除派生类自己的 context 成员

旧版派生类通常会自己持有：

```cpp
Context _ctx{};
```

新版中 `_ctx` 已由 `module_base_t` 统一持有，派生类不需要再声明自己的 `_ctx`。

### 3.5 移除派生类自己的 get_ctx()

旧版：

```cpp
[[nodiscard]] Context &get_ctx()
{
    return _ctx;
}
```

新版中基类已经提供：

```cpp
[[nodiscard]] ModuleCtx &get_ctx();
```

用户直接调用即可：

```cpp
auto &ctx = ExampleModule::instance()->get_ctx();
```

注意：`get_ctx()` 返回的是 `ModuleCtx &`，不是指针，也不是拷贝。

### 3.6 将嵌套 context 类型移到类外

如果旧版把 `DataCtx`、`Context` 定义在模块类内部，新版建议移动到模块类外部：

旧版：

```cpp
class ExampleModule final
{
    struct DataCtx
    {
        float value{0.0f};
    };

    struct Context
    {
        DataCtx data{};
    };
};
```

新版：

```cpp
struct ExampleDataCtx
{
    float value{0.0f};
};

struct ExampleCtx
{
    ExampleDataCtx data{};
};
```

这是因为新版 `module_base_t` 在基类内部持有 `ModuleCtx _ctx`，实例化基类时需要知道
`ModuleCtx` 的完整定义。

## 4. 实现文件中的更新

模块实现文件中对 `_ctx` 的使用通常不需要改变，因为 `_ctx` 仍然是派生类可访问的
protected 成员。

```cpp
pyro::status_t ExampleModule::_init()
{
    _ctx = {};
    _ctx.deps = _module_deps;
    return pyro::status_t::PYRO_OK;
}

void ExampleModule::_update_feedback()
{
    _ctx.data.value = read_value();
}
```

如果原本实现了派生类的 `get_ctx()`，删除该实现即可。

## 5. 迁移检查清单

- 将继承从 `module_base_t<Derived, CmdType, ModuleDeps>` 改为
  `module_base_t<Derived, ModuleParams>`。
- 新增 `ModuleParams` 类型，并提供 `CmdType`、`ModuleDeps`、`ModuleCtx` 三个别名。
- 确保 `ModuleCtx` 在模块类继承基类之前已经完整定义。
- 删除派生类自己的 `_ctx` 成员。
- 删除派生类自己的 `get_ctx()` 声明和实现。
- 更新 `friend class module_base_t<...>` 的模板参数。
- 保留 `_init()`、`_update_feedback()`、`_fsm_execute()` 的实现方式。
- 原有 `set_command()`、`configure()`、`start()` 的调用方式通常不需要修改。

## 6. 常见编译错误

### incomplete type

如果出现 `ModuleCtx` 是 incomplete type 的错误，通常说明 `ModuleCtx` 仍然定义在模块类内部，
或者定义顺序晚于继承语句。把 `ModuleCtx` 移到类外，并放在 `ModuleParams` 之前即可。

### get_ctx redefinition

如果出现 `get_ctx()` 重定义，说明派生类还保留了旧版 `get_ctx()`。新版由基类统一提供，
删除派生类中的声明和实现。

### no member named ModuleCtx

如果出现 `ModuleParams` 中没有 `ModuleCtx` 的错误，检查参数聚合类型是否包含：

```cpp
using ModuleCtx = ExampleCtx;
```

## 7. 最小新版模板

```cpp
struct ExampleCmd final : public pyro::cmd_base_t
{
};

struct ExampleDeps
{
};

struct ExampleCtx
{
    ExampleCmd *cmd{nullptr};
};

struct ExampleModuleParams
{
    using CmdType    = ExampleCmd;
    using ModuleDeps = ExampleDeps;
    using ModuleCtx  = ExampleCtx;
};

class ExampleModule final
    : public pyro::module_base_t<ExampleModule, ExampleModuleParams>
{
    friend class pyro::module_base_t<ExampleModule, ExampleModuleParams>;

  private:
    ExampleModule();

    pyro::status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;
};
```
