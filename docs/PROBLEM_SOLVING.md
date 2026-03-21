# 问题解决记录

## 问题：spawn 任务的正确用法

### 问题描述

尝试使用 `spawner.spawn()` 启动异步任务时遇到编译错误：

```
expected SpawnToken<{unknown}>, found Result<SpawnToken<impl Sized>, SpawnError>
```

### 错误尝试

#### 尝试 1：使用 `.unwrap()`

```rust
spawner.spawn(timer_1s()).unwrap();
```

**结果**: 编译错误，`unwrap()` 用法不正确。

#### 尝试 2：使用 `.ok()`

```rust
spawner.spawn(timer_1s()).ok();
```

**结果**: 同样的编译错误。

#### 尝试 3：使用 defmt 的 `unwrap!` 宏（错误位置）

```rust
unwrap!(spawner.spawn(timer_1s()));
```

**结果**: 编译错误，宏期望的类型不匹配。

### 正确解法

查阅 Embassy 官方示例 `i2c_slave.rs` 后发现，`unwrap!` 应该放在 `spawn()` 参数内部：

```rust
// 正确写法
spawner.spawn(unwrap!(timer_1s()));
```

### 原因分析

`#[embassy_executor::task]` 宏会将任务函数包装成返回 `Result<SpawnToken, SpawnError>` 的形式。`spawn()` 方法期望接收 `SpawnToken`，因此需要在传入前解包。

`defmt` 提供的 `unwrap!` 宏用于在编译时解包这个 Result。

### 关键代码对比

```rust
// 定义任务
#[embassy_executor::task]
async fn timer_1s() {
    // ...
}

// 启动任务 - 正确
spawner.spawn(unwrap!(timer_1s()));

// 启动任务 - 错误
spawner.spawn(timer_1s()).unwrap();  // 错误！
unwrap!(spawner.spawn(timer_1s()));  // 错误！
```

### 参考来源

- Embassy 示例: `examples/bin/i2c_slave.rs`
- Embassy 文档: https://embassy.dev/

## 另一种方案：使用 select

如果不使用 `spawn`，也可以在 `main` 中使用 `embassy_futures::select` 并行运行多个 Future：

```rust
// 方案 2：使用 select（不推荐用于此场景）
embassy_futures::select::select3(
    timer_1s(),
    timer_2s(),
    core::future::pending::<()>()
).await;
```

但对于需要展示 Embassy 任务调度能力的场景，`spawn` 方式更加直观和符合设计理念。

## 总结

| 写法 | 正确性 | 说明 |
|---|---|---|
| `spawner.spawn(unwrap!(task()))` | 正确 | 官方推荐写法 |
| `spawner.spawn(task()).unwrap()` | 错误 | Result 类型不匹配 |
| `unwrap!(spawner.spawn(task()))` | 错误 | 宏参数类型错误 |