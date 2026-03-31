/**
 * @file pyro_core_fsm.tpp
 * @brief Template implementation for the PYRO Core FSM.
 * PYRO 核心 FSM 的模板实现。
 */
#pragma once

namespace pyro
{

/* state_t Implementation ---------------------------------------------------*/

/**
 * @brief Get the pointer to the current state instance.
 * 获取当前状态实例的指针。
 *
 * @return state_t<Context>*
 * Pointer to this instance.
 * 指向此实例的指针。
 */
template <typename Context>
state_t<Context> *state_t<Context>::get_instance()
{
    return this;
}

/**
 * @brief Internal method to request a transition.
 * 状态内部请求切换状态的方法。
 *
 * @param next
 * Pointer to the target state.
 * 指向目标状态的指针。
 */
template <typename Context>
void state_t<Context>::request_switch(state_t<Context> *next)
{
    _requested_state = next;
}

/**
 * @brief Fetches and clears the pending transition request.
 * 获取并清除待处理的转换请求。
 *
 * @return state_t<Context>*
 * The requested state pointer.
 * 被请求的状态指针。
 */
template <typename Context>
state_t<Context> *state_t<Context>::fetch_request()
{
    state_t<Context> *next = _requested_state;
    _requested_state       = nullptr;
    return next;
}

/**
 * @brief Discards any pending transition request.
 * 丢弃任何待处理的转换请求。
 */
template <typename Context>
void state_t<Context>::discard_request()
{
    _requested_state = nullptr;
}

/* fsm_t Implementation -----------------------------------------------------*/

/**
 * @brief Enters the FSM and the initial active state.
 * 进入 FSM 及初始活动状态。
 *
 * @param ctx
 * Pointer to the system context.
 * 指向用户传参的指针。
 */
template <typename Context>
void fsm_t<Context>::enter(Context *ctx)
{
    on_enter(ctx);
    if (_active_state)
    {
        _active_state->enter(ctx);
    }
}

/**
 * @brief Updates the FSM and handles logic.
 * 更新 FSM 并执行相关逻辑。
 *
 * @details
 * Handles state transitions (Phase A) before executing logic (Phase B).
 * 在执行逻辑（阶段 B）之前处理状态转换（阶段 A）。
 *
 * @param ctx
 * Pointer to the system context.
 * 指向用户传参的指针。
 */
template <typename Context>
void fsm_t<Context>::execute(Context *ctx)
{
    // Phase A: Transition Processing.
    // 阶段 A：转换处理。
    if (process_switch(ctx))
        return;

    // Phase B: Execute FSM-level logic.
    // 阶段 B：执行 FSM 层级的逻辑。
    on_execute(ctx);

    if (_target_state)
        return;

    if (!_active_state)
        return;

    // Execute the active state's logic.
    // 执行当前活动状态的逻辑。
    _active_state->execute(ctx);

    // Sync: Fetch requested state from the child state.
    // 同步：从子状态中获取请求的目标状态。
    if (auto req = _active_state->fetch_request())
    {
        _target_state = req;
    }
}

/**
 * @brief Exits the FSM and the current active state.
 * 退出 FSM 及当前活动状态。
 *
 * @param ctx
 * Pointer to the system context.
 * 指向用户传参的指针。
 */
template <typename Context>
void fsm_t<Context>::exit(Context *ctx)
{
    if (_active_state)
    {
        _active_state->exit(ctx);
        _active_state->discard_request();
    }
    on_exit(ctx);
}

/**
 * @brief Resets the FSM by re-targeting the active state.
 * 通过重新将当前活动状态设为目标来重置 FSM。
 */
template <typename Context>
void fsm_t<Context>::reset()
{
    _target_state = _active_state;
}

/**
 * @brief Manually triggers a state change.
 * 手动触发状态切换。
 *
 * @param next
 * Pointer to the next state.
 * 指向下一个状态的指针。
 */
template <typename Context>
void fsm_t<Context>::change_state(state_t<Context> *next)
{
    if (next != _active_state)
        _target_state = next;
}

/**
 * @brief Internal logic to process state transitions.
 * 处理状态转换的内部逻辑。
 *
 * @param ctx
 * Pointer to the system context.
 * 指向用户传参的指针。
 *
 * @return bool
 * True if a transition occurred.
 * 如果发生了转换则返回 true。
 */
template <typename Context>
bool fsm_t<Context>::process_switch(Context *ctx)
{
    if (!_target_state)
        return false;

    // Exit old state.
    // 退出旧状态。
    if (_active_state)
    {
        _active_state->exit(ctx);
        _active_state->discard_request();
    }
    _last_state = _active_state;
    _active_state = _target_state;

    // Enter new state.
    // 进入新状态。
    if (_active_state)
    {
        _active_state->enter(ctx);
    }

    _target_state = nullptr;
    return true;
}

/* Hook Default Implementations ----------------------------------------------*/

template <typename Context>
void fsm_t<Context>::on_enter(Context *ctx) { (void)ctx; }

template <typename Context>
void fsm_t<Context>::on_exit(Context *ctx) { (void)ctx; }

template <typename Context>
void fsm_t<Context>::on_execute(Context *ctx) { (void)ctx; }

} // namespace pyro