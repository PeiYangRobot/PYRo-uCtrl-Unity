/**
 * @file pyro_core_fsm.h
 * @brief Header for the PYRO Core Finite State Machine (FSM) Library.
 * PYRO 核心有限状态机（FSM）库头文件。
 */

#ifndef __PYRO_CORE_FSM_H__
#define __PYRO_CORE_FSM_H__

namespace pyro
{

/* Forward Declarations ------------------------------------------------------*/
template <typename Context> class fsm_t;

/**
 * @brief Abstract base class for all states.
 * 所有状态的抽象基类。
 *
 * @tparam Context
 * Shared system context type.
 * 各状态间共享的用户传参类型。
 */
template <typename Context> class state_t
{
  protected:
    /**
     * @brief Pointer to the requested next state.
     * 指向请求切换的目标状态指针。
     */
    state_t<Context> *_requested_state = nullptr;

  public:
    virtual ~state_t()                 = default;

    /**
     * @brief Called when entering the state.
     * 进入状态时调用的回调。
     *
     * @param ctx Pointer to the system context.
     * 指向用户传参的指针。
     */
    virtual void enter(Context *ctx)   = 0;

    /**
     * @brief Called every update cycle.
     * 每个更新周期调用的逻辑回调。
     *
     * @param ctx Pointer to the system context.
     * 指向用户传参的指针。
     */
    virtual void execute(Context *ctx) = 0;

    /**
     * @brief Called when exiting the state.
     * 退出状态时调用的回调。
     *
     * @param ctx Pointer to the system context.
     * 指向用户传参的指针。
     */
    virtual void exit(Context *ctx)    = 0;

    /**
     * @brief Get the current instance pointer.
     * 获取当前实例指针。
     *
     * @return state_t<Context>* Instance pointer.
     * 实例指针。
     */
    state_t<Context> *get_instance();

  protected:
    /**
     * @brief Request a state transition.
     * 请求状态切换的回调处理器。
     *
     * @param next Target state. 目标状态。
     */
    void request_switch(state_t<Context> *next);

  private:
    /** @brief Internal fetch logic. 内部获取逻辑。 */
    state_t<Context> *fetch_request();

    /** @brief Internal discard logic. 内部丢弃逻辑。 */
    void discard_request();

    friend class fsm_t<Context>;
};

/**
 * @brief FSM Controller Class.
 * FSM 控制器类。
 */
template <typename Context> class fsm_t : public state_t<Context>
{
  protected:
    state_t<Context> *_last_state   = nullptr;
    state_t<Context> *_active_state = nullptr;
    state_t<Context> *_target_state = nullptr;

  public:
    void enter(Context *ctx) final;
    void execute(Context *ctx) final;
    void exit(Context *ctx) final;

    void reset();
    void change_state(state_t<Context> *next);

  protected:
    /** @brief FSM entry callback. FSM 进入回调。 */
    virtual void on_enter(Context *ctx);

    /** @brief FSM exit callback. FSM 退出回调。 */
    virtual void on_exit(Context *ctx);

    /** @brief FSM execution callback. FSM 执行逻辑回调。 */
    virtual void on_execute(Context *ctx);

  private:
    bool process_switch(Context *ctx);
};

} // namespace pyro

#include "pyro_core_fsm.tpp"

#endif // __PYRO_CORE_FSM_H__