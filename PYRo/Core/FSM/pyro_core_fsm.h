/**
 * @file pyro_core_fsm.h
 * @brief Header file for the PYRO Core Finite State Machine (FSM) Library.
 *
 * This file defines the `pyro::state_t` base class and the `pyro::fsm_t`
 * container class. It implements a Context-driven hierarchical state machine
 * framework featuring:
 * 1. Strict lifecycle management (Enter -> Execute -> Exit).
 * 2. Clean State Tick logic (separation of Logic Frames and Transition Frames).
 * 3. Unified internal/external transition buffering.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-12-13
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_CORE_FSM_H__
#define __PYRO_CORE_FSM_H__

namespace pyro
{

/* Forward Declarations ------------------------------------------------------*/
template <typename Context> class fsm_t;

/* Class Definition ----------------------------------------------------------*/

/**
 * @brief Abstract base class for all states.
 *
 * Defines the standard lifecycle (enter/execute/exit) and the mechanism
 * for a child state to request a transition.
 *
 * @tparam Context The shared data context type.
 */
template <typename Context> class state_t
{
  protected:
    /* Protected Members -----------------------------------------------------*/
    /**
     * @brief [Buffer] The target state this instance wants to switch to.
     * @note Only used by a child state to petition its parent FSM.
     */
    state_t<Context> *_requested_state = nullptr;

  public:
    /* Public Methods - Lifecycle --------------------------------------------*/
    virtual ~state_t()                 = default;

    /**
     * @brief Triggered when entering the state.
     *
     * Used for resource initialization, timer resets, etc.
     *
     * @param ctx Pointer to the shared context.
     */
    virtual void enter(Context *ctx)   = 0;

    /**
     * @brief Core execution logic.
     *
     * Used for business logic, typically called every frame.
     *
     * @param ctx Pointer to the shared context.
     */
    virtual void execute(Context *ctx) = 0;

    /**
     * @brief Triggered when exiting the state.
     *
     * Used for cleanup, data saving, etc.
     *
     * @param ctx Pointer to the shared context.
     */
    virtual void exit(Context *ctx)    = 0;

    /**
     * @brief Helper to get the pointer to the current instance.
     *
     * @return state_t<Context>* Pointer to this.
     */
    state_t<Context> *get_instance()
    {
        return this;
    }

  protected:
    /* Protected Methods - API -----------------------------------------------*/
    /**
     * @brief [Child API] Request a switch to another state.
     *
     * Child states call this to "raise a hand" for transition.
     * The parent FSM will fetch this request later.
     *
     * @param next Pointer to the target state.
     */
    void request_switch(state_t<Context> *next)
    {
        _requested_state = next;
    }

  private:
    /* Private Methods - Internal --------------------------------------------*/
    /**
     * @brief [Parent API] Fetch and consume the switch request.
     *
     * Reads `_requested_state` and resets it to nullptr.
     *
     * @return state_t<Context>* Pointer to the requested state, or nullptr.
     */
    state_t<Context> *fetch_request()
    {
        state_t<Context> *next = _requested_state;
        _requested_state       = nullptr;
        return next;
    }

    /**
     * @brief [Parent API] Discard any stale request.
     *
     * Called when a state is forcibly exited to prevent "Zombie Requests"
     * upon re-entry.
     */
    void discard_request()
    {
        _requested_state = nullptr;
    }

    // Grant FSM access to private methods
    friend class fsm_t<Context>;
};

/* Class Definition ----------------------------------------------------------*/

/**
 * @brief Finite State Machine (FSM) Controller Class.
 *
 * Acts as both a State (can be nested) and a Manager (manages children).
 * Implements the Template Method Pattern to enforce lifecycle order.
 *
 * @tparam Context The shared data context type.
 */
template <typename Context> class fsm_t : public state_t<Context>
{
  protected:
    /* Protected Members -----------------------------------------------------*/
    /**
     * @brief The currently active child state.
     */
    state_t<Context> *_active_state = nullptr;

    /**
     * @brief The target state decided for the next frame (Transition Buffer).
     *
     * Derived FSMs set this directly in on_execute() to command a switch.
     */
    state_t<Context> *_target_state = nullptr;

  public:
    /* Public Methods - Sealed Lifecycle (Final) -----------------------------*/
    // Note: These methods are final to enforce the framework's flow control.

    /**
     * @brief Standard FSM Entry Logic (Sealed).
     *
     * Execution Order: Derived Hook -> Child State Enter.
     *
     * @param ctx Pointer to the shared context.
     */
    void enter(Context *ctx) final
    {
        on_enter(ctx);
        if (_active_state)
        {
            _active_state->enter(ctx);
        }
    }

    /**
     * @brief Standard FSM Execution Logic (Sealed) - Clean State Tick.
     *
     * Implements strict separation between "Logic Execution" and "State
     * Transition". A frame executes either logic OR a transition, never both.
     *
     * Pipeline:
     * 1. Process Pending Switch? -> YES: Return.
     * 2. Run Parent Logic. Target set? -> YES: Return.
     * 3. Run Child Logic. Child requested? -> YES: Return.
     *
     * @param ctx Pointer to the shared context.
     */
    void execute(Context *ctx) final
    {
        // 1. Phase A: Transition Processing
        // If a target was set in the previous frame, execute the switch NOW.
        if (process_switch(ctx))
            return;

        // 2. Phase B: Logic Execution

        // --- Parent Logic ---
        // Derived FSM observes Context. If it decides to switch, it sets
        // _target_state.
        on_execute(ctx);

        // Checkpoint: Did Parent logic set a target?
        if (_target_state)
            return;

        if (!_active_state)
            return;

        // --- Child Logic ---
        _active_state->execute(ctx);

        // Sync: Fetch request from child
        if (auto req = _active_state->fetch_request())
        {
            _target_state = req;
        }
    }

    /**
     * @brief Standard FSM Exit Logic (Sealed).
     *
     * Execution Order: Child State Exit -> Clean Stale Request -> Derived Hook.
     *
     * @param ctx Pointer to the shared context.
     */
    void exit(Context *ctx) final
    {
        if (_active_state)
        {
            _active_state->exit(ctx);
            // Critical: Clean up child's garbage request before leaving
            _active_state->discard_request();
        }
        on_exit(ctx);
    }

    void change_state(state_t<Context> *next)
    {
        _target_state = next;
    }

  protected:
    /* Protected Methods - Hooks (NVI Pattern) -------------------------------*/

    /**
     * @brief [Hook] Custom logic when entering the state.
     * @param ctx Pointer to the shared context.
     */
    virtual void on_enter(Context *ctx)
    {
    }

    /**
     * @brief [Hook] Custom logic when exiting the state.
     * @param ctx Pointer to the shared context.
     */
    virtual void on_exit(Context *ctx)
    {
    }

    /**
     * @brief [Hook] Custom execution logic (Parent Decision Layer).
     * @param ctx Pointer to the shared context.
     */
    virtual void on_execute(Context *ctx)
    {
    }

  private:
    /* Private Methods - Internal Processing ---------------------------------*/

    /**
     * @brief Unified Switch Processor.
     *
     * Consumes _target_state and performs Exit->Swap->Enter.
     *
     * @param ctx Pointer to the shared context.
     * @return true if a switch occurred, false otherwise.
     */
    bool process_switch(Context *ctx)
    {
        // 1. Guard: No target?
        if (!_target_state)
            return false;

        // 2. Guard: Self-switch? (Ignore)
        if (_target_state == _active_state)
        {
            _target_state = nullptr;
            return false;
        }

        // 3. Exit Old
        if (_active_state)
        {
            _active_state->exit(ctx);
            // Safety: Discard any pending request from the old state
            _active_state->discard_request();
        }

        // 4. Swap
        _active_state = _target_state;

        // 5. Enter New
        if (_active_state)
        {
            _active_state->enter(ctx);
        }

        // 6. Reset Buffer
        _target_state = nullptr;

        return true;
    }
};

} // namespace pyro

#endif // __PYRO_CORE_FSM_H__