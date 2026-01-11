# Runner in Agents SDK

The **Runner** is a core utility for executing workflows in the Agents SDK. It orchestrates the execution of an agent, handling loops, handoffs, tool calls, and final output production.

---

## 1. `run` Method (Async)

Runs a workflow starting at a given agent asynchronously.

```python
run(
    starting_agent: Agent[TContext],
    input: str | list[TResponseInputItem],
    *,
    context: TContext | None = None,
    max_turns: int = DEFAULT_MAX_TURNS,
    hooks: RunHooks[TContext] | None = None,
    run_config: RunConfig | None = None,
    previous_response_id: str | None = None,
    auto_previous_response_id: bool = False,
    conversation_id: str | None = None,
    session: Session | None = None,
) -> RunResult
```

### Behavior

* Invokes the agent with the given input.
* Continues looping until:

  * The agent produces a final output (`agent.output_type`).
  * Or a handoff occurs, in which case a new agent loop starts.
  * Otherwise, tool calls are executed and the loop continues.
* Exceptions:

  * `MaxTurnsExceeded` if `max_turns` is exceeded.
  * `GuardrailTripwireTriggered` if a guardrail triggers.

> **Note:** Only the first agent's input guardrails are executed.

### Key Parameters

| Name                 | Type                           | Description                    | Default           |
| -------------------- | ------------------------------ | ------------------------------ | ----------------- |
| starting_agent       | Agent[TContext]                | Agent to run                   | required          |
| input                | str / list[TResponseInputItem] | Initial input                  | required          |
| context              | TContext                       | Context object                 | None              |
| max_turns            | int                            | Max AI turns (including tools) | DEFAULT_MAX_TURNS |
| hooks                | RunHooks[TContext]             | Lifecycle callbacks            | None              |
| run_config           | RunConfig                      | Global settings                | None              |
| previous_response_id | str                            | For OpenAI Responses API       | None              |
| conversation_id      | str                            | Conversation ID for history    | None              |
| session              | Session                        | Manages conversation history   | None              |

### Returns

`RunResult`: Contains all inputs, guardrail results, and output of the last agent.

---

## 2. `run_sync` Method (Sync)

Runs a workflow synchronously. Wraps `run` and **cannot** be used inside an existing event loop (e.g., Jupyter, FastAPI).

```python
run_sync(
    starting_agent: Agent[TContext],
    input: str | list[TResponseInputItem],
    *,
    context: TContext | None = None,
    max_turns: int = DEFAULT_MAX_TURNS,
    hooks: RunHooks[TContext] | None = None,
    run_config: RunConfig | None = None,
    previous_response_id: str | None = None,
    auto_previous_response_id: bool = False,
    conversation_id: str | None = None,
    session: Session | None = None,
) -> RunResult
```

> Behavior and parameters are identical to `run`.

---

## 3. `run_streamed` Method

Runs an agent workflow in **streaming mode**.

```python
run_streamed(
    starting_agent: Agent[TContext],
    input: str | list[TResponseInputItem],
    context: TContext | None = None,
    max_turns: int = DEFAULT_MAX_TURNS,
    hooks: RunHooks[TContext] | None = None,
    run_config: RunConfig | None = None,
    previous_response_id: str | None = None,
    auto_previous_response_id: bool = False,
    conversation_id: str | None = None,
    session: Session | None = None,
) -> RunResultStreaming
```

* Returns a `RunResultStreaming` object that allows streaming semantic events as they are generated.
* Looping behavior is identical to `run`.
* Exceptions: `MaxTurnsExceeded` or `GuardrailTripwireTriggered`.

---

## 4. Run Configuration (`RunConfig`)

Dataclass that configures global settings for a run.

### Important Attributes

| Attribute                    | Type                  | Description                                              |
| ---------------------------- | --------------------- | -------------------------------------------------------- |
| model                        | str                   | Model to use for the entire run (overrides agent models) |
| model_provider               | ModelProvider         | Resolves model names, default `MultiProvider`            |
| model_settings               | ModelSettings         | Overrides agent-specific model settings                  |
| handoff_input_filter         | HandoffInputFilter    | Filters inputs for handoffs                              |
| nest_handoff_history         | bool                  | Wrap prior run history into a single assistant message   |
| handoff_history_mapper       | callable              | Custom mapping for handoff input history                 |
| input_guardrails             | list[InputGuardrail]  | Guardrails on initial input                              |
| output_guardrails            | list[OutputGuardrail] | Guardrails on final output                               |
| tracing_disabled             | bool                  | Disable tracing                                          |
| trace_include_sensitive_data | bool                  | Include sensitive info in traces                         |
| workflow_name                | str                   | Name of the workflow for tracing                         |
| trace_id                     | str                   | Custom trace ID                                          |
| group_id                     | str                   | Grouping ID for linked traces                            |
| trace_metadata               | dict                  | Additional metadata for traces                           |
| session_input_callback       | callable              | Custom function to combine session history and new input |
| call_model_input_filter      | callable              | Modify model input before calling the model              |

> These attributes allow fine-grained control over how agents run, trace, and manage conversation history.

---

**Summary:**
The `Runner` provides async, sync, and streamed execution of agents with full support for context, tools, handoffs, guardrails, and tracing. `RunConfig` enables global customization for model calls and workflow management.
