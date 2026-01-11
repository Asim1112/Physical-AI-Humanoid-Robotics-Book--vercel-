# Agents

Agents are the **core building block** in your applications. An **agent** is a Large Language Model (LLM) configured with **instructions**, **tools**, and optional **settings** to perform tasks intelligently.

---

## Basic Configuration

The most common properties of an agent you’ll configure are:

* **`name`**: A required string that uniquely identifies your agent.
* **`instructions`**: Also known as a *developer message* or *system prompt*.
* **`model`**: The LLM to use, with optional `model_settings` (e.g., `temperature`, `top_p`).
* **`tools`**: Functions or agents the agent can call to achieve its tasks.

### Example

```python
from agents import Agent, ModelSettings, function_tool

@function_tool
def get_weather(city: str) -> str:
    """Returns weather info for the specified city."""
    return f"The weather in {city} is sunny"

agent = Agent(
    name="Haiku agent",
    instructions="Always respond in haiku form",
    model="gpt-5-nano",
    tools=[get_weather],
)
```

---

## Context

Agents are **generic over their context type**. Context acts as a **dependency-injection container**:

* Created by you
* Passed to `Runner.run()`
* Available to **agents, tools, and handoffs**

You may pass **any Python object** as context.

### Example: Custom Context

```python
from dataclasses import dataclass

@dataclass
class UserContext:
    name: str
    uid: str
    is_pro_user: bool

    async def fetch_purchases() -> list[Purchase]:
        return ...

agent = Agent[UserContext](
    ...,
)
```

---

## Output Types

By default, agents return **plain text (`str`)**. You can enforce **structured output** using the `output_type` parameter.

Supported output types include:

* **Pydantic models**
* **Dataclasses**
* **Lists**
* **TypedDict**
* Any type compatible with **Pydantic TypeAdapter**

### Example: Structured Output

```python
from pydantic import BaseModel
from agents import Agent

class CalendarEvent(BaseModel):
    name: str
    date: str
    participants: list[str]

agent = Agent(
    name="Calendar extractor",
    instructions="Extract calendar events from text",
    output_type=CalendarEvent,
)
```

> **Note**
> When `output_type` is provided, the model produces **structured outputs** instead of plain text.

---

## Multi‑Agent System Design Patterns

There are two widely used multi-agent design patterns:

1. **Manager (Agents as Tools)** – Centralized orchestration
2. **Handoffs** – Decentralized delegation

For deeper guidance, see the *Practical Guide to Building Agents*.

---

## Manager Pattern (Agents as Tools)

A **customer-facing agent** controls the conversation and invokes specialized agents exposed as tools.

### Example

```python
from agents import Agent

booking_agent = Agent(...)
refund_agent = Agent(...)

customer_facing_agent = Agent(
    name="Customer-facing agent",
    instructions=(
        "Handle all direct user communication. "
        "Call the relevant tools when specialized expertise is needed."
    ),
    tools=[
        booking_agent.as_tool(
            tool_name="booking_expert",
            tool_description="Handles booking questions and requests.",
        ),
        refund_agent.as_tool(
            tool_name="refund_expert",
            tool_description="Handles refund questions and requests.",
        )
    ],
)
```

---

## Handoffs

**Handoffs** allow one agent to delegate the entire conversation to another specialized agent.

* The delegated agent receives full conversation history
* Control is fully transferred
* Enables modular, expert-focused agents

### Example

```python
from agents import Agent

booking_agent = Agent(...)
refund_agent = Agent(...)

triage_agent = Agent(
    name="Triage agent",
    instructions=(
        "Help the user with their questions. "
        "If they ask about booking, hand off to the booking agent. "
        "If they ask about refunds, hand off to the refund agent."
    ),
    handoffs=[booking_agent, refund_agent],
)
```

---

## Dynamic Instructions

Instead of static instructions, you can supply a **function** that dynamically generates instructions.

* Receives the **agent** and **context**
* Can be synchronous or asynchronous

### Example

```python
def dynamic_instructions(
    context: RunContextWrapper[UserContext], agent: Agent[UserContext]
) -> str:
    return f"The user's name is {context.context.name}. Help them with their questions."

agent = Agent[UserContext](
    name="Triage agent",
    instructions=dynamic_instructions,
)
```

---

## Lifecycle Events (Hooks)

Agents expose **lifecycle hooks** to observe or modify behavior during execution.

Use cases include:

* Logging
* Metrics collection
* Pre-fetching data

To use hooks:

* Subclass `AgentHooks`
* Override required lifecycle methods

---

## Guardrails

**Guardrails** allow validation checks to run:

* On **user input** (pre-execution)
* On **agent output** (post-execution)

They run **in parallel** with the agent and help enforce:

* Relevance
* Safety
* Policy compliance

> See the *Guardrails Documentation* for full details.

---

## Cloning / Copying Agents

Agents can be duplicated using the `clone()` method and selectively modified.

### Example

```python
pirate_agent = Agent(
    name="Pirate",
    instructions="Write like a pirate",
    model="gpt-5.2",
)

robot_agent = pirate_agent.clone(
    name="Robot",
    instructions="Write like a robot",
)
```

---

## Forcing Tool Use

Providing tools does **not guarantee** they will be used. Control tool usage via `ModelSettings.tool_choice`.

### Valid Values

* **`"auto"`** – Model decides (default)
* **`"required"`** – Model must use *a* tool
* **`"none"`** – Model must not use tools
* **Specific tool name** – Forces that exact tool

### Example

```python
from agents import Agent, function_tool, ModelSettings

@function_tool
def get_weather(city: str) -> str:
    """Returns weather info for the specified city."""
    return f"The weather in {city} is sunny"

agent = Agent(
    name="Weather Agent",
    instructions="Retrieve weather details.",
    tools=[get_weather],
    model_settings=ModelSettings(tool_choice="get_weather")
)
```

---

## Tool Use Behavior

The `tool_use_behavior` parameter controls how tool outputs are handled.

### Options

#### 1. `"run_llm_again"` (Default)

* Tools run
* LLM processes tool output
* Produces final response

```python
agent = Agent(
    name="Weather Agent",
    instructions="Retrieve weather details.",
    tools=[get_weather],
    tool_use_behavior="run_llm_again"
)
```

#### 2. `"stop_on_first_tool"`

* First tool output becomes final response
* No additional LLM processing

```python
agent = Agent(
    name="Weather Agent",
    instructions="Retrieve weather details.",
    tools=[get_weather],
    tool_use_behavior="stop_on_first_tool"
)
```

---

## StopAtTools

Stops execution if any specified tool is called and returns its output immediately.

### Example

```python
from agents.agent import StopAtTools

@function_tool
def sum_numbers(a: int, b: int) -> int:
    """Adds two numbers."""
    return a + b

agent = Agent(
    name="Stop At Stock Agent",
    instructions="Get weather or sum numbers.",
    tools=[get_weather, sum_numbers],
    tool_use_behavior=StopAtTools(stop_at_tool_names=["get_weather"])
)
```

---

## ToolsToFinalOutputFunction

A **custom handler** that inspects tool results and decides whether to:

* Stop execution
* Continue with the LLM

### Example

```python
from agents import function_tool, FunctionToolResult
from agents.agent import ToolsToFinalOutputResult
from typing import List, Any

@function_tool
def get_weather(city: str) -> str:
    """Returns weather info for the specified city."""
    return f"The weather in {city} is sunny"

def custom_tool_handler(
    context: RunContextWrapper[Any],
    tool_results: List[FunctionToolResult]
) -> ToolsToFinalOutputResult:
    for result in tool_results:
        if result.output and "sunny" in result.output:
            return ToolsToFinalOutputResult(
                is_final_output=True,
                final_output=f"Final weather: {result.output}"
            )

    return ToolsToFinalOutputResult(
        is_final_output=False,
        final_output=None
    )

agent = Agent(
    name="Weather Agent",
    instructions="Retrieve weather details.",
    tools=[get_weather],
    tool_use_behavior=custom_tool_handler
)
```
