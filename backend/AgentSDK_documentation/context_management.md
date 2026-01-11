# Context Management

Context is a broad concept in Agents SDK. There are **two main types** of context:

1. **Local context** – available to your code during tool execution, lifecycle hooks, and callbacks.
2. **Agent/LLM context** – visible to the LLM when generating responses.

---

## Local Context

Local context is represented via the `RunContextWrapper` class and its `context` property.

### How it works:

* Create any Python object (commonly a dataclass or Pydantic model).
* Pass it to run methods like `Runner.run(..., context=...)`.
* Tool calls, lifecycle hooks, etc., receive a wrapper object `RunContextWrapper[T]`.
* Every agent/tool/lifecycle call in a run must use the **same context type**.

### Common uses:

* Contextual data (e.g., username, uid)
* Dependencies (e.g., loggers, data fetchers)
* Helper functions

> **Note:** Local context is **not sent to the LLM**. It's purely for internal use.

```python
import asyncio
from dataclasses import dataclass
from agents import Agent, RunContextWrapper, Runner, function_tool

@dataclass
class UserInfo:  
    name: str
    uid: int

@function_tool
async def fetch_user_age(wrapper: RunContextWrapper[UserInfo]) -> str:  
    return f"The user {wrapper.context.name} is 47 years old"

async def main():
    user_info = UserInfo(name="John", uid=123)

    agent = Agent[UserInfo](  
        name="Assistant",
        tools=[fetch_user_age],
    )

    result = await Runner.run(  
        starting_agent=agent,
        input="What is the age of the user?",
        context=user_info,
    )

    print(result.final_output)  # The user John is 47 years old.

if __name__ == "__main__":
    asyncio.run(main())
```

---

## Advanced: ToolContext

For extra metadata during tool execution (e.g., tool name, call ID, raw args), use the `ToolContext` class, which extends `RunContextWrapper`.

```python
from typing import Annotated
from pydantic import BaseModel, Field
from agents import Agent, Runner, function_tool
from agents.tool_context import ToolContext

class WeatherContext(BaseModel):
    user_id: str

class Weather(BaseModel):
    city: str = Field(description="The city name")
    temperature_range: str = Field(description="Temperature range in Celsius")
    conditions: str = Field(description="Weather conditions")

@function_tool
def get_weather(ctx: ToolContext[WeatherContext], city: Annotated[str, "The city to get the weather for"]) -> Weather:
    print(f"[debug] Tool context: (name: {ctx.tool_name}, call_id: {ctx.tool_call_id}, args: {ctx.tool_arguments})")
    return Weather(city=city, temperature_range="14-20C", conditions="Sunny with wind.")

agent = Agent(
    name="Weather Agent",
    instructions="You are a helpful agent that can tell the weather of a given city.",
    tools=[get_weather],
)
```

### ToolContext properties:

* `.context` – the user-provided context object
* `tool_name` – name of the tool being invoked
* `tool_call_id` – unique ID for this tool call
* `tool_arguments` – raw argument string passed to the tool

> Use `ToolContext` for **tool-level metadata**. For general sharing, `RunContextWrapper` is sufficient.

---

## Agent / LLM Context

LLMs only see **conversation history**. To make data available to the LLM:

1. **Agent instructions** – static or dynamic system prompts (e.g., user name, current date).
2. **Input to Runner.run** – add messages lower in the hierarchy.
3. **Function tools** – on-demand data retrieval the LLM can invoke.
4. **Retrieval or web search tools** – fetch context from files, databases, or the web to ground responses.
