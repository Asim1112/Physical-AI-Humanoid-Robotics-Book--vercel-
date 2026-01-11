# Tools

Tools enable agents to **take actions** beyond text generation—such as fetching data, executing code, calling external APIs, or even controlling a computer.

In the Agent SDK, tools fall into **three main categories**:

1. **Hosted Tools** – Run on LLM servers alongside the model
2. **Function Tools** – Any Python function exposed as a tool
3. **Agents as Tools** – Use agents themselves as callable tools

---

## Tool Categories Overview

### 1. Hosted Tools

Hosted tools are built-in tools provided by OpenAI when using the `OpenAIResponsesModel`.

Available hosted tools include:

* **WebSearchTool** – Search the web
* **FileSearchTool** – Retrieve information from OpenAI Vector Stores
* **ComputerTool** – Automate computer-use tasks
* **CodeInterpreterTool** – Execute code in a sandboxed environment
* **HostedMCPTool** – Expose a remote MCP server’s tools
* **ImageGenerationTool** – Generate images from prompts
* **LocalShellTool** – Run shell commands on your local machine

### Example: Using Hosted Tools

```python
from agents import Agent, FileSearchTool, Runner, WebSearchTool

agent = Agent(
    name="Assistant",
    tools=[
        WebSearchTool(),
        FileSearchTool(
            max_num_results=3,
            vector_store_ids=["VECTOR_STORE_ID"],
        ),
    ],
)

async def main():
    result = await Runner.run(
        agent,
        "Which coffee shop should I go to, taking into account my preferences and the weather today in SF?"
    )
    print(result.final_output)
```

---

## Function Tools

Any Python function can be turned into a tool. The Agents SDK automatically configures the tool using **introspection**.

### What Happens Automatically

* Tool name defaults to the Python function name (can be overridden)
* Tool description is extracted from the function docstring
* Input schema is inferred from function arguments
* Argument descriptions are parsed from docstrings

Internally, the SDK uses:

* `inspect` – function signature parsing
* `griffe` – docstring parsing
* `pydantic` – JSON schema generation

---

### Example: Function Tools

```python
from typing_extensions import TypedDict, Any
from agents import Agent, FunctionTool, RunContextWrapper, function_tool

class Location(TypedDict):
    lat: float
    long: float

@function_tool
async def fetch_weather(location: Location) -> str:
    """Fetch the weather for a given location.

    Args:
        location: The location to fetch the weather for.
    """
    return "sunny"

@function_tool(name_override="fetch_data")
def read_file(
    ctx: RunContextWrapper[Any],
    path: str,
    directory: str | None = None
) -> str:
    """Read the contents of a file.

    Args:
        path: The path to the file to read.
        directory: The directory to read the file from.
    """
    return "<file contents>"

agent = Agent(
    name="Assistant",
    tools=[fetch_weather, read_file],
)
```

---

### Inspecting Generated Tool Metadata

```python
import json

for tool in agent.tools:
    if isinstance(tool, FunctionTool):
        print(tool.name)
        print(tool.description)
        print(json.dumps(tool.params_json_schema, indent=2))
        print()
```

---

## Returning Images or Files from Tools

Function tools can return **more than text**.

Supported return types:

* **Text** – `str`, stringable objects, or `ToolOutputText`
* **Images** – `ToolOutputImage` or `ToolOutputImageDict`
* **Files** – `ToolOutputFileContent` or `ToolOutputFileContentDict`

This allows tools to generate rich outputs like charts, images, or downloadable files.

---

## Custom Function Tools

If you don’t want to use a Python function decorator, you can manually create a `FunctionTool`.

### Required Fields

* `name`
* `description`
* `params_json_schema`
* `on_invoke_tool` – async function handling the tool logic

### Example

```python
from typing import Any
from pydantic import BaseModel
from agents import RunContextWrapper, FunctionTool

class FunctionArgs(BaseModel):
    username: str
    age: int

async def run_function(ctx: RunContextWrapper[Any], args: str) -> str:
    parsed = FunctionArgs.model_validate_json(args)
    return f"{parsed.username} is {parsed.age} years old"

tool = FunctionTool(
    name="process_user",
    description="Processes extracted user data",
    params_json_schema=FunctionArgs.model_json_schema(),
    on_invoke_tool=run_function,
)
```

---

## Automatic Argument & Docstring Parsing

The SDK automatically extracts tool schemas and documentation:

* **Function signatures** are parsed using `inspect`
* **Type hints** define argument types
* **Docstrings** are parsed using `griffe`

### Supported Docstring Formats

* Google
* Sphinx
* NumPy

You can:

* Explicitly set docstring format
* Disable parsing with `use_docstring_info=False`

Schema generation logic lives in `agents.function_schema`.

---

## Agents as Tools

Agents themselves can be exposed as tools, allowing **orchestration without handoff**.

This is useful when:

* A central agent coordinates multiple specialists
* You want partial results from sub-agents
* Control must remain centralized

### Example: Agents as Translation Tools

```python
from agents import Agent, Runner

spanish_agent = Agent(
    name="Spanish agent",
    instructions="You translate the user's message to Spanish",
)

french_agent = Agent(
    name="French agent",
    instructions="You translate the user's message to French",
)

orchestrator_agent = Agent(
    name="orchestrator_agent",
    instructions=(
        "You are a translation agent. You use the tools given to you to translate."
        "If asked for multiple translations, you call the relevant tools."
    ),
    tools=[
        spanish_agent.as_tool(
            tool_name="translate_to_spanish",
            tool_description="Translate the user's message to Spanish",
        ),
        french_agent.as_tool(
            tool_name="translate_to_french",
            tool_description="Translate the user's message to French",
        ),
    ],
)

async def main():
    result = await Runner.run(
        orchestrator_agent,
        input="Say 'Hello, how are you?' in Spanish."
    )
    print(result.final_output)
```

---

## Customizing Tool-Agents

`agent.as_tool()` is a convenience method but has limitations (e.g., cannot set `max_turns`).

For advanced use cases, run agents manually inside a function tool.

```python
from agents import function_tool, Agent, Runner

@function_tool
async def run_my_agent() -> str:
    """A tool that runs the agent with custom configs"""

    agent = Agent(name="My agent", instructions="...")

    result = await Runner.run(
        agent,
        input="...",
        max_turns=5,
        run_config=...
    )

    return str(result.final_output)
```

---

## Custom Output Extraction

Sometimes you want to modify what a tool-agent returns.

Common use cases:

* Extract JSON from chat history
* Reformat output (Markdown → CSV)
* Validate or sanitize agent responses

### Example

```python
async def extract_json_payload(run_result: RunResult) -> str:
    for item in reversed(run_result.new_items):
        if isinstance(item, ToolCallOutputItem) and item.output.strip().startswith("{"):
            return item.output.strip()
    return "{}"

json_tool = data_agent.as_tool(
    tool_name="get_data_json",
    tool_description="Run the data agent and return only its JSON payload",
    custom_output_extractor=extract_json_payload,
)
```

---

## Streaming Nested Agent Runs

You can listen to streaming events from tool-agents using `on_stream`.

```python
from agents import AgentToolStreamEvent

async def handle_stream(event: AgentToolStreamEvent) -> None:
    print(f"[stream] {event['agent']['name']} :: {event['event'].type}")

billing_agent_tool = billing_agent.as_tool(
    tool_name="billing_helper",
    tool_description="Answer billing questions.",
    on_stream=handle_stream,
)
```

### What to Expect

* Event types mirror `StreamEvent["type"]`
* Nested agent runs in streaming mode automatically
* Events are delivered in order
* `tool_call_id` may be `None` for direct calls

See `examples/agent_patterns/agents_as_tools_streaming.py` for a full example.

---

## Conditional Tool Enabling

Tools can be enabled or disabled dynamically using `is_enabled`.

### Accepted Values

* **Boolean** – Always enabled/disabled
* **Callable** – `(context, agent) -> bool`
* **Async callable** – For complex logic

Disabled tools are **hidden from the LLM** at runtime.

### Example

```python
from agents import Agent, AgentBase, Runner, RunContextWrapper
from pydantic import BaseModel

class LanguageContext(BaseModel):
    language_preference: str = "french_spanish"

def french_enabled(ctx: RunContextWrapper[LanguageContext], agent: AgentBase) -> bool:
    return ctx.context.language_preference == "french_spanish"

spanish_agent = Agent(
    name="spanish_agent",
    instructions="You respond in Spanish.",
)

french_agent = Agent(
    name="french_agent",
    instructions="You respond in French.",
)

orchestrator = Agent(
    name="orchestrator",
    instructions=(
        "You are a multilingual assistant. You must always respond using tools."
    ),
    tools=[
        spanish_agent.as_tool(
            tool_name="respond_spanish",
            tool_description="Respond in Spanish",
            is_enabled=True,
        ),
        french_agent.as_tool(
            tool_name="respond_french",
            tool_description="Respond in French",
            is_enabled=french_enabled,
        ),
    ],
)
```

---

## Handling Errors in Function Tools

You can customize how tool errors are reported to the LLM using `failure_error_function`.

### Error Handling Options

* **Default behavior** – Sends a generic error to the LLM
* **Custom error function** – Returns a user-friendly message
* **Explicit `None`** – Errors are re-raised for manual handling

### Example

```python
from agents import function_tool, RunContextWrapper
from typing import Any

def my_custom_error_function(
    context: RunContextWrapper[Any], error: Exception
) -> str:
    print(f"Tool failed: {error}")
    return "An internal server error occurred. Please try again later."

@function_tool(failure_error_function=my_custom_error_function)
def get_user_profile(user_id: str) -> str:
    """Fetches a user profile from a mock API."""
    if user_id == "user_123":
        return "User profile retrieved successfully."
    raise ValueError(f"Could not retrieve profile for {user_id}.")
```

> **Note**
> When manually creating a `FunctionTool`, error handling must be implemented inside `on_invoke_tool`.
