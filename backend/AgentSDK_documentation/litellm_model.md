# Using Any Model via LiteLLM

> **Note:**
> LiteLLM integration is currently in **beta**. Some model providers, especially smaller ones, may have issues. Please report problems via GitHub issues for quick fixes.

LiteLLM is a library that lets you access **100+ models via a single interface**. The Agents SDK integrates with LiteLLM to allow you to use any AI model seamlessly.

---

## Setup

Ensure `litellm` is installed by using the optional dependency group:

```bash
pip install "openai-agents[litellm]"
```

Once installed, you can use `LitellmModel` in any agent.

---

## Example Usage

This fully working example demonstrates creating an agent that uses LiteLLM. When running, you'll be prompted for:

* **Model name** (e.g., `openai/gpt-4.1` or `anthropic/claude-3-5-sonnet-20240620`)
* **API key** for the chosen provider

For a full list of LiteLLM-supported models, see the [LiteLLM providers documentation].

```python
from __future__ import annotations
import asyncio
from agents import Agent, Runner, function_tool
from agents.extensions.models.litellm_model import LitellmModel

@function_tool
def get_weather(city: str):
    print(f"[debug] getting weather for {city}")
    return f"The weather in {city} is sunny."

async def main(model: str, api_key: str):
    agent = Agent(
        name="Assistant",
        instructions="You only respond in haikus.",
        model=LitellmModel(model=model, api_key=api_key),
        tools=[get_weather],
    )

    result = await Runner.run(agent, "What's the weather in Tokyo?")
    print(result.final_output)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, required=False)
    parser.add_argument("--api-key", type=str, required=False)
    args = parser.parse_args()

    model = args.model or input("Enter a model name for LiteLLM: ")
    api_key = args.api_key or input("Enter an API key for LiteLLM: ")

    asyncio.run(main(model, api_key))
```

---

## Tracking Usage Data

To track LiteLLM usage metrics within the Agents SDK, pass `ModelSettings(include_usage=True)` when creating your agent. This allows token counts and request metrics to populate `result.context_wrapper.usage`, just like built-in OpenAI models.

```python
from agents import Agent, ModelSettings
from agents.extensions.models.litellm_model import LitellmModel

agent = Agent(
    name="Assistant",
    model=LitellmModel(model="your/model", api_key="..."),
    model_settings=ModelSettings(include_usage=True),
)
```

With `include_usage=True`, all LiteLLM requests report usage metrics transparently for monitoring or analytics.
