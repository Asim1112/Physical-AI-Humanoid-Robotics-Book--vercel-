# Models

The Agents SDK comes with out-of-the-box support for OpenAI models in two flavors:

- **Recommended**: the `OpenAIResponsesModel`, which calls OpenAI APIs using the new Responses API.
- The `OpenAIChatCompletionsModel`, which calls OpenAI APIs using the Chat Completions API.

---

## OpenAI Models

When you don't specify a model when initializing an Agent, the default model will be used. The default is currently `gpt-4.1` for compatibility and low latency. If you have access, we recommend setting your agents to `gpt-5.2` for higher quality while keeping explicit `model_settings`.

If you want to switch to other models like `gpt-5.2`, follow the steps in the next section.

---

### Default OpenAI Model

If you want to consistently use a specific model for all agents that do not set a custom model, set the `OPENAI_DEFAULT_MODEL` environment variable before running your agents:

```bash
export OPENAI_DEFAULT_MODEL=gpt-5
python3 my_awesome_agent.py


GPT-5 Models

When you use any of GPT-5's reasoning models (gpt-5, gpt-5-mini, or gpt-5-nano) this way, the SDK applies sensible ModelSettings by default. Specifically, it sets both reasoning.effort and verbosity to "low".

If you want to build these settings yourself:

from openai.types.shared import Reasoning
from agents import Agent, ModelSettings

my_agent = Agent(
    name="My Agent",
    instructions="You're a helpful agent.",
    model_settings=ModelSettings(reasoning=Reasoning(effort="minimal"), verbosity="low")
    # If OPENAI_DEFAULT_MODEL=gpt-5 is set, passing only model_settings works.
    # It's also fine to pass a GPT-5 model name explicitly:
    # model="gpt-5",
)
Specifically for lower latency, using either gpt-5-mini or gpt-5-nano with reasoning.effort="minimal" will often return responses faster than the default settings.
Some built-in tools (file search, image generation) in the Responses API do not support "minimal" reasoning effort, which is why the SDK defaults to "low".

Non-GPT-5 Models

If you pass a non–GPT-5 model name without custom model_settings, the SDK reverts to generic ModelSettings compatible with any model.

Non-OpenAI Models

You can use most other non-OpenAI models via the LiteLLM integration. First, install the dependency:

pip install "openai-agents[litellm]"


Then, use any of the supported models with the litellm/ prefix:

claude_agent = Agent(model="litellm/anthropic/claude-3-5-sonnet-20240620", ...)
gemini_agent = Agent(model="litellm/gemini/gemini-2.5-flash-preview-04-17", ...)



Other Ways to Use Non-OpenAI Models

You can integrate other LLM providers in three more ways:

set_default_openai_client: Use an instance of AsyncOpenAI globally as the LLM client. Useful if the LLM provider has an OpenAI-compatible API endpoint.

ModelProvider: Set at the Runner.run level to use a custom model provider for all agents in that run.

Agent.model: Specify the model on a specific Agent instance, allowing mixing different providers.

Note: For LLMs without an OpenAI API key, disable tracing via set_tracing_disabled() or set up a different tracing processor.


Mixing and Matching Models

You can use different models for each agent in a workflow. Examples:

from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel
import asyncio

spanish_agent = Agent(
    name="Spanish agent",
    instructions="You only speak Spanish.",
    model="gpt-5-mini", 
)

english_agent = Agent(
    name="English agent",
    instructions="You only speak English",
    model=OpenAIChatCompletionsModel( 
        model="gpt-5-nano",
        openai_client=AsyncOpenAI()
    ),
)

triage_agent = Agent(
    name="Triage agent",
    instructions="Handoff to the appropriate agent based on the language of the request.",
    handoffs=[spanish_agent, english_agent],
    model="gpt-5",
)

async def main():
    result = await Runner.run(triage_agent, input="Hola, ¿cómo estás?")
    print(result.final_output)




Common Issues with Other LLM Providers
Tracing Client Error 401

Options to resolve:

Disable tracing: set_tracing_disabled(True).

Set an OpenAI API key for tracing: set_tracing_export_api_key(...).

Use a non-OpenAI trace processor.

Responses API Support

Most other LLM providers don’t yet support the Responses API. To resolve 404 errors:

Call set_default_openai_api("chat_completions").

Use OpenAIChatCompletionsModel.

Structured Outputs Support

Some providers don’t support structured outputs, which can result in errors:

BadRequestError: Error code: 400 - {'error': {'message': "'response_format.type' : value is not one of the allowed values ['text','json_object']", 'type': 'invalid_request_error'}}


Use providers with JSON schema support to avoid malformed output.

Mixing Models Across Providers

Be aware of feature differences (structured outputs, multimodal input, hosted file/web search).

Don’t send unsupported tools or multimodal inputs to text-only providers.

Providers lacking structured JSON output may occasionally produce invalid JSON.


---

If you want, I can also create a **more compact, visually clean Markdown version with nested bullet points and collapsible code blocks** for easier reading in docs or GitHub.  

Do you want me to do that?
