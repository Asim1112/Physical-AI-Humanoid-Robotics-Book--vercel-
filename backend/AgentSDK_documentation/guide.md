Realtime Agents Quickstart

Realtime agents enable voice conversations with your AI agents using OpenAI's Realtime API.

Beta feature: Realtime agents are currently in beta. Expect breaking changes as the implementation evolves.


Prerequisites

Python 3.9 or higher

OpenAI API key

Basic familiarity with the OpenAI Agents SDK



Creating Your First Realtime Agent

1. Import Required Components
import asyncio
from agents.realtime import RealtimeAgent, RealtimeRunner

2. Create a Realtime Agent
agent = RealtimeAgent(
    name="Assistant",
    instructions="You are a helpful voice assistant. Keep your responses conversational and friendly.",
)


3. Set Up the Runner
runner = RealtimeRunner(
    starting_agent=agent,
    config={
        "model_settings": {
            "model_name": "gpt-realtime",
            "voice": "ash",
            "modalities": ["audio"],
            "input_audio_format": "pcm16",
            "output_audio_format": "pcm16",
            "input_audio_transcription": {"model": "gpt-4o-mini-transcribe"},
            "turn_detection": {"type": "semantic_vad", "interrupt_response": True},
        }
    }
)



4. Start a Session & Process Events
# Start the session
session = await runner.run()

async with session:
    print("Session started! The agent will stream audio responses in real-time.")
    
    async for event in session:
        try:
            if event.type == "agent_start":
                print(f"Agent started: {event.agent.name}")
            elif event.type == "agent_end":
                print(f"Agent ended: {event.agent.name}")
            elif event.type == "handoff":
                print(f"Handoff from {event.from_agent.name} to {event.to_agent.name}")
            elif event.type == "tool_start":
                print(f"Tool started: {event.tool.name}")
            elif event.type == "tool_end":
                print(f"Tool ended: {event.tool.name}; output: {event.output}")
            elif event.type == "audio_end":
                print("Audio ended")
            elif event.type == "audio":
                # Enqueue audio for callback-based playback with metadata
                pass
            elif event.type == "audio_interrupted":
                print("Audio interrupted")
            elif event.type == "error":
                print(f"Error: {event.error}")
            elif event.type == "history_updated":
                pass  # Frequent events, can skip
            elif event.type == "history_added":
                pass  # Frequent events, can skip
            elif event.type == "raw_model_event":
                print(f"Raw model event: {_truncate_str(str(event.data), 200)}")
            else:
                print(f"Unknown event type: {event.type}")
        except Exception as e:
            print(f"Error processing event: {_truncate_str(str(e), 200)}")

def _truncate_str(s: str, max_length: int) -> str:
    if len(s) > max_length:
        return s[:max_length] + "..."
    return s


## Complete Working Example

import asyncio
from agents.realtime import RealtimeAgent, RealtimeRunner

async def main():
    agent = RealtimeAgent(
        name="Assistant",
        instructions="You are a helpful voice assistant. Keep responses brief and conversational.",
    )
    runner = RealtimeRunner(
        starting_agent=agent,
        config={
            "model_settings": {
                "model_name": "gpt-realtime",
                "voice": "ash",
                "modalities": ["audio"],
                "input_audio_format": "pcm16",
                "output_audio_format": "pcm16",
                "input_audio_transcription": {"model": "gpt-4o-mini-transcribe"},
                "turn_detection": {"type": "semantic_vad", "interrupt_response": True},
            }
        },
    )
    session = await runner.run()

    async with session:
        print("Session started! The agent will stream audio responses in real-time.")
        async for event in session:
            try:
                if event.type == "agent_start":
                    print(f"Agent started: {event.agent.name}")
                elif event.type == "agent_end":
                    print(f"Agent ended: {event.agent.name}")
                elif event.type == "handoff":
                    print(f"Handoff from {event.from_agent.name} to {event.to_agent.name}")
                elif event.type == "tool_start":
                    print(f"Tool started: {event.tool.name}")
                elif event.type == "tool_end":
                    print(f"Tool ended: {event.tool.name}; output: {event.output}")
                elif event.type == "audio_end":
                    print("Audio ended")
                elif event.type == "audio":
                    pass
                elif event.type == "audio_interrupted":
                    print("Audio interrupted")
                elif event.type == "error":
                    print(f"Error: {event.error}")
                elif event.type in ["history_updated", "history_added"]:
                    pass
                elif event.type == "raw_model_event":
                    print(f"Raw model event: {_truncate_str(str(event.data), 200)}")
                else:
                    print(f"Unknown event type: {event.type}")
            except Exception as e:
                print(f"Error processing event: {_truncate_str(str(e), 200)}")

def _truncate_str(s: str, max_length: int) -> str:
    if len(s) > max_length:
        return s[:max_length] + "..."
    return s

if __name__ == "__main__":
    asyncio.run(main())


## Configuration Options

### Model Settings

model_name: Choose available realtime models (e.g., gpt-realtime)

voice: Select voice (alloy, echo, fable, onyx, nova, shimmer)

modalities: Enable "text" or "audio"

### Audio Settings

input_audio_format: pcm16, g711_ulaw, g711_alaw

output_audio_format: format for output audio

input_audio_transcription: Transcription configuration

### Turn Detection

type: server_vad or semantic_vad

threshold: voice activity threshold (0.0–1.0)

silence_duration_ms: silence duration to detect turn end

prefix_padding_ms: audio padding before speech

### Next Steps

Learn more about realtime agents

Check examples in examples/realtime

Add tools to your agent

Implement handoffs between agents

Set up guardrails for safety

### Authentication

Set your OpenAI API key in the environment:

export OPENAI_API_KEY="your-api-key-here"


Or pass it directly when running the session:

session = await runner.run(model_config={"api_key": "your-api-key"})
