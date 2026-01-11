from pydantic import BaseModel, Field, validator
from typing import Optional, List, Literal
from uuid import UUID
from datetime import datetime


class ChatRequest(BaseModel):
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question about the textbook"
    )
    session_id: Optional[UUID] = Field(
        None,
        description="Session ID for multi-turn conversations (omit for new session)"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=5000,
        description="Highlighted text from textbook for contextual queries"
    )
    stream: bool = Field(
        False,
        description="Enable streaming response (not implemented in MVP)"
    )

    @validator('query')
    def query_must_not_be_empty(cls, v):
        if not v.strip():
            raise ValueError('Query cannot be empty or whitespace-only')
        return v.strip()

    @validator('selected_text')
    def truncate_selected_text(cls, v):
        if v and len(v) > 5000:
            return v[:5000]
        return v


class ChatResponse(BaseModel):
    response: str = Field(..., description="Agent's response to the query")
    session_id: UUID = Field(..., description="Session ID (newly created or provided)")
    message_id: UUID = Field(..., description="Unique message identifier")
    retrieved_chunks: int = Field(..., ge=0, description="Number of textbook chunks retrieved")
    response_time_ms: float = Field(..., ge=0, description="Response time in milliseconds")
    error: Optional[str] = Field(None, description="Error message if generation failed")


class MessageSummary(BaseModel):
    message_id: UUID
    role: Literal['user', 'assistant']
    content: str
    timestamp: datetime
    selected_text: Optional[str] = None


class SessionResponse(BaseModel):
    session_id: UUID
    created_at: datetime
    last_accessed: datetime
    message_count: int
    messages: List[MessageSummary]


class ErrorResponse(BaseModel):
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Additional error context")
    request_id: Optional[str] = Field(None, description="Request ID for tracing")
    timestamp: datetime = Field(default_factory=datetime.utcnow)