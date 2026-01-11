"""Utility functions for RAG ingestion pipeline."""

import json
import logging
import os
import random
import sys
import time
from datetime import datetime
from functools import wraps
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from dotenv import load_dotenv


# ==============================================================================
# Logging Setup (T010)
# ==============================================================================

def setup_logging(log_level: str = "INFO", log_file: Optional[str] = None) -> logging.Logger:
    """
    Configure structured logging with JSON format for file and human-readable for console.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
        log_file: Optional log file path for JSON output

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger("rag_ingestion")
    logger.setLevel(getattr(logging, log_level.upper()))

    # Remove existing handlers
    logger.handlers.clear()

    # Console handler (human-readable)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)

    # File handler (JSON format)
    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(JSONFormatter())
        logger.addHandler(file_handler)

    return logger


class JSONFormatter(logging.Formatter):
    """Custom formatter for structured JSON logging."""

    def format(self, record: logging.LogRecord) -> str:
        log_data = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "message": record.getMessage(),
        }

        # Add extra fields if present
        if hasattr(record, "stage"):
            log_data["stage"] = record.stage
        if hasattr(record, "metadata"):
            log_data["metadata"] = record.metadata

        return json.dumps(log_data)


# ==============================================================================
# Environment Variable Loading (T011)
# ==============================================================================

def load_environment_variables(env_file: str = ".env") -> None:
    """
    Load environment variables from .env file.

    Args:
        env_file: Path to .env file (relative to backend/)

    Raises:
        FileNotFoundError: If .env file doesn't exist
    """
    env_path = Path(__file__).parent / env_file

    if not env_path.exists():
        raise FileNotFoundError(
            f"Environment file not found: {env_path}\n"
            f"Copy .env.example to .env and configure your API keys."
        )

    load_dotenv(dotenv_path=env_path)


# ==============================================================================
# Exponential Backoff Retry Decorator (T012)
# ==============================================================================

def retry_with_exponential_backoff(
    max_retries: int = 3,
    base_delay: float = 1.0,
    max_delay: float = 60.0,
    exponential_base: float = 2.0,
    jitter: bool = True,
    retryable_exceptions: tuple = (Exception,)
):
    """
    Decorator for retrying functions with exponential backoff and jitter.

    Args:
        max_retries: Maximum number of retry attempts
        base_delay: Initial delay in seconds
        max_delay: Maximum delay cap in seconds
        exponential_base: Base for exponential calculation
        jitter: Add randomness to delay to avoid thundering herd
        retryable_exceptions: Tuple of exception types to retry on

    Returns:
        Decorated function with retry logic

    Example:
        @retry_with_exponential_backoff(max_retries=3)
        def call_api():
            response = requests.get("https://api.example.com")
            response.raise_for_status()
            return response.json()
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            logger = logging.getLogger("rag_ingestion")

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except retryable_exceptions as e:
                    if attempt == max_retries:
                        logger.error(
                            f"{func.__name__} failed after {max_retries} retries: {e}"
                        )
                        raise

                    # Calculate delay with exponential backoff
                    delay = min(base_delay * (exponential_base ** attempt), max_delay)

                    # Add jitter (±50% randomness)
                    if jitter:
                        delay = delay * (0.5 + random.random())

                    logger.warning(
                        f"{func.__name__} attempt {attempt + 1} failed: {e}. "
                        f"Retrying in {delay:.2f}s..."
                    )
                    time.sleep(delay)

            # This should never be reached due to raise in loop
            raise RuntimeError(f"{func.__name__} exhausted all retries")

        return wrapper
    return decorator


# ==============================================================================
# API Key Validation (T013)
# ==============================================================================

def validate_api_keys(required_vars: Optional[List[str]] = None) -> Dict[str, str]:
    """
    Validate that all required environment variables are set.

    Args:
        required_vars: List of required environment variable names.
                      If None, uses default set: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY

    Returns:
        Dictionary of environment variable names to values

    Raises:
        ValueError: If any required environment variable is missing or empty

    Example:
        env_vars = validate_api_keys()
        cohere_key = env_vars["COHERE_API_KEY"]
    """
    if required_vars is None:
        required_vars = [
            "COHERE_API_KEY",
            "QDRANT_URL",
            "QDRANT_API_KEY",
        ]

    missing_vars = []
    empty_vars = []
    env_values = {}

    for var in required_vars:
        value = os.getenv(var)
        if value is None:
            missing_vars.append(var)
        elif not value.strip():
            empty_vars.append(var)
        else:
            env_values[var] = value

    errors = []
    if missing_vars:
        errors.append(f"Missing environment variables: {', '.join(missing_vars)}")
    if empty_vars:
        errors.append(f"Empty environment variables: {', '.join(empty_vars)}")

    if errors:
        raise ValueError(
            "\n".join(errors) + "\n\n"
            "Please set these variables in backend/.env\n"
            "See backend/.env.example for template"
        )

    return env_values


def redact_api_key(api_key: str, visible_chars: int = 6) -> str:
    """
    Redact API key for safe logging (show first N chars only).

    Args:
        api_key: Full API key
        visible_chars: Number of characters to show

    Returns:
        Redacted string (e.g., "sk_tes...")
    """
    if len(api_key) <= visible_chars:
        return "***"
    return api_key[:visible_chars] + "..."
