"""Markdown file discovery, parsing, and chunking for RAG ingestion."""

import logging
import re
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import tiktoken
from markdown_it import MarkdownIt

logger = logging.getLogger("rag_ingestion")


# ==============================================================================
# T024: DocumentChunk Dataclass
# ==============================================================================

@dataclass
class DocumentChunk:
    """
    Represents a chunk of document content with metadata.

    Attributes per data-model.md specification.
    """
    chunk_id: str
    source_file: str  # Relative path from frontend/docs/
    module_name: str
    section_heading: str
    chunk_index: int
    total_chunks: int
    text: str
    token_count: int
    overlap_text: str = ""
    created_at: str = field(default_factory=lambda: datetime.utcnow().isoformat() + "Z")

    def __post_init__(self):
        """Validate chunk after initialization."""
        if self.chunk_index >= self.total_chunks:
            raise ValueError(f"chunk_index ({self.chunk_index}) must be < total_chunks ({self.total_chunks})")

        # Token count validation (150-300 acceptable for Cohere API limit accounting for ~30% tokenizer variance)
        is_final_chunk = (self.chunk_index == self.total_chunks - 1)
        min_tokens = 100 if is_final_chunk else 150
        max_tokens = 300

        if self.token_count < min_tokens:
            logger.warning(
                f"Chunk {self.chunk_id} has {self.token_count} tokens (< {min_tokens}). "
                f"Final chunk: {is_final_chunk}"
            )
        elif self.token_count > max_tokens:
            logger.warning(f"Chunk {self.chunk_id} has {self.token_count} tokens (> {max_tokens})")


# ==============================================================================
# T020: Discover Markdown Files
# ==============================================================================

def discover_markdown_files(docs_path: str) -> List[Path]:
    """
    Recursively discover all markdown files in docs directory.

    Args:
        docs_path: Path to docs directory (e.g., "frontend/docs/")

    Returns:
        List of Path objects for .md and .mdx files

    Example:
        files = discover_markdown_files("../frontend/docs")
        print(f"Found {len(files)} markdown files")
    """
    docs_dir = Path(docs_path)

    if not docs_dir.exists():
        raise FileNotFoundError(f"Docs directory not found: {docs_dir}")

    # Glob for .md and .mdx files
    md_files = list(docs_dir.rglob("*.md"))
    mdx_files = list(docs_dir.rglob("*.mdx"))

    all_files = md_files + mdx_files
    all_files.sort()  # Sort for consistent ordering

    logger.info(f"Discovered {len(all_files)} markdown files in {docs_dir}")
    logger.debug(f"Files: {[str(f.relative_to(docs_dir.parent)) for f in all_files[:5]]}...")

    return all_files


# ==============================================================================
# T021: Parse Markdown
# ==============================================================================

def parse_markdown(file_path: Path) -> Tuple[Optional[dict], str, List[str]]:
    """
    Parse markdown file to extract frontmatter, content, and section headings.

    Args:
        file_path: Path to markdown file

    Returns:
        Tuple of (frontmatter_dict, content_text, section_headings)

    Example:
        frontmatter, content, headings = parse_markdown(Path("intro.md"))
        print(f"Sections: {headings}")
    """
    content = file_path.read_text(encoding="utf-8")

    # Extract frontmatter (YAML between --- delimiters)
    frontmatter = None
    frontmatter_pattern = re.compile(r'^---\s*\n(.*?)\n---\s*\n', re.DOTALL)
    match = frontmatter_pattern.match(content)

    if match:
        frontmatter_text = match.group(1)
        frontmatter = {}  # Simple parsing - just capture as dict
        for line in frontmatter_text.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                frontmatter[key.strip()] = value.strip()

        # Remove frontmatter from content
        content = frontmatter_pattern.sub('', content)

    # Extract section headings (# Header, ## Header, etc.)
    heading_pattern = re.compile(r'^(#{1,6})\s+(.+)$', re.MULTILINE)
    section_headings = [match.group(2).strip() for match in heading_pattern.finditer(content)]

    logger.debug(f"Parsed {file_path.name}: {len(section_headings)} headings, frontmatter: {frontmatter is not None}")

    return frontmatter, content, section_headings


# ==============================================================================
# T022: Chunk Text with Token-based Semantic Boundaries
# ==============================================================================

def chunk_text(
    text: str,
    target_tokens: int = 250,
    min_tokens: int = 150,
    max_tokens: int = 300,
    overlap_tokens: int = 30,
    encoding_name: str = "cl100k_base"
) -> List[Tuple[str, int, str]]:
    """
    Chunk text into segments of 150-300 tokens with semantic boundaries and overlap.

    Args:
        text: Text to chunk
        target_tokens: Target tokens per chunk (default: 250)
        min_tokens: Minimum acceptable tokens (default: 150)
        max_tokens: Maximum acceptable tokens (default: 300)
        overlap_tokens: Tokens to overlap between chunks (default: 30)
        encoding_name: Tiktoken encoding (cl100k_base approximates Cohere)

    Returns:
        List of (chunk_text, token_count, overlap_text) tuples

    Strategy:
        - Split at sentence/paragraph boundaries within token range
        - Add 30-token overlap from previous chunk
        - Preserve code blocks intact when possible
        - Conservative max of 300 to account for ~30% tokenizer variance with Cohere
    """
    if not text.strip():
        logger.warning("Empty text provided for chunking")
        return []

    try:
        tokenizer = tiktoken.get_encoding(encoding_name)
    except Exception as e:
        logger.error(f"Failed to load tokenizer: {e}")
        raise

    # Split text into paragraphs and code blocks
    segments = split_into_segments(text)

    chunks = []
    current_chunk = []
    current_tokens = 0
    previous_overlap = ""

    for segment in segments:
        segment_tokens = len(tokenizer.encode(segment))

        # If segment alone exceeds max, split it further
        if segment_tokens > max_tokens:
            # Split long segment (e.g., large code block)
            sub_chunks = split_large_segment(segment, max_tokens, tokenizer)
            for sub_chunk in sub_chunks:
                sub_tokens = len(tokenizer.encode(sub_chunk))
                if chunks:  # Add overlap from previous chunk
                    chunk_with_overlap = previous_overlap + "\n\n" + sub_chunk
                    token_count = len(tokenizer.encode(chunk_with_overlap))
                else:
                    chunk_with_overlap = sub_chunk
                    token_count = sub_tokens

                chunks.append((chunk_with_overlap, token_count, previous_overlap))
                previous_overlap = extract_overlap(sub_chunk, overlap_tokens, tokenizer)
            continue

        # Check if adding segment would exceed max
        would_exceed = current_tokens + segment_tokens > max_tokens

        if would_exceed and current_tokens >= min_tokens:
            # Finalize current chunk
            chunk_text = "\n\n".join(current_chunk)

            if chunks:  # Add overlap from previous chunk
                chunk_with_overlap = previous_overlap + "\n\n" + chunk_text
                token_count = len(tokenizer.encode(chunk_with_overlap))
            else:
                chunk_with_overlap = chunk_text
                token_count = current_tokens

            chunks.append((chunk_with_overlap, token_count, previous_overlap))

            # Extract overlap for next chunk
            previous_overlap = extract_overlap(chunk_text, overlap_tokens, tokenizer)

            # Start new chunk with current segment
            current_chunk = [segment]
            current_tokens = segment_tokens
        else:
            # Add to current chunk
            current_chunk.append(segment)
            current_tokens += segment_tokens

    # Add final chunk
    if current_chunk:
        chunk_text = "\n\n".join(current_chunk)

        if chunks:  # Add overlap from previous chunk
            chunk_with_overlap = previous_overlap + "\n\n" + chunk_text
            token_count = len(tokenizer.encode(chunk_with_overlap))
        else:
            chunk_with_overlap = chunk_text
            token_count = current_tokens

        chunks.append((chunk_with_overlap, token_count, previous_overlap))

    logger.debug(f"Created {len(chunks)} chunks from {len(tokenizer.encode(text))} tokens")

    return chunks


def split_into_segments(text: str) -> List[str]:
    """Split text into paragraphs and code blocks while preserving structure."""
    segments = []

    # Pattern to match code blocks (```...```)
    code_block_pattern = re.compile(r'```[\s\S]*?```', re.MULTILINE)

    last_end = 0
    for match in code_block_pattern.finditer(text):
        # Add text before code block
        before_code = text[last_end:match.start()].strip()
        if before_code:
            # Split by paragraphs
            paragraphs = [p.strip() for p in before_code.split('\n\n') if p.strip()]
            segments.extend(paragraphs)

        # Add code block as single segment
        segments.append(match.group(0))
        last_end = match.end()

    # Add remaining text
    remaining = text[last_end:].strip()
    if remaining:
        paragraphs = [p.strip() for p in remaining.split('\n\n') if p.strip()]
        segments.extend(paragraphs)

    return segments


def split_large_segment(segment: str, max_tokens: int, tokenizer) -> List[str]:
    """Split large segment (e.g., code block) at logical boundaries."""
    # Split by sentences for text, or by lines for code
    if segment.startswith('```'):
        # Code block - split by lines
        lines = segment.split('\n')
        chunks = []
        current = []
        current_tokens = 0

        for line in lines:
            line_tokens = len(tokenizer.encode(line))
            if current_tokens + line_tokens > max_tokens and current:
                chunks.append('\n'.join(current))
                current = [line]
                current_tokens = line_tokens
            else:
                current.append(line)
                current_tokens += line_tokens

        if current:
            chunks.append('\n'.join(current))

        return chunks
    else:
        # Text - split by sentences
        sentences = re.split(r'(?<=[.!?])\s+', segment)
        chunks = []
        current = []
        current_tokens = 0

        for sentence in sentences:
            sent_tokens = len(tokenizer.encode(sentence))
            if current_tokens + sent_tokens > max_tokens and current:
                chunks.append(' '.join(current))
                current = [sentence]
                current_tokens = sent_tokens
            else:
                current.append(sentence)
                current_tokens += sent_tokens

        if current:
            chunks.append(' '.join(current))

        return chunks


def extract_overlap(text: str, overlap_tokens: int, tokenizer) -> str:
    """Extract last N tokens from text for overlap with next chunk."""
    tokens = tokenizer.encode(text)
    if len(tokens) <= overlap_tokens:
        return text

    overlap_token_list = tokens[-overlap_tokens:]
    overlap_text = tokenizer.decode(overlap_token_list)
    return overlap_text


# ==============================================================================
# T023: Extract Metadata
# ==============================================================================

def extract_metadata(file_path: Path, docs_root: Path) -> dict:
    """
    Extract metadata from file path (module name, relative path, etc.).

    Args:
        file_path: Absolute path to markdown file
        docs_root: Root docs directory path

    Returns:
        Dictionary with module_name, relative_path, etc.

    Example:
        metadata = extract_metadata(
            Path("frontend/docs/module-1-ros2/intro.md"),
            Path("frontend/docs")
        )
        # Returns: {"module_name": "module-1-ros2", "relative_path": "module-1-ros2/intro.md"}
    """
    relative_path = file_path.relative_to(docs_root)

    # Extract module name from path (e.g., "module-1-ros2" from "module-1-ros2/intro.md")
    parts = relative_path.parts
    module_name = parts[0] if parts else "unknown"

    # Check if it matches module-N-name pattern
    module_pattern = re.compile(r'module-\d+-[\w-]+')
    if not module_pattern.match(module_name):
        logger.warning(f"Non-standard module name: {module_name} (expected module-N-name format)")

    metadata = {
        "module_name": module_name,
        "relative_path": str(relative_path),
        "file_name": file_path.name
    }

    logger.debug(f"Extracted metadata for {file_path.name}: {metadata}")

    return metadata


# ==============================================================================
# High-Level Chunking Function
# ==============================================================================

def process_file_to_chunks(file_path: Path, docs_root: Path) -> List[DocumentChunk]:
    """
    Process a single markdown file into DocumentChunk objects.

    Args:
        file_path: Path to markdown file
        docs_root: Root docs directory

    Returns:
        List of DocumentChunk objects

    This function orchestrates T021-T024:
        1. Parse markdown (T021)
        2. Extract metadata (T023)
        3. Chunk text (T022)
        4. Create DocumentChunk objects (T024)
    """
    # T021: Parse markdown
    frontmatter, content, headings = parse_markdown(file_path)

    # T046: Edge case handling - empty files
    if not content.strip():
        if frontmatter:
            logger.warning(f"Frontmatter-only file: {file_path.name}, skipping")
        else:
            logger.warning(f"Empty file: {file_path.name}, skipping")
        return []

    # T046: Edge case handling - very short content
    content_length = len(content.strip())
    if content_length < 50:  # Less than 50 chars
        logger.warning(
            f"File too short ({content_length} chars): {file_path.name}, "
            "may produce small chunks"
        )

    # T023: Extract metadata
    metadata = extract_metadata(file_path, docs_root)

    # T022: Chunk text
    try:
        chunk_tuples = chunk_text(content)
    except Exception as e:
        # T046: Edge case handling - chunking failures
        logger.error(f"Failed to chunk {file_path.name}: {e}")
        return []

    if not chunk_tuples:
        logger.warning(f"No chunks created for {file_path.name} (content may be too short)")
        return []

    # T046: Edge case handling - validate chunk quality
    if len(chunk_tuples) == 1:
        _, token_count_val, _ = chunk_tuples[0]
        if token_count_val < 100:
            logger.warning(
                f"Single chunk with {token_count_val} tokens from {file_path.name} "
                "(below minimum threshold)"
            )

    # T024: Create DocumentChunk objects
    chunks = []
    total_chunks = len(chunk_tuples)

    # Determine section heading (use first heading or filename)
    section_heading = headings[0] if headings else file_path.stem

    for idx, (text_content, token_count, overlap_text) in enumerate(chunk_tuples):
        chunk = DocumentChunk(
            chunk_id=str(uuid.uuid4()),
            source_file=metadata["relative_path"],
            module_name=metadata["module_name"],
            section_heading=section_heading,
            chunk_index=idx,
            total_chunks=total_chunks,
            text=text_content,
            token_count=token_count,
            overlap_text=overlap_text
        )
        chunks.append(chunk)

    logger.info(f"Created {len(chunks)} chunks from {file_path.name} ({len(content)} chars)")

    return chunks
