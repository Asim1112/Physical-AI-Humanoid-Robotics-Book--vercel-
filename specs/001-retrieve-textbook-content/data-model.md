# Data Model: Textbook Content Retrieval System

**Feature**: 001-retrieve-textbook-content
**Date**: 2025-12-25

## Overview

Data model for the retrieval system that queries embedded textbook content from Qdrant vector database.

## Core Entities

### Query
**Description**: User input text that needs to be matched against embedded content
- **query_text**: string - The original user query text
- **query_embedding**: List[float] - 1024-dimensional embedding vector from Cohere
- **query_type**: string - Category of query (e.g., "ROS2", "Gazebo", "VLA", etc.)
- **created_at**: datetime - Timestamp when query was processed
- **embedding_model**: string - Model used for embedding (e.g., "embed-english-v3.0")

### RetrievedChunk
**Description**: Content segment returned from Qdrant with similarity score and metadata
- **chunk_id**: string - Unique identifier for the content chunk
- **similarity_score**: float - Cosine similarity score (0.0-1.0) between query and chunk
- **content_text**: string - The actual text content of the retrieved chunk
- **source_file**: string - Original markdown file path
- **module_name**: string - Module name derived from file path
- **section_heading**: string - Section heading from original content
- **chunk_index**: int - Position of chunk within original document
- **total_chunks**: int - Total number of chunks in original document
- **token_count**: int - Number of tokens in the chunk
- **retrieved_at**: datetime - Timestamp when chunk was retrieved

### RetrievalResult
**Description**: Collection of retrieved chunks for a single query
- **query_id**: string - Unique identifier for the query session
- **original_query**: Query - The original query object
- **retrieved_chunks**: List[RetrievedChunk] - Top-K most relevant chunks
- **retrieval_time_ms**: float - Time taken to perform retrieval in milliseconds
- **retrieval_timestamp**: datetime - When retrieval was performed
- **total_candidates**: int - Total number of candidates considered
- **search_params**: dict - Parameters used for the search (top_k, score_threshold, etc.)

### PerformanceMetric
**Description**: Measurement of retrieval performance for validation
- **metric_id**: string - Unique identifier for the metric
- **query_text**: string - The query that was measured
- **latency_ms**: float - Time taken to complete retrieval
- **relevance_score**: float - Average similarity score of top results
- **result_count**: int - Number of results returned
- **query_category**: string - Category of the query
- **timestamp**: datetime - When measurement was taken
- **success**: bool - Whether retrieval completed successfully

### ValidationError
**Description**: Information about validation failures
- **error_id**: string - Unique identifier for the error
- **validation_type**: string - Type of validation that failed
- **query_text**: string - Query that caused the validation error
- **expected_result**: string - What was expected
- **actual_result**: string - What was actually returned
- **error_message**: string - Description of the validation failure
- **timestamp**: datetime - When validation error occurred

## Relationships

- Query → RetrievalResult (one-to-many: one query produces one retrieval result)
- RetrievalResult → RetrievedChunk (one-to-many: one result contains multiple chunks)
- RetrievedChunk → PerformanceMetric (many-to-many: chunks contribute to performance metrics)

## Validation Rules

### Query Validation
- query_text must not be empty or whitespace only
- query_text must be less than 1000 characters (to prevent overly long queries)
- query_embedding must be 1024-dimensional (for Cohere embed-english-v3.0)

### RetrievedChunk Validation
- similarity_score must be between 0.0 and 1.0
- content_text must not be empty
- chunk_id must be a valid UUID format
- token_count must be positive

### RetrievalResult Validation
- retrieved_chunks must contain at least 1 and at most 10 chunks
- retrieval_time_ms must be positive
- total_candidates must be greater than or equal to the number of retrieved_chunks

### PerformanceMetric Validation
- latency_ms must be positive
- relevance_score must be between 0.0 and 1.0
- result_count must be non-negative

## State Transitions

### Query Processing States
- **PENDING**: Query received, embedding generation started
- **EMBEDDING**: Query embedding in progress
- **SEARCHING**: Vector search in Qdrant in progress
- **COMPLETED**: Retrieval successful, results available
- **FAILED**: Retrieval failed due to error

### RetrievalResult States
- **PROCESSING**: Retrieval in progress
- **VALIDATED**: Results validated against criteria
- **RECORDED**: Performance metrics recorded
- **DELIVERED**: Results delivered to user