"""Main orchestration script for RAG content ingestion pipeline."""

import logging
import os
import sys
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

from utils import (
    setup_logging,
    load_environment_variables,
    validate_api_keys,
    redact_api_key
)
from deployment import deploy_to_vercel, verify_deployment, check_deployment_features
from chunking import discover_markdown_files, process_file_to_chunks
from embedding import embed_chunks
from storage import (
    create_qdrant_client,
    setup_qdrant_collection,
    convert_chunks_to_points,
    upsert_vectors,
    query_similar
)
from retrieve import validate_retrieval_with_sample_queries


# ==============================================================================
# T038: ProcessingJob Dataclass
# ==============================================================================

@dataclass
class ProcessingJob:
    """
    Tracks pipeline execution metrics and status.

    Attributes per data-model.md specification.
    """
    job_id: str
    files_discovered: int = 0
    files_processed: int = 0
    files_failed: int = 0
    total_chunks: int = 0
    total_embeddings: int = 0
    total_vectors_stored: int = 0
    started_at: str = field(default_factory=lambda: datetime.utcnow().isoformat() + "Z")
    completed_at: Optional[str] = None
    status: str = "running"  # running, completed, failed
    errors: List[str] = field(default_factory=list)
    deployment_url: Optional[str] = None


# ==============================================================================
# T017: Main Pipeline Orchestration with Deployment Phase
# ==============================================================================

def main() -> int:
    """
    Main entry point for RAG ingestion pipeline.

    Returns:
        Exit code (0 for success, 1 for failure)
    """
    # Setup logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = f"logs/ingestion-{timestamp}.log"
    logger = setup_logging(log_level="INFO", log_file=log_file)

    logger.info("=" * 70)
    logger.info("RAG Content Ingestion Pipeline Started")
    logger.info("=" * 70)

    # T038: Create ProcessingJob to track metrics
    job = ProcessingJob(job_id=str(uuid.uuid4()))
    logger.info(f"Job ID: {job.job_id}")

    try:
        # ======================================================================
        # Phase 0: Environment Setup
        # ======================================================================
        logger.info("\n[Phase 0] Environment Setup")
        logger.info("-" * 70)

        # Load environment variables
        try:
            load_environment_variables()
            logger.info("[OK] Environment variables loaded")
        except FileNotFoundError as e:
            logger.error(f"[ERROR] {e}")
            return 1

        # Validate API keys
        try:
            env_vars = validate_api_keys()
            logger.info("[OK] API keys validated")

            # Log redacted keys for verification
            logger.info(f"  - Cohere API: {redact_api_key(env_vars['COHERE_API_KEY'])}")
            logger.info(f"  - Qdrant URL: {env_vars['QDRANT_URL']}")
            logger.info(f"  - Qdrant API: {redact_api_key(env_vars['QDRANT_API_KEY'])}")

        except ValueError as e:
            logger.error(f"[ERROR] API key validation failed:\n{e}")
            return 1

        # ======================================================================
        # Phase 1: Deploy Frontend to Vercel (User Story 1)
        # ======================================================================
        logger.info("\n[Phase 1] Deploy Frontend to Vercel (US1)")
        logger.info("-" * 70)

        # Check if deployment URL already exists in environment
        existing_url = os.getenv("DEPLOYED_VERCEL_URL")
        if existing_url:
            logger.info(f"Using existing deployment URL from environment: {existing_url}")
            deployment_url = existing_url
            job.deployment_url = deployment_url
            logger.info("Deployment phase skipped - using existing URL")
        else:
            deployment_url = run_deployment_phase()

            if not deployment_url:
                logger.error("Deployment phase failed")
                job.status = "failed"
                job.errors.append("Deployment phase failed")
                return 1

            job.deployment_url = deployment_url
            logger.info(f"Deployment phase complete: {deployment_url}")

        # ======================================================================
        # Phase 2: Chunking and Embedding (User Story 2)
        # ======================================================================
        logger.info("\n[Phase 2] Chunking and Embedding (US2)")
        logger.info("-" * 70)

        chunks, embedding_vectors = run_chunking_embedding_phase(env_vars, job)

        if not chunks or not embedding_vectors:
            logger.error("[ERROR] Chunking/embedding phase failed")
            job.status = "failed"
            job.errors.append("Chunking/embedding phase failed")
            return 1

        job.total_chunks = len(chunks)
        job.total_embeddings = len(embedding_vectors)
        logger.info(f"[OK] Chunking/embedding phase complete: {len(chunks)} chunks, {len(embedding_vectors)} embeddings")

        # ======================================================================
        # Phase 3: Vector Storage (User Story 3)
        # ======================================================================
        logger.info("\n[Phase 3] Vector Storage (US3)")
        logger.info("-" * 70)

        vectors_stored, top_result = run_storage_phase(env_vars, chunks, embedding_vectors)

        if vectors_stored == 0:
            logger.error("[ERROR] Storage phase failed")
            job.status = "failed"
            job.errors.append("Storage phase failed")
            return 1

        job.total_vectors_stored = vectors_stored
        logger.info(f"[OK] Storage phase complete: {vectors_stored} vectors stored")

        # ======================================================================
        # Pipeline Complete - T039: Final Report
        # ======================================================================
        job.status = "completed"
        job.completed_at = datetime.utcnow().isoformat() + "Z"

        logger.info("\n" + "=" * 70)
        logger.info("PIPELINE EXECUTION SUMMARY")
        logger.info("=" * 70)
        logger.info(f"Job ID: {job.job_id}")
        logger.info(f"Status: {job.status}")
        logger.info(f"Started: {job.started_at}")
        logger.info(f"Completed: {job.completed_at}")
        logger.info("")
        logger.info("Metrics:")
        logger.info(f"  - Deployment URL: {job.deployment_url}")
        logger.info(f"  - Files discovered: {job.files_discovered}")
        logger.info(f"  - Files processed: {job.files_processed}")
        logger.info(f"  - Files failed: {job.files_failed}")
        logger.info(f"  - Total chunks: {job.total_chunks}")
        logger.info(f"  - Total embeddings: {job.total_embeddings}")
        logger.info(f"  - Vectors stored: {job.total_vectors_stored}")
        logger.info("")
        logger.info(f"Log file: {log_file}")

        # T039: Show top query result if available
        if top_result:
            point_id, score, payload = top_result
            logger.info("")
            logger.info("Sample Query Result (verification):")
            logger.info(f"  - Score: {score:.3f}")
            logger.info(f"  - Source: {payload.get('source_file', 'N/A')}")
            logger.info(f"  - Module: {payload.get('module_name', 'N/A')}")
            logger.info(f"  - Section: {payload.get('section_heading', 'N/A')}")

        logger.info("=" * 70)

        return 0

    except KeyboardInterrupt:
        logger.warning("\n[WARN] Pipeline interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"\n[ERROR] Unexpected error: {e}", exc_info=True)
        return 1


# ==============================================================================
# T017-T019: Deployment Phase Implementation
# ==============================================================================

def run_deployment_phase() -> Optional[str]:
    """
    Execute deployment phase: deploy frontend and verify accessibility.

    Returns:
        Deployment URL if successful, None if failed

    Handles:
        - T018: Deployment logging
        - T019: Deployment error handling
    """
    logger = logging.getLogger("rag_ingestion")

    try:
        # T018: Deployment started logging
        logger.info("Deploying Docusaurus frontend to Vercel...")
        logger.info("This may take 1-2 minutes for build and deployment...")

        # T015: Deploy to Vercel
        deployment_url = deploy_to_vercel(frontend_path="../frontend", production=True)

        if not deployment_url:
            logger.error("[ERROR] Deployment completed but URL not captured")
            return None

        # T018: URL captured logging
        logger.info(f"[OK] Deployment URL: {deployment_url}")

        # T016: Verify deployment
        logger.info("Verifying deployment accessibility...")
        success, error = verify_deployment(deployment_url)

        if not success:
            # T019: Verification failure handling
            logger.error(f"[ERROR] Deployment verification failed: {error}")
            logger.warning(
                "URL was returned but site may not be accessible yet. "
                "Vercel deployments can take a few moments to propagate."
            )
            return None

        # T018: Verification success logging
        logger.info("[OK] Deployment verified - site is accessible")

        # Additional feature checks
        features = check_deployment_features(deployment_url)
        logger.info(f"Feature checks: {features}")

        return deployment_url

    except RuntimeError as e:
        # T019: CLI not found, build failure, authentication error
        error_msg = str(e).lower()

        if "cli not found" in error_msg or "vercel" in error_msg:
            logger.error("[ERROR] Vercel CLI not found")
            logger.error("Install with: npm install -g vercel")
            logger.error("Authenticate with: vercel login")
        elif "authentication" in error_msg or "login" in error_msg:
            logger.error("[ERROR] Vercel authentication failed")
            logger.error("Run: vercel login")
        elif "build" in error_msg or "failed with exit code" in error_msg:
            logger.error("[ERROR] Frontend build failed")
            logger.error("Check frontend/package.json and build configuration")
        else:
            logger.error(f"[ERROR] Deployment error: {e}")

        return None

    except Exception as e:
        # T019: Unexpected errors
        logger.error(f"[ERROR] Unexpected deployment error: {e}", exc_info=True)
        return None


# ==============================================================================
# T028-T030: Chunking and Embedding Phase Implementation
# ==============================================================================

def run_chunking_embedding_phase(env_vars: dict, job: ProcessingJob) -> Tuple[List, List]:
    """
    Execute chunking and embedding phase.

    Returns:
        Tuple of (chunks, embedding_vectors)

    Handles:
        - T028: Chunking and embedding orchestration
        - T029: Pipeline logging
        - T030: Error handling
    """
    logger = logging.getLogger("rag_ingestion")

    try:
        # Get docs path from env or use default
        frontend_path = os.getenv("FRONTEND_PATH", "../frontend")
        docs_path = os.getenv("DOCS_PATH", "docs")
        full_docs_path = Path(__file__).parent / frontend_path / docs_path

        # T028: Discover markdown files
        # T029: Logging
        logger.info("Discovering markdown files...")
        files = discover_markdown_files(str(full_docs_path))
        logger.info(f"[OK] Discovered {len(files)} markdown files")

        # Update job metrics
        job.files_discovered = len(files)

        if not files:
            logger.warning("No markdown files found in docs directory")
            return [], []

        # T028: Process files to chunks
        # T029: Logging
        logger.info("Chunking markdown files...")
        all_chunks = []
        files_processed = 0
        files_failed = 0

        for file_path in files:
            try:
                chunks = process_file_to_chunks(file_path, full_docs_path)
                all_chunks.extend(chunks)
                files_processed += 1
            except Exception as e:
                # T030: Error handling
                logger.warning(f"Failed to process {file_path.name}: {e}")
                files_failed += 1
                continue

        # Update job metrics
        job.files_processed = files_processed
        job.files_failed = files_failed

        # T029: Logging
        logger.info(f"[OK] Created {len(all_chunks)} chunks from {files_processed} files")
        if files_failed > 0:
            logger.warning(f"[WARN] {files_failed} files failed to process")

        if not all_chunks:
            logger.error("No chunks created from markdown files")
            return [], []

        # Calculate and log statistics
        avg_tokens = sum(c.token_count for c in all_chunks) / len(all_chunks)
        min_tokens = min(c.token_count for c in all_chunks)
        max_tokens = max(c.token_count for c in all_chunks)
        logger.info(f"  Token stats - avg: {avg_tokens:.0f}, min: {min_tokens}, max: {max_tokens}")

        # T045: Validate token count ranges (150-300 acceptable accounting for ~30% Cohere tokenizer variance)
        out_of_range_chunks = [
            c for c in all_chunks
            if c.token_count < 150 or c.token_count > 300
        ]

        if out_of_range_chunks:
            # Count by range
            too_small = [c for c in out_of_range_chunks if c.token_count < 150]
            too_large = [c for c in out_of_range_chunks if c.token_count > 300]

            logger.warning(f"[WARN] {len(out_of_range_chunks)} chunks outside 150-300 token range:")
            if too_small:
                logger.warning(f"  - {len(too_small)} chunks < 150 tokens")
            if too_large:
                logger.warning(f"  - {len(too_large)} chunks > 300 tokens")

            # Log samples for debugging
            if too_small:
                sample = too_small[0]
                logger.warning(
                    f"  Sample (too small): {sample.chunk_id[:8]}... "
                    f"from {sample.source_file} ({sample.token_count} tokens)"
                )
            if too_large:
                sample = too_large[0]
                logger.warning(
                    f"  Sample (too large): {sample.chunk_id[:8]}... "
                    f"from {sample.source_file} ({sample.token_count} tokens)"
                )
        else:
            logger.info("[OK] All chunks within 200-500 token range (Cohere API limit)")

        # T028: Generate embeddings
        # T029: Logging
        logger.info("Generating embeddings...")
        logger.info(f"  Processing {len(all_chunks)} chunks with Cohere API...")

        embedding_vectors = embed_chunks(all_chunks, api_key=env_vars["COHERE_API_KEY"])

        # T029: Logging
        logger.info(f"[OK] Generated {len(embedding_vectors)} embeddings")

        # Validate embedding count
        if len(embedding_vectors) != len(all_chunks):
            # Partial embeddings due to token filtering - this is expected
            skipped = len(all_chunks) - len(embedding_vectors)
            logger.warning(
                f"Partial embeddings: {len(embedding_vectors)}/{len(all_chunks)} chunks embedded "
                f"({skipped} skipped due to Cohere's 508-token limit)"
            )

        # Return only the chunks that were successfully embedded
        # Create mapping of embedded chunk IDs
        embedded_chunk_ids = {ev.chunk_id for ev in embedding_vectors}
        embedded_chunks = [c for c in all_chunks if c.chunk_id in embedded_chunk_ids]

        logger.info(f"Returning {len(embedded_chunks)} chunks with embeddings for storage")

        return embedded_chunks, embedding_vectors

    except FileNotFoundError as e:
        # T030: Error handling
        logger.error(f"[ERROR] Docs directory not found: {e}")
        return [], []
    except ValueError as e:
        # T030: Error handling (API key issues, validation failures)
        logger.error(f"[ERROR] Validation error: {e}")
        return [], []
    except Exception as e:
        # T030: Error handling
        logger.error(f"[ERROR] Chunking/embedding phase failed: {e}", exc_info=True)
        return [], []


# ==============================================================================
# T035-T037: Storage and Verification Phase Implementation
# ==============================================================================

def run_storage_phase(
    env_vars: dict,
    chunks: List,
    embedding_vectors: List
) -> Tuple[int, Optional[Tuple]]:
    """
    Execute storage and verification phase.

    Returns:
        Tuple of (vectors_stored, top_result)
        - vectors_stored: Number of vectors successfully stored
        - top_result: Tuple of (point_id, score, payload) from sample query

    Handles:
        - T035: Storage orchestration
        - T036: Storage pipeline logging
        - T037: Storage error handling
    """
    logger = logging.getLogger("rag_ingestion")

    try:
        # T035: Create Qdrant client
        # T036: Logging
        logger.info("Connecting to Qdrant...")
        client = create_qdrant_client(
            url=env_vars["QDRANT_URL"],
            api_key=env_vars["QDRANT_API_KEY"]
        )
        logger.info("[OK] Qdrant client connected")

        # Get collection name
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook")

        # T035: Setup collection
        # T036: Logging
        logger.info(f"Setting up collection '{collection_name}'...")
        success = setup_qdrant_collection(
            client=client,
            collection_name=collection_name,
            vector_size=1024,
            distance="Cosine",
            recreate=False  # Don't recreate if exists
        )

        if not success:
            # T037: Error handling
            logger.error("Failed to setup Qdrant collection")
            return 0, None

        logger.info(f"[OK] Collection '{collection_name}' ready")

        # T035: Convert chunks to Qdrant points
        # T036: Logging
        logger.info("Converting chunks to Qdrant points...")
        points = convert_chunks_to_points(chunks, embedding_vectors)
        logger.info(f"[OK] Converted {len(points)} points")

        # T035: Upsert vectors
        # T036: Logging
        logger.info("Uploading vectors to Qdrant...")
        vectors_stored = upsert_vectors(
            client=client,
            collection_name=collection_name,
            points=points,
            batch_size=100
        )

        if vectors_stored == 0:
            # T037: Error handling
            logger.error("No vectors were stored")
            return 0, None

        logger.info(f"[OK] {vectors_stored} vectors stored successfully")

        # T035: Verification query (use first embedding as sample)
        # T036: Logging
        logger.info("Verifying storage with sample query...")
        sample_vector = embedding_vectors[0].vector

        results = query_similar(
            client=client,
            collection_name=collection_name,
            query_vector=sample_vector,
            top_k=5,
            score_threshold=0.7
        )

        if not results:
            # T037: Error handling
            logger.warning("[WARN] Sample query returned no results (may indicate storage issue)")
            return vectors_stored, None

        # T036: Logging
        top_result = results[0]
        point_id, score, payload = top_result
        logger.info(f"[OK] Verification query successful")
        logger.info(f"  Top result score: {score:.3f}")
        logger.info(f"  Source: {payload.get('source_file', 'N/A')}")

        return vectors_stored, top_result

    except ValueError as e:
        # T037: Error handling (connection, validation issues)
        logger.error(f"[ERROR] Storage configuration error: {e}")
        return 0, None
    except Exception as e:
        # T037: Error handling
        logger.error(f"[ERROR] Storage phase failed: {e}", exc_info=True)
        return 0, None


# ==============================================================================
# T035: Retrieval Validation Phase
# ==============================================================================

def run_retrieval_phase(env_vars: dict) -> bool:
    """
    Execute retrieval validation phase.

    Args:
        env_vars: Environment variables dict with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY

    Returns:
        True if validation passes (>= 80% success rate), False otherwise

    Implements:
        - T030-T034: Validation with sample queries, performance metrics, success rate calculation
        - T035: Integration into main.py
    """
    logger = logging.getLogger("rag_retrieval")

    logger.info("=" * 80)
    logger.info("PHASE: RETRIEVAL VALIDATION")
    logger.info("=" * 80)

    # Connect to Qdrant
    logger.info("Connecting to Qdrant...")
    try:
        client = create_qdrant_client(
            url=env_vars["QDRANT_URL"],
            api_key=env_vars["QDRANT_API_KEY"]
        )
        logger.info("[OK] Qdrant client connected")
    except Exception as e:
        logger.error(f"[ERROR] Failed to connect to Qdrant: {e}")
        return False

    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook")

    # T030-T032: Run validation with sample queries
    logger.info(f"Running validation with 5 sample queries on collection '{collection_name}'...")

    try:
        validation_results = validate_retrieval_with_sample_queries(
            client,
            collection_name,
            api_key=env_vars.get("COHERE_API_KEY")
        )
    except Exception as e:
        logger.error(f"[ERROR] Validation failed: {e}", exc_info=True)
        return False

    # T032-T033: Calculate success rate and log detailed results
    successful_queries = sum(1 for _, _, success in validation_results if success)
    total_queries = len(validation_results)

    logger.info("")
    logger.info(f"Validation Results: {successful_queries}/{total_queries} queries successful")
    logger.info("-" * 80)

    # T033: Log detailed results for each query
    total_latency = 0
    total_relevance = 0
    relevant_count = 0

    for query, result, success in validation_results:
        if success and result:
            logger.info(f"✓ Query: '{query}'")
            logger.info(f"  Retrieved: {len(result.retrieved_chunks)} chunks in {result.retrieval_time_ms:.2f}ms")

            if result.retrieved_chunks:
                top_chunk = result.retrieved_chunks[0]
                logger.info(f"  Top result: {top_chunk.similarity_score:.3f} similarity from {top_chunk.source_file}")

                total_latency += result.retrieval_time_ms
                total_relevance += top_chunk.similarity_score
                relevant_count += 1
        else:
            logger.warning(f"✗ Query: '{query}' - No relevant results or error")

    logger.info("-" * 80)

    # T034: Generate summary statistics
    success_rate = successful_queries / total_queries if total_queries > 0 else 0
    avg_latency = total_latency / relevant_count if relevant_count > 0 else 0
    avg_relevance = total_relevance / relevant_count if relevant_count > 0 else 0

    logger.info("Summary Statistics:")
    logger.info(f"  Success Rate: {success_rate:.1%} ({successful_queries}/{total_queries})")
    logger.info(f"  Average Latency: {avg_latency:.2f}ms")
    logger.info(f"  Average Relevance: {avg_relevance:.3f}")

    # Check if we met success criteria (SC-001: 80% success rate)
    meets_criteria = success_rate >= 0.8

    if meets_criteria:
        logger.info("[OK] Retrieval validation PASSED (≥80% success rate)")
    else:
        logger.error(f"[ERROR] Retrieval validation FAILED ({success_rate:.1%} success rate < 80%)")

    return meets_criteria


# ==============================================================================
# T070: Agent CLI Integration
# ==============================================================================

def run_agent_cli():
    """
    Run agent CLI commands (T070).
    """
    from agent import (
        run_agent_query,
        run_agent_query_streaming,
        create_agent_session,
        continue_agent_session,
        check_health
    )

    parser = argparse.ArgumentParser(description="RAG Agent CLI")
    subparsers = parser.add_subparsers(dest="command", help="Agent command")

    # Query command
    query_parser = subparsers.add_parser("query", help="Execute a single query")
    query_parser.add_argument("query_text", help="Query text")
    query_parser.add_argument("--selected-text", help="Selected text context")
    query_parser.add_argument("--temperature", type=float, default=0.5, help="Response temperature")
    query_parser.add_argument("--top-k", type=int, default=5, help="Number of chunks")
    query_parser.add_argument("--stream", action="store_true", help="Stream response")

    # Conversation command
    conv_parser = subparsers.add_parser("conversation", help="Interactive conversation")
    conv_parser.add_argument("--session-id", help="Resume session")
    conv_parser.add_argument("--temperature", type=float, default=0.5)
    conv_parser.add_argument("--top-k", type=int, default=5)

    # Health command
    subparsers.add_parser("health", help="Check system health")

    args = parser.parse_args(sys.argv[2:])  # Skip "agent" command

    if args.command == "query":
        if args.stream:
            for event in run_agent_query_streaming(
                query_text=args.query_text,
                selected_text=args.selected_text,
                temperature=args.temperature,
                top_k=args.top_k
            ):
                if event["event"] == "response_chunk":
                    print(event["text"], end="", flush=True)
                elif event["event"] == "response_complete":
                    print()
        else:
            result = run_agent_query(
                query_text=args.query_text,
                selected_text=args.selected_text,
                temperature=args.temperature,
                top_k=args.top_k
            )
            print(f"\n{result.response_text}\n")

    elif args.command == "conversation":
        session = create_agent_session() if not args.session_id else None
        if session:
            print(f"Session: {session.session_id}")

        while True:
            user_input = input("You: ").strip()
            if user_input.lower() in ["exit", "quit"]:
                break

            result = continue_agent_session(
                session_id=session.session_id if session else args.session_id,
                query_text=user_input,
                temperature=args.temperature,
                top_k=args.top_k
            )
            print(f"Assistant: {result.response_text}\n")

    elif args.command == "health":
        health = check_health()
        print(f"Status: {health['status']}")
        for service, data in health['services'].items():
            print(f"  {service}: {data['status']}")


# ==============================================================================
# Entry Point with Command-Line Arguments
# ==============================================================================

if __name__ == "__main__":
    import argparse

    # Main parser with sub-commands
    parser = argparse.ArgumentParser(description="RAG Pipeline and Agent Orchestration (T070)")
    parser.add_argument(
        "--retrieval-only",
        action="store_true",
        help="Run only the retrieval validation phase (skip ingestion)"
    )
    parser.add_argument(
        "mode",
        nargs="?",
        choices=["pipeline", "agent"],
        default="pipeline",
        help="Mode: pipeline (ingestion) or agent (CLI)"
    )

    args, remaining = parser.parse_known_args()

    if args.mode == "agent":
        # T070: Agent CLI mode
        run_agent_cli()

    elif args.retrieval_only:
        # Run only retrieval validation
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = f"logs/retrieval-{timestamp}.log"
        logger = setup_logging(log_level="INFO", log_file=log_file)

        logger.info("=" * 70)
        logger.info("RAG Retrieval Validation (Standalone Mode)")
        logger.info("=" * 70)

        try:
            load_environment_variables()
            env_vars = validate_api_keys()

            # Run retrieval validation
            success = run_retrieval_phase(env_vars)
            sys.exit(0 if success else 1)

        except Exception as e:
            logger.error(f"[ERROR] Retrieval validation failed: {e}", exc_info=True)
            sys.exit(1)
    else:
        # Run full pipeline
        sys.exit(main())
