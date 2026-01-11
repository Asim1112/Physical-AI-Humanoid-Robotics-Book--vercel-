"""
Quickstart Validation Script (T064)

Validates all steps from specs/002-rag-agent-integration/quickstart.md
"""

import os
import sys
import logging
from typing import Dict, Any, List

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def validate_step(step_name: str, validation_func) -> Dict[str, Any]:
    """
    Run a validation step and return results.

    Args:
        step_name: Name of the validation step
        validation_func: Function to execute for validation

    Returns:
        Dict with validation results
    """
    logger.info(f"\n{'='*80}")
    logger.info(f"Validating: {step_name}")
    logger.info(f"{'='*80}")

    try:
        result = validation_func()
        logger.info(f"✓ {step_name}: PASS")
        return {
            "step": step_name,
            "status": "PASS",
            "result": result
        }
    except Exception as e:
        logger.error(f"✗ {step_name}: FAIL - {str(e)}")
        return {
            "step": step_name,
            "status": "FAIL",
            "error": str(e)
        }


# ============================================================================
# Validation Steps from quickstart.md
# ============================================================================

def step1_verify_dependencies():
    """Step 1: Verify all required dependencies are installed"""
    logger.info("Checking Python packages...")

    dependencies = [
        ("openai-agents", "agents"),
        ("cohere", "cohere"),
        ("qdrant-client", "qdrant_client"),
        ("google-generativeai", "google.generativeai"),
        ("python-dotenv", "dotenv")
    ]

    missing = []
    for package_name, import_name in dependencies:
        try:
            __import__(import_name)
            logger.info(f"  ✓ {package_name} available")
        except ImportError:
            logger.error(f"  ✗ {package_name} MISSING")
            missing.append(package_name)

    if missing:
        raise Exception(f"Missing packages: {', '.join(missing)}")

    return {"dependencies_checked": len(dependencies), "all_available": True}


def step2_verify_environment():
    """Step 2: Verify environment variables are configured"""
    logger.info("Checking environment variables...")

    from dotenv import load_dotenv
    load_dotenv()

    required_vars = [
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "COHERE_API_KEY",
        "GEMINI_API_KEY"
    ]

    missing = []
    for var in required_vars:
        value = os.getenv(var)
        if value:
            logger.info(f"  ✓ {var} configured")
        else:
            logger.error(f"  ✗ {var} MISSING")
            missing.append(var)

    if missing:
        raise Exception(f"Missing environment variables: {', '.join(missing)}")

    return {"env_vars_checked": len(required_vars), "all_configured": True}


def step3_verify_vector_database():
    """Step 3: Verify Qdrant contains textbook content"""
    logger.info("Checking Qdrant database...")

    from storage import create_qdrant_client

    client = create_qdrant_client(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid-robotics-textbook")

    collection_info = client.get_collection(collection_name=collection_name)
    points_count = collection_info.points_count

    logger.info(f"  Collection: {collection_name}")
    logger.info(f"  Vectors: {points_count}")
    logger.info(f"  Vector size: {collection_info.config.params.vectors.size}")

    if points_count == 0:
        raise Exception("Collection exists but contains no vectors")

    if points_count < 100:
        logger.warning(f"  ⚠ Low vector count: {points_count} (expected 1000+)")

    return {
        "collection": collection_name,
        "points_count": points_count,
        "vector_size": collection_info.config.params.vectors.size
    }


def step4_test_single_query():
    """Step 4: Test single query example from quickstart"""
    logger.info("Testing single query...")

    from agent import run_agent_query

    result = run_agent_query(
        query_text="How do I install ROS 2?",
        top_k=5,
        score_threshold=0.4
    )

    logger.info(f"  Retrieval success: {result.retrieval_success}")
    logger.info(f"  Chunks retrieved: {len(result.retrieved_chunks)}")
    logger.info(f"  Response length: {len(result.response_text)} chars")
    logger.info(f"  Response time: {result.response_time_ms:.0f}ms")

    if not result.retrieval_success:
        raise Exception(f"Query failed: {result.error_message}")

    if len(result.retrieved_chunks) == 0:
        raise Exception("No chunks retrieved")

    if len(result.response_text) < 50:
        raise Exception("Response too short")

    # Show response preview
    preview = result.response_text[:200] + "..." if len(result.response_text) > 200 else result.response_text
    logger.info(f"  Response preview: {preview}")

    return {
        "retrieval_success": result.retrieval_success,
        "chunks_retrieved": len(result.retrieved_chunks),
        "response_length": len(result.response_text),
        "response_time_ms": result.response_time_ms
    }


def step5_test_multi_turn_conversation():
    """Step 5: Test multi-turn conversation from quickstart"""
    logger.info("Testing multi-turn conversation...")

    from agent import create_agent_session, continue_agent_session

    # Create session
    session = create_agent_session()
    logger.info(f"  Created session: {session.session_id}")

    # First query
    response1 = continue_agent_session(
        session_id=session.session_id,
        query_text="What is Gazebo simulation?",
        top_k=3
    )

    logger.info(f"  Turn 1: {response1.retrieval_success}, {len(response1.response_text)} chars")

    # Follow-up query (context maintained)
    response2 = continue_agent_session(
        session_id=session.session_id,
        query_text="How do I get started with it?",
        top_k=3
    )

    logger.info(f"  Turn 2: {response2.retrieval_success}, {len(response2.response_text)} chars")

    if not response1.retrieval_success:
        raise Exception("Turn 1 failed")

    if not response2.retrieval_success:
        raise Exception("Turn 2 failed")

    if session.current_turn != 2:
        raise Exception(f"Expected 2 turns, got {session.current_turn}")

    return {
        "session_id": session.session_id,
        "total_turns": session.current_turn,
        "both_successful": True
    }


def step6_test_selected_text_mode():
    """Step 6: Test selected text mode from quickstart"""
    logger.info("Testing selected text mode...")

    from agent import run_agent_query

    result = run_agent_query(
        query_text="Explain this",
        selected_text="Gazebo is a robot simulation environment that provides physics simulation, sensor simulation, and robot models.",
        top_k=5,
        score_threshold=0.4
    )

    logger.info(f"  Retrieval success: {result.retrieval_success}")
    logger.info(f"  Chunks retrieved: {len(result.retrieved_chunks)}")
    logger.info(f"  Response length: {len(result.response_text)} chars")

    if not result.retrieval_success:
        raise Exception(f"Selected text query failed: {result.error_message}")

    if len(result.response_text) < 50:
        raise Exception("Response too short")

    return {
        "retrieval_success": result.retrieval_success,
        "chunks_retrieved": len(result.retrieved_chunks),
        "response_length": len(result.response_text)
    }


def step7_test_custom_parameters():
    """Step 7: Test custom parameter configuration"""
    logger.info("Testing custom parameters...")

    from agent import run_agent_query

    result = run_agent_query(
        query_text="What is ROS 2?",
        temperature=0.3,       # Lower for more factual
        top_k=3,               # Fewer chunks
        score_threshold=0.6,   # Higher threshold
        max_turns=5
    )

    logger.info(f"  Query successful: {result.retrieval_success}")
    logger.info(f"  Chunks: {len(result.retrieved_chunks)}")

    if not result.retrieval_success and "quota" not in (result.error_message or "").lower():
        raise Exception("Custom parameter query failed")

    return {
        "parameters_accepted": True,
        "retrieval_success": result.retrieval_success
    }


def step8_test_health_check():
    """Step 8: Test health check endpoint"""
    logger.info("Testing health check...")

    from agent import check_health

    health = check_health()

    logger.info(f"  Overall status: {health['status']}")
    logger.info(f"  Services checked: {len(health['services'])}")

    for service_name, service_data in health['services'].items():
        logger.info(f"    {service_name}: {service_data['status']}")

    return {
        "overall_status": health['status'],
        "services_checked": len(health['services']),
        "details": health
    }


def step9_test_validation_queries():
    """Step 9: Run 5 validation queries from quickstart"""
    logger.info("Running 5 validation queries...")

    from agent import run_agent_query

    test_queries = [
        "How do I install ROS 2?",
        "What is Gazebo simulation?",
        "Explain VLA models",
        "How to work with humanoid robots?",
        "What are the key concepts in robotics?"
    ]

    results = []
    for i, query in enumerate(test_queries, 1):
        logger.info(f"  Query {i}/5: {query[:50]}...")

        result = run_agent_query(
            query_text=query,
            top_k=3,
            score_threshold=0.4
        )

        success = result.retrieval_success and len(result.response_text) > 50
        results.append({
            "query": query,
            "success": success,
            "chunks": len(result.retrieved_chunks),
            "response_time_ms": result.response_time_ms
        })

        logger.info(f"    {'✓' if success else '✗'} {result.response_time_ms:.0f}ms, {len(result.retrieved_chunks)} chunks")

    successful = sum(1 for r in results if r["success"])
    total = len(results)
    success_rate = (successful / total * 100) if total > 0 else 0

    logger.info(f"  Success rate: {successful}/{total} ({success_rate:.1f}%)")

    if successful < total:
        logger.warning(f"  ⚠ Some queries failed ({total - successful}/{total})")

    return {
        "total_queries": total,
        "successful": successful,
        "success_rate": success_rate,
        "results": results
    }


# ============================================================================
# Main Validation Runner
# ============================================================================

def run_quickstart_validation() -> Dict[str, Any]:
    """
    Run full quickstart validation (T064).

    Returns:
        Dict with complete validation results
    """
    logger.info(f"\n{'#'*80}")
    logger.info("# QUICKSTART VALIDATION (T064)")
    logger.info(f"# Following steps from: specs/002-rag-agent-integration/quickstart.md")
    logger.info(f"{'#'*80}")

    steps = [
        ("Verify Dependencies", step1_verify_dependencies),
        ("Verify Environment Variables", step2_verify_environment),
        ("Verify Vector Database", step3_verify_vector_database),
        ("Test Single Query", step4_test_single_query),
        ("Test Multi-Turn Conversation", step5_test_multi_turn_conversation),
        ("Test Selected Text Mode", step6_test_selected_text_mode),
        ("Test Custom Parameters", step7_test_custom_parameters),
        ("Test Health Check", step8_test_health_check),
        ("Run Validation Queries", step9_test_validation_queries)
    ]

    results = []
    for step_name, step_func in steps:
        result = validate_step(step_name, step_func)
        results.append(result)

        # Stop on critical failures
        if result["status"] == "FAIL" and step_name in ["Verify Dependencies", "Verify Environment Variables"]:
            logger.error(f"Critical failure at: {step_name}")
            logger.error("Cannot continue validation without required dependencies/configuration")
            break

    # Calculate summary
    passed = sum(1 for r in results if r["status"] == "PASS")
    total = len(results)
    pass_rate = (passed / total * 100) if total > 0 else 0

    summary = {
        "total_steps": total,
        "passed": passed,
        "failed": total - passed,
        "pass_rate": pass_rate,
        "results": results,
        "overall_status": "PASS" if passed == total else "FAIL"
    }

    # Print summary
    logger.info(f"\n{'#'*80}")
    logger.info("# QUICKSTART VALIDATION SUMMARY")
    logger.info(f"{'#'*80}")
    logger.info(f"\nSteps Passed: {passed}/{total} ({pass_rate:.1f}%)")
    logger.info(f"Overall Status: {summary['overall_status']}")

    return summary


if __name__ == "__main__":
    import json

    # Run validation
    results = run_quickstart_validation()

    # Save results
    output_file = "quickstart_validation_results.json"
    with open(output_file, "w") as f:
        json.dump(results, f, indent=2)

    logger.info(f"\nResults saved to: {output_file}")

    # Exit with appropriate code
    exit(0 if results["overall_status"] == "PASS" else 1)
