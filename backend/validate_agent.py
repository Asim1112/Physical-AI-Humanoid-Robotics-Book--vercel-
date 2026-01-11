"""
Validation Script for RAG Agent (T069)

Runs 5+ sample queries covering all textbook modules to validate:
- Retrieval accuracy
- Response quality
- Performance metrics
- Error handling
"""

import logging
import time
from typing import List, Dict, Any
from agent import run_agent_query, create_agent_session, continue_agent_session, check_health

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# Test Queries
# ============================================================================

TEST_QUERIES = [
    {
        "category": "ROS 2 Basics",
        "query": "How do I install ROS 2?",
        "expected_keywords": ["install", "ROS 2", "ubuntu", "setup"],
        "min_chunks": 2
    },
    {
        "category": "Gazebo Simulation",
        "query": "What is Gazebo used for in robotics?",
        "expected_keywords": ["simulation", "gazebo", "physics", "environment"],
        "min_chunks": 2
    },
    {
        "category": "VLA Models",
        "query": "What are VLA models and how are they used in robotics?",
        "expected_keywords": ["vla", "vision", "language", "action"],
        "min_chunks": 2
    },
    {
        "category": "Humanoid Robots",
        "query": "What are the key challenges in humanoid robot control?",
        "expected_keywords": ["humanoid", "control", "balance", "locomotion"],
        "min_chunks": 2
    },
    {
        "category": "ROS 2 Nodes",
        "query": "Explain ROS 2 nodes and their role in robotics applications",
        "expected_keywords": ["node", "ros", "communication", "modular"],
        "min_chunks": 2
    },
    {
        "category": "Simulation and Testing",
        "query": "Why is simulation important for testing humanoid robots?",
        "expected_keywords": ["simulation", "testing", "safe", "physics"],
        "min_chunks": 2
    }
]


# ============================================================================
# Validation Functions
# ============================================================================

def validate_single_query(test_case: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate a single query against expected results.

    Args:
        test_case: Dict with query, expected_keywords, min_chunks

    Returns:
        Dict with validation results
    """
    logger.info(f"\n{'='*80}")
    logger.info(f"Testing: {test_case['category']}")
    logger.info(f"Query: {test_case['query']}")
    logger.info(f"{'='*80}")

    start_time = time.time()

    # Run query
    result = run_agent_query(
        query_text=test_case['query'],
        top_k=5,
        score_threshold=0.4,
        temperature=0.5
    )

    elapsed_time = time.time() - start_time

    # Validate results
    validation = {
        "category": test_case['category'],
        "query": test_case['query'],
        "success": result.retrieval_success,
        "response_length": len(result.response_text),
        "chunks_retrieved": len(result.retrieved_chunks),
        "response_time_ms": result.response_time_ms,
        "total_time_ms": elapsed_time * 1000,
        "has_error": result.error_message is not None,
        "error_message": result.error_message,
        "checks": {}
    }

    # Check 1: Retrieval success
    validation["checks"]["retrieval_success"] = result.retrieval_success

    # Check 2: Minimum chunks retrieved
    min_chunks = test_case.get("min_chunks", 1)
    validation["checks"]["min_chunks_retrieved"] = len(result.retrieved_chunks) >= min_chunks

    # Check 3: Response has content
    validation["checks"]["has_response_text"] = len(result.response_text) > 50

    # Check 4: Expected keywords in response (case-insensitive)
    response_lower = result.response_text.lower()
    keywords_found = []
    keywords_missing = []

    for keyword in test_case.get("expected_keywords", []):
        if keyword.lower() in response_lower:
            keywords_found.append(keyword)
        else:
            keywords_missing.append(keyword)

    validation["checks"]["keywords_found"] = len(keywords_found)
    validation["checks"]["keywords_total"] = len(test_case.get("expected_keywords", []))
    validation["checks"]["keywords_found_list"] = keywords_found
    validation["checks"]["keywords_missing_list"] = keywords_missing

    # Check 5: Response time acceptable (<15 seconds)
    validation["checks"]["response_time_acceptable"] = result.response_time_ms < 15000

    # Overall pass/fail
    validation["passed"] = all([
        validation["checks"]["retrieval_success"],
        validation["checks"]["min_chunks_retrieved"],
        validation["checks"]["has_response_text"],
        validation["checks"]["response_time_acceptable"]
    ])

    # Log results
    logger.info(f"\nResults:")
    logger.info(f"  Retrieval Success: {validation['checks']['retrieval_success']}")
    logger.info(f"  Chunks Retrieved: {validation['chunks_retrieved']} (min: {min_chunks})")
    logger.info(f"  Response Length: {validation['response_length']} chars")
    logger.info(f"  Response Time: {validation['response_time_ms']:.0f}ms")
    logger.info(f"  Keywords Found: {validation['checks']['keywords_found']}/{validation['checks']['keywords_total']}")

    if keywords_found:
        logger.info(f"    Found: {', '.join(keywords_found)}")
    if keywords_missing:
        logger.info(f"    Missing: {', '.join(keywords_missing)}")

    logger.info(f"  Overall: {'PASS' if validation['passed'] else 'FAIL'}")

    if validation["has_error"]:
        logger.error(f"  Error: {validation['error_message']}")

    # Show response preview
    if result.response_text:
        logger.info(f"\nResponse Preview:")
        preview = result.response_text[:200] + "..." if len(result.response_text) > 200 else result.response_text
        logger.info(f"  {preview}")

    # Show top chunks
    if result.retrieved_chunks:
        logger.info(f"\nTop Retrieved Chunks:")
        for i, chunk in enumerate(result.retrieved_chunks[:3], 1):
            logger.info(f"  {i}. {chunk.source_file} (score: {chunk.similarity_score:.3f})")
            logger.info(f"     {chunk.content_text[:100]}...")

    return validation


def validate_multi_turn_conversation() -> Dict[str, Any]:
    """
    Validate multi-turn conversation functionality.

    Returns:
        Dict with validation results
    """
    logger.info(f"\n{'='*80}")
    logger.info("Testing Multi-Turn Conversation")
    logger.info(f"{'='*80}")

    # Create session
    session = create_agent_session()
    logger.info(f"Created session: {session.session_id}")

    conversation_queries = [
        "What are VLA models?",
        "How are they trained?",
        "What are some applications in humanoid robotics?"
    ]

    results = []

    for i, query in enumerate(conversation_queries, 1):
        logger.info(f"\nTurn {i}: {query}")

        result = continue_agent_session(
            session_id=session.session_id,
            query_text=query,
            top_k=3,
            score_threshold=0.4
        )

        turn_result = {
            "turn": i,
            "query": query,
            "success": result.retrieval_success,
            "response_length": len(result.response_text),
            "has_error": result.error_message is not None
        }

        results.append(turn_result)

        logger.info(f"  Success: {turn_result['success']}")
        logger.info(f"  Response Length: {turn_result['response_length']} chars")

        if result.response_text:
            preview = result.response_text[:150] + "..." if len(result.response_text) > 150 else result.response_text
            logger.info(f"  Response: {preview}")

    # Validate conversation continuity
    validation = {
        "category": "Multi-Turn Conversation",
        "session_id": session.session_id,
        "total_turns": len(results),
        "successful_turns": sum(1 for r in results if r["success"]),
        "results": results,
        "passed": all(r["success"] and not r["has_error"] for r in results)
    }

    logger.info(f"\nConversation Summary:")
    logger.info(f"  Total Turns: {validation['total_turns']}")
    logger.info(f"  Successful: {validation['successful_turns']}")
    logger.info(f"  Overall: {'PASS' if validation['passed'] else 'FAIL'}")

    return validation


def run_health_check() -> Dict[str, Any]:
    """
    Run health check on all services.

    Returns:
        Dict with health check results
    """
    logger.info(f"\n{'='*80}")
    logger.info("Running Health Check")
    logger.info(f"{'='*80}")

    health = check_health()

    logger.info(f"\nOverall Status: {health['status'].upper()}")
    logger.info(f"Total Latency: {health['overall_latency_ms']:.0f}ms")

    logger.info(f"\nService Health:")
    for service_name, service_data in health['services'].items():
        status = service_data['status']
        latency = service_data.get('latency_ms', 0)
        error = service_data.get('error')

        logger.info(f"  {service_name.capitalize()}: {status.upper()} ({latency:.0f}ms)")

        if error:
            logger.error(f"    Error: {error}")

        if 'details' in service_data and service_data['details']:
            details = service_data['details']
            if 'points_count' in details:
                logger.info(f"    Vectors: {details['points_count']}")

    return health


# ============================================================================
# Main Validation Flow
# ============================================================================

def run_validation() -> Dict[str, Any]:
    """
    Run full validation suite.

    Returns:
        Dict with complete validation results
    """
    logger.info(f"\n{'#'*80}")
    logger.info("# RAG Agent Validation Suite (T069)")
    logger.info(f"{'#'*80}")

    start_time = time.time()

    # Step 1: Health Check
    health_result = run_health_check()

    # Step 2: Single Query Tests
    query_results = []
    for test_case in TEST_QUERIES:
        try:
            result = validate_single_query(test_case)
            query_results.append(result)

            # Small delay between queries to avoid rate limiting
            time.sleep(1)
        except Exception as e:
            logger.error(f"Query failed with exception: {e}")
            query_results.append({
                "category": test_case['category'],
                "query": test_case['query'],
                "passed": False,
                "error_message": str(e)
            })

    # Step 3: Multi-Turn Conversation Test
    try:
        conversation_result = validate_multi_turn_conversation()
    except Exception as e:
        logger.error(f"Multi-turn conversation failed with exception: {e}")
        conversation_result = {
            "category": "Multi-Turn Conversation",
            "passed": False,
            "error_message": str(e)
        }

    # Calculate summary
    total_time = time.time() - start_time

    passed_queries = sum(1 for r in query_results if r.get("passed", False))
    total_queries = len(query_results)

    summary = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "total_time_seconds": total_time,
        "health_check": health_result,
        "query_tests": {
            "total": total_queries,
            "passed": passed_queries,
            "failed": total_queries - passed_queries,
            "pass_rate": (passed_queries / total_queries * 100) if total_queries > 0 else 0,
            "results": query_results
        },
        "conversation_test": conversation_result,
        "overall_status": "PASS" if (
            passed_queries == total_queries and
            conversation_result.get("passed", False) and
            health_result["status"] in ["healthy", "degraded"]
        ) else "FAIL"
    }

    # Print Summary
    logger.info(f"\n{'#'*80}")
    logger.info("# VALIDATION SUMMARY")
    logger.info(f"{'#'*80}")
    logger.info(f"\nHealth Check: {health_result['status'].upper()}")
    logger.info(f"Query Tests: {passed_queries}/{total_queries} passed ({summary['query_tests']['pass_rate']:.1f}%)")
    logger.info(f"Conversation Test: {'PASS' if conversation_result.get('passed', False) else 'FAIL'}")
    logger.info(f"\nOverall Status: {summary['overall_status']}")
    logger.info(f"Total Time: {total_time:.2f}s")

    return summary


if __name__ == "__main__":
    import json

    # Run validation
    results = run_validation()

    # Save results to file
    output_file = "validation_results.json"
    with open(output_file, "w") as f:
        json.dump(results, f, indent=2)

    logger.info(f"\nResults saved to: {output_file}")

    # Exit with appropriate code
    exit(0 if results["overall_status"] == "PASS" else 1)
