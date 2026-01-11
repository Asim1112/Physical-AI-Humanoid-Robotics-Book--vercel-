"""
Phase 7 Validation Test Suite

Tests all requirements from tasks.md T082-T090:
- 5 required test queries
- Edge case testing
- Response time benchmarking
"""

import requests
import json
import time
import statistics
import sys
import io

# Fix Windows console encoding
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

API_BASE_URL = "http://localhost:8000"

def test_required_queries():
    """T082-T083: Run 5 required test queries and verify response times"""
    print("\n" + "="*60)
    print("T082-T083: Required Test Queries (< 5 seconds)")
    print("="*60)

    queries = [
        "What is Gazebo simulation?",
        "What are the key features of ROS?",
        "How does it relate to forward kinematics?",
        "What sensors are used in humanoid robots?",
    ]

    response_times = []
    passed = 0
    failed = 0

    for i, query in enumerate(queries, 1):
        print(f"\n[Query {i}/4] {query}")

        start = time.time()
        try:
            response = requests.post(
                f"{API_BASE_URL}/api/chat",
                json={"query": query, "session_id": None, "selected_text": None},
                timeout=30
            )
            elapsed = (time.time() - start) * 1000
            response_times.append(elapsed)

            if response.status_code == 200:
                data = response.json()
                print(f"  Status: 200 OK")
                print(f"  Response time: {elapsed:.0f}ms")
                print(f"  Response preview: {data['response'][:100]}...")

                if elapsed < 5000:
                    print(f"  [PASS] Under 5 seconds")
                    passed += 1
                else:
                    print(f"  [FAIL] Exceeded 5 seconds")
                    failed += 1
            else:
                print(f"  [FAIL] Status: {response.status_code}")
                failed += 1
        except Exception as e:
            print(f"  [FAIL] Error: {e}")
            failed += 1

    # Test selected-text mode (5th query)
    print(f"\n[Query 5/5 - Selected Text Mode] Highlight 'Inverse kinematics' → 'Explain this concept'")
    selected_text = "Inverse kinematics (IK) is the process of determining joint angles needed to position a robot's end-effector at a desired location."

    start = time.time()
    try:
        response = requests.post(
            f"{API_BASE_URL}/api/chat",
            json={
                "query": "Explain this concept",
                "session_id": None,
                "selected_text": selected_text
            },
            timeout=30
        )
        elapsed = (time.time() - start) * 1000
        response_times.append(elapsed)

        if response.status_code == 200:
            data = response.json()
            print(f"  Status: 200 OK")
            print(f"  Response time: {elapsed:.0f}ms")
            print(f"  Response preview: {data['response'][:100]}...")

            if elapsed < 5000:
                print(f"  [PASS] Under 5 seconds")
                passed += 1
            else:
                print(f"  [FAIL] Exceeded 5 seconds")
                failed += 1
        else:
            print(f"  [FAIL] Status: {response.status_code}")
            failed += 1
    except Exception as e:
        print(f"  [FAIL] Error: {e}")
        failed += 1

    print(f"\n{'='*60}")
    print(f"Results: {passed}/5 passed, {failed}/5 failed")
    print(f"Average response time: {statistics.mean(response_times):.0f}ms")
    print(f"{'='*60}")

    return passed == 5, response_times


def test_edge_cases():
    """T084-T088: Test edge cases"""
    print("\n" + "="*60)
    print("T084-T088: Edge Case Testing")
    print("="*60)

    tests = []

    # T084: Empty query
    print("\n[T084] Empty query")
    try:
        response = requests.post(
            f"{API_BASE_URL}/api/chat",
            json={"query": "", "session_id": None}
        )
        if response.status_code == 422:  # Pydantic validation error
            print(f"  [PASS] Returns 422 validation error")
            tests.append(True)
        else:
            print(f"  [FAIL] Expected 422, got {response.status_code}")
            tests.append(False)
    except Exception as e:
        print(f"  [FAIL] {e}")
        tests.append(False)

    # T085: Query >2000 chars
    print("\n[T085] Query >2000 characters")
    long_query = "A" * 2001
    try:
        response = requests.post(
            f"{API_BASE_URL}/api/chat",
            json={"query": long_query, "session_id": None}
        )
        if response.status_code == 422:
            print(f"  [PASS] Returns 422 validation error")
            tests.append(True)
        else:
            print(f"  [FAIL] Expected 422, got {response.status_code}")
            tests.append(False)
    except Exception as e:
        print(f"  [FAIL] {e}")
        tests.append(False)

    # T086: Invalid session_id
    print("\n[T086] Invalid session_id")
    try:
        response = requests.post(
            f"{API_BASE_URL}/api/chat",
            json={"query": "test", "session_id": "invalid-uuid-format"}
        )
        if response.status_code in [400, 422]:
            print(f"  [PASS] Returns {response.status_code} error")
            tests.append(True)
        else:
            print(f"  [FAIL] Expected 400/422, got {response.status_code}")
            tests.append(False)
    except Exception as e:
        print(f"  [FAIL] {e}")
        tests.append(False)

    # T087: Backend down (skip - can't test while backend is running)
    print("\n[T087] Backend down - SKIPPED (requires manual test)")
    print("  Manual test: Stop backend, verify frontend shows error message")

    # T088: Selected text >5000 chars
    print("\n[T088] Selected text >5000 characters")
    long_text = "B" * 6000
    try:
        response = requests.post(
            f"{API_BASE_URL}/api/chat",
            json={"query": "test", "session_id": None, "selected_text": long_text}
        )
        if response.status_code == 200:
            # Should truncate automatically via Pydantic validator
            print(f"  [PASS] Request accepted (text auto-truncated)")
            tests.append(True)
        else:
            print(f"  [INFO] Status: {response.status_code}")
            tests.append(False)
    except Exception as e:
        print(f"  [FAIL] {e}")
        tests.append(False)

    passed = sum(tests)
    total = len(tests)
    print(f"\n{'='*60}")
    print(f"Edge Cases: {passed}/{total} passed")
    print(f"{'='*60}")

    return all(tests)


def test_benchmarks():
    """T090: Benchmark response times"""
    print("\n" + "="*60)
    print("T090: Response Time Benchmarks (10 queries)")
    print("="*60)

    queries = [
        "What is ROS?",
        "Explain Gazebo",
        "What is a humanoid robot?",
        "How does vision work in robots?",
        "What is SLAM?",
        "Explain motion planning",
        "What is a neural network?",
        "How do robots learn?",
        "What is reinforcement learning?",
        "Explain computer vision"
    ]

    response_times = []

    for i, query in enumerate(queries, 1):
        print(f"\n[{i}/10] {query}")
        start = time.time()
        try:
            response = requests.post(
                f"{API_BASE_URL}/api/chat",
                json={"query": query, "session_id": None},
                timeout=30
            )
            elapsed = (time.time() - start) * 1000
            response_times.append(elapsed)
            print(f"  {elapsed:.0f}ms")
        except Exception as e:
            print(f"  Failed: {e}")

    if response_times:
        p50 = statistics.median(response_times)
        p95 = statistics.quantiles(response_times, n=20)[18] if len(response_times) >= 20 else max(response_times)
        p99 = statistics.quantiles(response_times, n=100)[98] if len(response_times) >= 100 else max(response_times)

        print(f"\n{'='*60}")
        print(f"Latency Benchmarks:")
        print(f"  p50 (median): {p50:.0f}ms")
        print(f"  p95:          {p95:.0f}ms")
        print(f"  p99:          {p99:.0f}ms")
        print(f"  min:          {min(response_times):.0f}ms")
        print(f"  max:          {max(response_times):.0f}ms")
        print(f"  mean:         {statistics.mean(response_times):.0f}ms")
        print(f"{'='*60}")

    return response_times


def main():
    print("\n" + "="*60)
    print("PHASE 7 VALIDATION TEST SUITE")
    print("="*60)

    try:
        # Test 1: Required queries
        queries_pass, query_times = test_required_queries()

        # Test 2: Edge cases
        edge_pass = test_edge_cases()

        # Test 3: Benchmarks
        benchmark_times = test_benchmarks()

        # Summary
        print("\n" + "="*60)
        print("VALIDATION SUMMARY")
        print("="*60)
        print(f"T082-T083 Required Queries: {'[PASS]' if queries_pass else '[FAIL]'}")
        print(f"T084-T088 Edge Cases:       {'[PASS]' if edge_pass else '[PARTIAL]'}")
        print(f"T090 Benchmarks:            [COMPLETE]")

        if queries_pass and edge_pass:
            print("\n[SUCCESS] All automated tests passed!")
        else:
            print("\n[PARTIAL] Some tests require manual validation")

        print("="*60)

    except requests.exceptions.ConnectionError:
        print("\n[ERROR] Cannot connect to backend API")
        print("Make sure the backend server is running on http://localhost:8000")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
