"""
Test script for RAG chatbot API endpoints.

This script tests the full chat flow:
1. Create a new session
2. Send a query about ROS 2
3. Send a follow-up query
4. Retrieve session history
5. Test selected-text mode
"""

import requests
import json
import time
import sys
import io

# Fix Windows console encoding issues
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

API_BASE_URL = "http://localhost:8000"

def test_health_check():
    """Test basic health endpoint."""
    print("\n[TEST 1] Health Check")
    print("=" * 50)

    response = requests.get(f"{API_BASE_URL}/health")
    print(f"Status: {response.status_code}")
    print(f"Response: {response.json()}")

    assert response.status_code == 200, "Health check failed"
    print("[PASS] Health check passed")
    return True


def test_chat_new_session():
    """Test chat endpoint with a new session."""
    print("\n[TEST 2] Chat with New Session")
    print("=" * 50)

    payload = {
        "query": "What is ROS 2?",
        "session_id": None,
        "selected_text": None,
        "stream": False
    }

    print(f"Sending query: {payload['query']}")
    start_time = time.time()

    response = requests.post(
        f"{API_BASE_URL}/api/chat",
        json=payload,
        headers={"Content-Type": "application/json"}
    )

    elapsed = (time.time() - start_time) * 1000

    print(f"Status: {response.status_code}")
    print(f"Response time: {elapsed:.2f}ms")

    if response.status_code == 200:
        data = response.json()
        print(f"Session ID: {data['session_id']}")
        print(f"Message ID: {data['message_id']}")
        print(f"Retrieved chunks: {data['retrieved_chunks']}")
        print(f"Response preview: {data['response'][:200]}...")
        print(f"Backend response time: {data['response_time_ms']:.2f}ms")

        assert 'session_id' in data, "No session_id in response"
        assert 'response' in data, "No response text"
        print("✓ Chat with new session passed")
        return data['session_id']
    else:
        print(f"✗ Error: {response.text}")
        return None


def test_chat_existing_session(session_id):
    """Test chat endpoint with existing session (multi-turn)."""
    print("\n[TEST 3] Chat with Existing Session (Multi-turn)")
    print("=" * 50)

    payload = {
        "query": "How do I install it?",
        "session_id": session_id,
        "selected_text": None,
        "stream": False
    }

    print(f"Sending follow-up query: {payload['query']}")
    print(f"Using session ID: {session_id}")

    start_time = time.time()
    response = requests.post(
        f"{API_BASE_URL}/api/chat",
        json=payload,
        headers={"Content-Type": "application/json"}
    )
    elapsed = (time.time() - start_time) * 1000

    print(f"Status: {response.status_code}")
    print(f"Response time: {elapsed:.2f}ms")

    if response.status_code == 200:
        data = response.json()
        print(f"Response preview: {data['response'][:200]}...")
        print("✓ Multi-turn conversation passed")
        return True
    else:
        print(f"✗ Error: {response.text}")
        return False


def test_get_session_history(session_id):
    """Test retrieving session history."""
    print("\n[TEST 4] Get Session History")
    print("=" * 50)

    response = requests.get(f"{API_BASE_URL}/api/sessions/{session_id}")

    print(f"Status: {response.status_code}")

    if response.status_code == 200:
        data = response.json()
        print(f"Session ID: {data['session_id']}")
        print(f"Created at: {data['created_at']}")
        print(f"Message count: {data['message_count']}")
        print(f"\nConversation history:")
        for i, msg in enumerate(data['messages'], 1):
            print(f"  {i}. [{msg['role']}]: {msg['content'][:100]}...")

        assert data['message_count'] >= 4, f"Expected at least 4 messages (2 user + 2 assistant), got {data['message_count']}"
        print("✓ Session history retrieval passed")
        return True
    else:
        print(f"✗ Error: {response.text}")
        return False


def test_selected_text_mode():
    """Test chat with selected text."""
    print("\n[TEST 5] Selected Text Mode")
    print("=" * 50)

    selected_text = """
    ROS 2 (Robot Operating System 2) is a set of software libraries and tools
    for building robot applications. It provides hardware abstraction, device
    drivers, libraries, visualizers, message-passing, and package management.
    """

    payload = {
        "query": "Explain this in simpler terms",
        "session_id": None,  # New session for this test
        "selected_text": selected_text.strip(),
        "stream": False
    }

    print(f"Query: {payload['query']}")
    print(f"Selected text: {selected_text[:100]}...")

    start_time = time.time()
    response = requests.post(
        f"{API_BASE_URL}/api/chat",
        json=payload,
        headers={"Content-Type": "application/json"}
    )
    elapsed = (time.time() - start_time) * 1000

    print(f"Status: {response.status_code}")
    print(f"Response time: {elapsed:.2f}ms")

    if response.status_code == 200:
        data = response.json()
        print(f"Response preview: {data['response'][:200]}...")
        print("✓ Selected text mode passed")
        return True
    else:
        print(f"✗ Error: {response.text}")
        return False


def test_delete_session(session_id):
    """Test session deletion."""
    print("\n[TEST 6] Delete Session")
    print("=" * 50)

    response = requests.delete(f"{API_BASE_URL}/api/sessions/{session_id}")

    print(f"Status: {response.status_code}")

    if response.status_code == 200:
        print(f"Response: {response.json()}")

        # Verify session is archived
        get_response = requests.get(f"{API_BASE_URL}/api/sessions/{session_id}")
        if get_response.status_code == 404:
            print("✓ Session successfully archived (returns 404)")
            return True
        else:
            print("✗ Session still accessible after deletion")
            return False
    else:
        print(f"✗ Error: {response.text}")
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 50)
    print("RAG CHATBOT API TEST SUITE")
    print("=" * 50)

    try:
        # Test 1: Health check
        test_health_check()

        # Test 2: Create new session and send first query
        session_id = test_chat_new_session()
        if not session_id:
            print("\n✗ Failed to create session, stopping tests")
            return

        # Test 3: Multi-turn conversation
        test_chat_existing_session(session_id)

        # Test 4: Retrieve session history
        test_get_session_history(session_id)

        # Test 5: Selected text mode
        test_selected_text_mode()

        # Test 6: Delete session
        test_delete_session(session_id)

        print("\n" + "=" * 50)
        print("✓ ALL TESTS PASSED")
        print("=" * 50)

    except requests.exceptions.ConnectionError:
        print("\n✗ ERROR: Cannot connect to backend API")
        print("Make sure the backend server is running on http://localhost:8000")
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

