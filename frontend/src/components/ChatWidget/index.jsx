import React, { useState, useEffect, useRef } from 'react';

// Function to get API URL dynamically at runtime (not build time)
// This ensures the correct URL is used based on where the app is actually running
const getApiBaseUrl = () => {
    // During SSR, always return production URL (ChatWidget won't render during SSR anyway)
    if (typeof window === 'undefined') {
        return 'https://asim1112-humanoid-robotics-hackathon.hf.space';
    }

    // Runtime detection: check actual hostname in the browser
    const isLocalhost = window.location.hostname === 'localhost' ||
                       window.location.hostname === '127.0.0.1' ||
                       window.location.hostname === '';

    if (isLocalhost) {
        return 'http://localhost:8000';
    }

    // Production: use Hugging Face backend
    return 'https://asim1112-humanoid-robotics-hackathon.hf.space';
};

export default function ChatWidget() {
    const API_BASE_URL = getApiBaseUrl();
    const [messages, setMessages] = useState([]);
    const [sessionId, setSessionId] = useState(null);
    const [isOpen, setIsOpen] = useState(false);
    const [isLoading, setIsLoading] = useState(false);
    const [error, setError] = useState(null);
    const [selectedTextPreview, setSelectedTextPreview] = useState(null);
    const [inputValue, setInputValue] = useState('');
    const [isBrowser, setIsBrowser] = useState(false);
    const messagesEndRef = useRef(null);

    // Check if we're in the browser
    useEffect(() => {
        setIsBrowser(true);
    }, []);

    // Load session ID from localStorage on component mount
    useEffect(() => {
        if (typeof window !== 'undefined') {
            const storedSessionId = localStorage.getItem('chatSessionId');
            if (storedSessionId) {
                setSessionId(storedSessionId);
            }
        }
    }, []);

    // Load session history from backend when session ID is available
    useEffect(() => {
        if (sessionId) {
            const fetchSessionHistory = async () => {
                try {
                    const response = await fetch(`${API_BASE_URL}/api/sessions/${sessionId}`);
                    if (response.ok) {
                        const data = await response.json();
                        if (data.messages && Array.isArray(data.messages)) {
                            const formattedMessages = data.messages.map(msg => ({
                                id: msg.message_id,
                                text: msg.content,
                                sender: msg.role,
                                timestamp: msg.timestamp
                            }));
                            setMessages(formattedMessages);
                        }
                    }
                } catch (error) {
                    console.error('Failed to load session history:', error);
                    // Continue without history if loading fails
                }
            };

            fetchSessionHistory();
        }
    }, [sessionId]);

    // Function to get selected text from the page
    const getSelectedText = () => {
        const selection = window.getSelection();
        const text = selection?.toString().trim();
        // Limit to 5000 characters
        return text && text.length > 0 ? text.substring(0, 5000) : null;
    };

    // Auto-scroll to bottom when messages change
    useEffect(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages]);

    const handleSendMessage = async (e) => {
        e.preventDefault();

        if (!inputValue.trim() || isLoading) return;

        const query = inputValue.trim();
        setInputValue('');
        setIsLoading(true);
        setError(null);

        try {
            // Get selected text before sending the message
            const selectedText = getSelectedText();

            // Set preview for visual indicator
            if (selectedText) {
                setSelectedTextPreview(selectedText.substring(0, 100) + (selectedText.length > 100 ? '...' : ''));
            }

            // Add user message immediately
            const userMessage = {
                id: `user-${Date.now()}`,
                text: query,
                sender: 'user',
                timestamp: new Date().toISOString()
            };
            setMessages(prev => [...prev, userMessage]);

            const response = await fetch(`${API_BASE_URL}/api/chat`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    query: query,
                    session_id: sessionId,
                    selected_text: selectedText,
                })
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();

            // Update session ID if new session was created
            if (data.session_id && data.session_id !== sessionId) {
                setSessionId(data.session_id);
                if (typeof window !== 'undefined') {
                    localStorage.setItem('chatSessionId', data.session_id);
                }
            }

            // Add assistant response to chat history
            const assistantMessage = {
                id: `assistant-${Date.now()}`,
                text: data.response,
                sender: 'assistant',
                timestamp: new Date().toISOString()
            };

            setMessages(prev => [...prev, assistantMessage]);

            // Clear selected text preview after sending
            setSelectedTextPreview(null);
            // Clear browser selection
            window.getSelection()?.removeAllRanges();

        } catch (error) {
            console.error('Chat error:', error);
            setError('Failed to send message. Please try again.');
        } finally {
            setIsLoading(false);
        }
    };

    const toggleChat = () => {
        setIsOpen(!isOpen);
    };

    const closeChat = () => {
        setIsOpen(false);
    };

    const clearChat = async () => {
        if (sessionId) {
            try {
                const response = await fetch(`${API_BASE_URL}/api/sessions/${sessionId}`, {
                    method: 'DELETE'
                });

                if (response.ok) {
                    // Clear local storage and state
                    if (typeof window !== 'undefined') {
                        localStorage.removeItem('chatSessionId');
                    }
                    setMessages([]);
                    setSessionId(null);
                } else {
                    console.error('Failed to delete session:', response.status);
                }
            } catch (error) {
                console.error('Error deleting session:', error);
            }
        } else {
            // If no session exists, just clear local state
            if (typeof window !== 'undefined') {
                localStorage.removeItem('chatSessionId');
            }
            setMessages([]);
            setSessionId(null);
        }
    };

    // Don't render during SSR
    if (!isBrowser) {
        return null;
    }

    return (
        <div className="chat-widget">
            {!isOpen ? (
                <button
                    className="chat-toggle-button"
                    onClick={toggleChat}
                    style={{
                        position: 'fixed',
                        bottom: '20px',
                        right: '20px',
                        backgroundColor: '#10a37f',
                        color: 'white',
                        border: 'none',
                        borderRadius: '50%',
                        width: '60px',
                        height: '60px',
                        fontSize: '24px',
                        cursor: 'pointer',
                        zIndex: 1000,
                        boxShadow: '0 4px 8px rgba(0,0,0,0.2)'
                    }}
                >
                    💬
                </button>
            ) : (
                <div
                    className="chat-container"
                    style={{
                        position: 'fixed',
                        bottom: '20px',
                        right: '20px',
                        width: '350px',
                        height: '500px',
                        backgroundColor: 'white',
                        border: '1px solid #ccc',
                        borderRadius: '8px',
                        boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
                        zIndex: 1000,
                        display: 'flex',
                        flexDirection: 'column'
                    }}
                >
                    <div
                        className="chat-header"
                        style={{
                            backgroundColor: '#10a37f',
                            color: 'white',
                            padding: '10px',
                            borderRadius: '8px 8px 0 0',
                            display: 'flex',
                            justifyContent: 'space-between',
                            alignItems: 'center'
                        }}
                    >
                        <h3>Textbook Assistant</h3>
                        <div style={{ display: 'flex', gap: '10px' }}>
                            <button
                                onClick={clearChat}
                                title="Clear chat history"
                                style={{
                                    background: 'none',
                                    border: 'none',
                                    color: 'white',
                                    fontSize: '16px',
                                    cursor: 'pointer',
                                    padding: '0 5px'
                                }}
                            >
                                🗑️
                            </button>
                            <button
                                onClick={closeChat}
                                style={{
                                    background: 'none',
                                    border: 'none',
                                    color: 'white',
                                    fontSize: '18px',
                                    cursor: 'pointer'
                                }}
                            >
                                ×
                            </button>
                        </div>
                    </div>

                    <div className="chat-messages" style={{ flex: 1, padding: '10px', overflowY: 'auto' }}>
                        {messages.length === 0 && (
                            <div style={{ textAlign: 'center', color: '#666', padding: '20px' }}>
                                <p>👋 Welcome to the Textbook Assistant!</p>
                                <p style={{ fontSize: '14px' }}>Ask me anything about humanoid robotics, ROS2, Gazebo, or VLA models.</p>
                            </div>
                        )}
                        {messages.map((msg) => (
                            <div
                                key={msg.id}
                                style={{
                                    textAlign: msg.sender === 'user' ? 'right' : 'left',
                                    marginBottom: '10px'
                                }}
                            >
                                <div
                                    style={{
                                        display: 'inline-block',
                                        padding: '8px 12px',
                                        borderRadius: '8px',
                                        backgroundColor: msg.sender === 'user' ? '#e3f2fd' : '#f5f5f5',
                                        maxWidth: '80%',
                                        textAlign: 'left',
                                        wordWrap: 'break-word'
                                    }}
                                >
                                    {msg.text}
                                </div>
                            </div>
                        ))}
                        {isLoading && (
                            <div style={{ textAlign: 'left', marginBottom: '10px' }}>
                                <div
                                    style={{
                                        display: 'inline-block',
                                        padding: '8px 12px',
                                        borderRadius: '8px',
                                        backgroundColor: '#f5f5f5',
                                        maxWidth: '80%'
                                    }}
                                >
                                    <span className="typing-indicator">Thinking</span>
                                    <span className="dot-animation">...</span>
                                </div>
                            </div>
                        )}
                        {error && (
                            <div style={{ textAlign: 'left', marginBottom: '10px', color: 'red' }}>
                                <div
                                    style={{
                                        display: 'inline-block',
                                        padding: '8px 12px',
                                        borderRadius: '8px',
                                        backgroundColor: '#ffebee',
                                        maxWidth: '80%'
                                    }}
                                >
                                    {error}
                                </div>
                            </div>
                        )}
                        <div ref={messagesEndRef} />
                    </div>

                    {selectedTextPreview && (
                        <div
                            style={{
                                padding: '8px 10px',
                                backgroundColor: '#e3f2fd',
                                borderTop: '1px solid #bbdefb',
                                fontSize: '12px',
                                color: '#1976d2',
                                display: 'flex',
                                alignItems: 'center',
                                gap: '8px'
                            }}
                        >
                            <span style={{ fontWeight: 'bold' }}>📎 Selected text:</span>
                            <span style={{ fontStyle: 'italic', flex: 1, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                                {selectedTextPreview}
                            </span>
                            <button
                                onClick={() => {
                                    setSelectedTextPreview(null);
                                    window.getSelection()?.removeAllRanges();
                                }}
                                style={{
                                    background: 'none',
                                    border: 'none',
                                    color: '#1976d2',
                                    cursor: 'pointer',
                                    fontSize: '16px',
                                    padding: '0'
                                }}
                                title="Clear selection"
                            >
                                ×
                            </button>
                        </div>
                    )}

                    <div className="chat-input" style={{ padding: '10px', borderTop: '1px solid #eee' }}>
                        <form onSubmit={handleSendMessage} style={{ display: 'flex', gap: '8px' }}>
                            <input
                                type="text"
                                value={inputValue}
                                onChange={(e) => setInputValue(e.target.value)}
                                placeholder="Ask a question..."
                                disabled={isLoading}
                                style={{
                                    flex: 1,
                                    padding: '8px 12px',
                                    border: '1px solid #ddd',
                                    borderRadius: '4px',
                                    fontSize: '14px',
                                    outline: 'none'
                                }}
                                onFocus={(e) => e.target.style.borderColor = '#10a37f'}
                                onBlur={(e) => e.target.style.borderColor = '#ddd'}
                            />
                            <button
                                type="submit"
                                disabled={isLoading || !inputValue.trim()}
                                style={{
                                    backgroundColor: isLoading || !inputValue.trim() ? '#ccc' : '#10a37f',
                                    color: 'white',
                                    border: 'none',
                                    borderRadius: '4px',
                                    padding: '8px 16px',
                                    fontSize: '14px',
                                    cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer',
                                    fontWeight: '500'
                                }}
                            >
                                {isLoading ? 'Sending...' : 'Send'}
                            </button>
                        </form>
                    </div>
                </div>
            )}
        </div>
    );
}