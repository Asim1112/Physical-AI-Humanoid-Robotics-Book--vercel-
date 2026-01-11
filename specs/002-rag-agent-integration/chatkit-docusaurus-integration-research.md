# Research: OpenAI ChatKit SDK Integration with Docusaurus

**Context**: Embedding a chat widget in a Docusaurus site for RAG chatbot with React-based Docusaurus frontend, chat widget connecting to FastAPI backend, support for selected-text mode, and responsive design.

**Date**: 2025-12-29
**Feature**: 002-rag-agent-integration
**Status**: Research Complete

---

## 1. ChatKit SDK Installation & Setup

### Decision: Use Custom Chat Component Instead of OpenAI ChatKit

**Rationale**: After investigating OpenAI ChatKit SDK, it appears that OpenAI does not currently offer a public "ChatKit SDK" npm package. The OpenAI ecosystem primarily provides:
- OpenAI API (for LLM interactions)
- OpenAI Agents SDK (for backend agent orchestration)
- No official pre-built React chat UI component library

For Docusaurus integration, the recommended approach is to build a custom React chat component that connects to your FastAPI backend, which uses the OpenAI Agents SDK for agent orchestration.

**Recommended Approach**:

```bash
# Install necessary dependencies for custom chat widget
cd frontend
npm install --save \
  react-markdown \
  remark-gfm \
  react-syntax-highlighter \
  @mui/material @emotion/react @emotion/styled
```

**Component Structure**:

```javascript
// frontend/src/components/ChatWidget/ChatWidget.jsx
import React, { useState, useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { vscDarkPlus } from 'react-syntax-highlighter/dist/esm/styles/prism';

const ChatWidget = ({ apiBaseUrl = 'http://localhost:8000' }) => {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = {
      role: 'user',
      content: input,
      selectedText: selectedText || null,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(`${apiBaseUrl}/api/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: input,
          selected_text: selectedText || null,
          session_id: sessionStorage.getItem('chat_session_id')
        })
      });

      if (!response.ok) throw new Error('Failed to get response');

      const data = await response.json();

      // Store session ID for multi-turn conversations
      if (data.session_id) {
        sessionStorage.setItem('chat_session_id', data.session_id);
      }

      const assistantMessage = {
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText(''); // Clear selected text after use
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, {
        role: 'error',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`chat-widget ${isOpen ? 'open' : 'closed'}`}>
      {/* Chat widget implementation */}
    </div>
  );
};

export default ChatWidget;
```

**Alternatives Considered**:

1. **Third-Party Chat UI Libraries** (e.g., react-chat-widget, react-chatbot-kit):
   - **Trade-offs**: Pre-built functionality but less flexibility for RAG-specific features (source citations, selected-text mode)
   - **Decision**: Custom component provides better control for textbook-specific features

2. **Headless Chat Libraries** (e.g., chatscope/chat-ui-kit):
   - **Trade-offs**: More flexibility than pre-built widgets but still constrained by library design patterns
   - **Decision**: Full custom component allows seamless Docusaurus theming integration

3. **OpenAI Platform UI Components** (hypothetical):
   - **Trade-offs**: Would offer official support but doesn't exist as public SDK
   - **Decision**: Not available, must build custom

---

## 2. Docusaurus Integration

### Decision: Client Module with Root Component Wrapper

**Rationale**: Docusaurus provides multiple integration patterns for custom React components. For a global chat widget that should appear on all pages, the **Client Module** approach with a Root component wrapper is optimal. This balances functionality (always available, maintains state) with simplicity (single configuration point, minimal code).

**Implementation**:

**Step 1: Create Client Module**

```javascript
// frontend/src/modules/chatWidget.js
import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Only render on client-side (not during SSR)
export default (function chatWidgetModule() {
  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  return {
    onRouteUpdate({ location, previousLocation }) {
      // Optional: Track page changes for analytics
      console.log('Route changed:', location.pathname);
    },
  };
})();

// Export the root wrapper
export function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget apiBaseUrl={process.env.REACT_APP_API_URL || 'http://localhost:8000'} />
    </>
  );
}
```

**Step 2: Register Client Module in docusaurus.config.js**

```javascript
// frontend/docusaurus.config.js
const config = {
  // ... existing config

  clientModules: [
    require.resolve('./src/modules/chatWidget.js'),
  ],

  // Custom fields for environment-specific configuration
  customFields: {
    apiBaseUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],
};
```

**Step 3: Add Environment-Specific Configuration**

```javascript
// frontend/.env.development
REACT_APP_API_URL=http://localhost:8000

// frontend/.env.production
REACT_APP_API_URL=https://your-fastapi-backend.vercel.app
```

**Code Example - Complete Integration**:

```javascript
// frontend/src/theme/Root.js
import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// This wraps the entire Docusaurus app
export default function Root({ children }) {
  const { siteConfig } = useDocusaurusContext();
  const apiBaseUrl = siteConfig.customFields?.apiBaseUrl;

  return (
    <>
      {children}
      <ChatWidget
        apiBaseUrl={apiBaseUrl}
        theme={siteConfig.themeConfig.colorMode.defaultMode}
      />
    </>
  );
}
```

**Alternatives Considered**:

1. **Theme Swizzling (Classic Theme)**:
   - **Approach**: Swizzle the `Layout` component and inject chat widget
   - **Trade-offs**: More control over exact placement, but requires maintaining swizzled component across Docusaurus updates
   - **Code Example**:
     ```bash
     npm run swizzle @docusaurus/theme-classic Layout -- --wrap
     ```
   - **Why Not Chosen**: Higher maintenance burden; client module is cleaner

2. **Page-Specific Integration**:
   - **Approach**: Import and render ChatWidget in specific MDX pages
   - **Trade-offs**: Fine-grained control, but chat state resets between pages
   - **Code Example**:
     ```mdx
     ---
     title: Module 1
     ---
     import ChatWidget from '@site/src/components/ChatWidget';

     # Module 1 Content

     <ChatWidget />
     ```
   - **Why Not Chosen**: Doesn't support global state across pages

3. **Plugin Development**:
   - **Approach**: Create custom Docusaurus plugin for chat widget
   - **Trade-offs**: Most flexible, reusable across projects, but higher complexity
   - **Code Example**:
     ```javascript
     // plugins/chat-widget-plugin/index.js
     module.exports = function (context, options) {
       return {
         name: 'chat-widget-plugin',
         getClientModules() {
           return [require.resolve('./chatWidget')];
         },
       };
     };
     ```
   - **Why Not Chosen**: Overkill for single-project use; client module suffices

---

## 3. Selected-Text Capture

### Decision: Global Event Listener with Floating Button Trigger

**Rationale**: The optimal pattern for selected-text capture is a global `selectionchange` event listener combined with a floating button that appears when text is selected. This provides clear user affordance (visible button), handles all edge cases (code blocks, special characters), and integrates seamlessly with the chat widget.

**Implementation**:

```javascript
// frontend/src/components/ChatWidget/TextSelectionHandler.jsx
import React, { useEffect, useState } from 'react';

const TextSelectionHandler = ({ onTextSelected }) => {
  const [selectedText, setSelectedText] = useState('');
  const [buttonPosition, setButtonPosition] = useState({ x: 0, y: 0 });
  const [showButton, setShowButton] = useState(false);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(text);
        setButtonPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 40 // Position above selection
        });
        setShowButton(true);
      } else {
        setShowButton(false);
      }
    };

    // Listen for selection changes
    document.addEventListener('selectionchange', handleSelection);

    // Hide button on click outside
    const handleClickOutside = (e) => {
      if (!e.target.closest('.selection-button')) {
        setShowButton(false);
      }
    };
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('selectionchange', handleSelection);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  const handleAskAboutSelection = () => {
    onTextSelected(selectedText);
    setShowButton(false);
    window.getSelection().removeAllRanges(); // Clear selection
  };

  if (!showButton) return null;

  return (
    <button
      className="selection-button"
      style={{
        position: 'fixed',
        left: `${buttonPosition.x}px`,
        top: `${buttonPosition.y}px`,
        transform: 'translateX(-50%)',
        zIndex: 9999,
      }}
      onClick={handleAskAboutSelection}
    >
      Ask about this
    </button>
  );
};

export default TextSelectionHandler;
```

**Integration with ChatWidget**:

```javascript
// frontend/src/components/ChatWidget/ChatWidget.jsx
import TextSelectionHandler from './TextSelectionHandler';

const ChatWidget = ({ apiBaseUrl }) => {
  const [selectedText, setSelectedText] = useState('');

  const handleTextSelected = (text) => {
    setSelectedText(text);
    setIsOpen(true); // Open chat widget
    setInput(`Explain this: "${text.substring(0, 50)}..."`); // Pre-fill input
  };

  return (
    <>
      <TextSelectionHandler onTextSelected={handleTextSelected} />
      <div className="chat-widget">
        {/* Chat widget UI */}
      </div>
    </>
  );
};
```

**Edge Case Handling**:

```javascript
// Enhanced selection handler with edge case management
const handleSelection = () => {
  const selection = window.getSelection();
  let text = selection.toString().trim();

  // Edge case 1: Code blocks - preserve formatting
  const isCodeBlock = selection.anchorNode?.parentElement?.closest('pre, code');
  if (isCodeBlock) {
    text = selection.toString(); // Keep whitespace for code
  }

  // Edge case 2: Multi-line selections - normalize whitespace
  if (text.includes('\n') && !isCodeBlock) {
    text = text.replace(/\s+/g, ' ').trim();
  }

  // Edge case 3: Special characters - sanitize for API
  const sanitizedText = text
    .replace(/[\u200B-\u200D\uFEFF]/g, '') // Remove zero-width characters
    .replace(/[""]/g, '"') // Normalize quotes
    .replace(/['']/g, "'");

  // Edge case 4: Minimum selection length
  if (sanitizedText.length < 3) {
    setShowButton(false);
    return;
  }

  // Edge case 5: Maximum selection length (prevent oversized context)
  const maxLength = 500;
  const finalText = sanitizedText.length > maxLength
    ? sanitizedText.substring(0, maxLength) + '...'
    : sanitizedText;

  setSelectedText(finalText);
  setShowButton(true);
};
```

**Alternatives Considered**:

1. **Context Menu Integration**:
   - **Approach**: Add "Ask AI" to browser right-click context menu
   - **Trade-offs**: More native feel, but limited browser support and UX discoverability
   - **Code Example**:
     ```javascript
     useEffect(() => {
       const handleContextMenu = (e) => {
         const selection = window.getSelection().toString();
         if (selection) {
           e.preventDefault();
           // Show custom context menu
         }
       };
       document.addEventListener('contextmenu', handleContextMenu);
     }, []);
     ```
   - **Why Not Chosen**: Browser restrictions, poor mobile support

2. **Inline Tooltip on Selection**:
   - **Approach**: Show tooltip immediately adjacent to selected text
   - **Trade-offs**: Very discoverable, but can interfere with text selection UX
   - **Code Example**:
     ```javascript
     const rect = range.getBoundingClientRect();
     setTooltipPosition({
       x: rect.right + 5,
       y: rect.top
     });
     ```
   - **Why Not Chosen**: Can block content, less predictable positioning

3. **Dedicated Selection Panel**:
   - **Approach**: Fixed sidebar panel that always shows current selection
   - **Trade-offs**: Always visible, good for power users, but takes screen space
   - **Why Not Chosen**: Not suitable for mobile, reduces content area

---

## 4. Widget Customization

### Decision: CSS Modules with Docusaurus Theme Tokens

**Rationale**: Use CSS Modules for component-scoped styling while leveraging Docusaurus theme tokens (CSS variables) for colors, spacing, and responsive breakpoints. This approach balances custom styling needs with seamless theme integration, ensuring the chat widget automatically adapts to light/dark mode and maintains visual consistency with the Docusaurus site.

**Implementation**:

**Step 1: Create CSS Module**

```css
/* frontend/src/components/ChatWidget/ChatWidget.module.css */

.chatWidget {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 380px;
  max-width: calc(100vw - 40px);
  height: 600px;
  max-height: calc(100vh - 100px);

  background-color: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 12px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);

  display: flex;
  flex-direction: column;
  z-index: 1000;
  transition: transform 0.3s ease, opacity 0.3s ease;
}

.chatWidget.closed {
  transform: translateY(calc(100% + 20px));
  opacity: 0;
  pointer-events: none;
}

.chatHeader {
  padding: 16px;
  background: var(--ifm-color-primary);
  color: white;
  border-radius: 12px 12px 0 0;
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-weight: 600;
}

.messagesContainer {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  background: var(--ifm-background-color);
}

.message {
  margin-bottom: 16px;
  padding: 12px;
  border-radius: 8px;
  max-width: 85%;
}

.messageUser {
  background: var(--ifm-color-primary-lightest);
  margin-left: auto;
  text-align: right;
}

.messageAssistant {
  background: var(--ifm-color-emphasis-100);
}

.messageError {
  background: var(--ifm-color-danger-lightest);
  color: var(--ifm-color-danger-dark);
}

.sourceCitations {
  margin-top: 12px;
  padding-top: 12px;
  border-top: 1px solid var(--ifm-color-emphasis-200);
  font-size: 0.85rem;
  color: var(--ifm-color-emphasis-700);
}

.inputContainer {
  padding: 16px;
  background: var(--ifm-background-surface-color);
  border-top: 1px solid var(--ifm-color-emphasis-200);
  display: flex;
  gap: 8px;
}

.input {
  flex: 1;
  padding: 10px 14px;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  font-family: var(--ifm-font-family-base);
}

.sendButton {
  padding: 10px 20px;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  font-weight: 600;
  transition: background 0.2s;
}

.sendButton:hover {
  background: var(--ifm-color-primary-dark);
}

.sendButton:disabled {
  background: var(--ifm-color-emphasis-300);
  cursor: not-allowed;
}

.toggleButton {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 999;
  transition: transform 0.2s, background 0.2s;
}

.toggleButton:hover {
  transform: scale(1.1);
  background: var(--ifm-color-primary-dark);
}

/* Responsive Design */
@media (max-width: 768px) {
  .chatWidget {
    width: 100vw;
    height: 100vh;
    max-width: 100vw;
    max-height: 100vh;
    bottom: 0;
    right: 0;
    border-radius: 0;
  }

  .chatHeader {
    border-radius: 0;
  }
}

@media (max-width: 480px) {
  .message {
    max-width: 90%;
  }

  .inputContainer {
    padding: 12px;
  }
}

/* Dark Mode Specific Adjustments */
[data-theme='dark'] .chatWidget {
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.4);
}

[data-theme='dark'] .messageUser {
  background: var(--ifm-color-primary-darker);
}
```

**Step 2: Apply CSS Module in Component**

```javascript
// frontend/src/components/ChatWidget/ChatWidget.jsx
import React, { useState } from 'react';
import styles from './ChatWidget.module.css';
import ReactMarkdown from 'react-markdown';

const ChatWidget = ({ apiBaseUrl }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  return (
    <>
      {isOpen && (
        <div className={`${styles.chatWidget} ${!isOpen && styles.closed}`}>
          <div className={styles.chatHeader}>
            <span>AI Textbook Assistant</span>
            <button onClick={() => setIsOpen(false)}>✕</button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${
                  msg.role === 'user' ? styles.messageUser :
                  msg.role === 'error' ? styles.messageError :
                  styles.messageAssistant
                }`}
              >
                <ReactMarkdown>{msg.content}</ReactMarkdown>

                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sourceCitations}>
                    <strong>Sources:</strong>
                    <ul>
                      {msg.sources.map((src, i) => (
                        <li key={i}>
                          {src.module_name} - Score: {src.similarity_score.toFixed(2)}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
          </div>

          <div className={styles.inputContainer}>
            <input
              className={styles.input}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
              placeholder="Ask about the textbook..."
            />
            <button
              className={styles.sendButton}
              onClick={sendMessage}
              disabled={!input.trim() || isLoading}
            >
              Send
            </button>
          </div>
        </div>
      )}

      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
      >
        💬
      </button>
    </>
  );
};
```

**Responsive Behavior Strategy**:

```javascript
// frontend/src/components/ChatWidget/useResponsive.js
import { useState, useEffect } from 'react';

export const useResponsive = () => {
  const [isMobile, setIsMobile] = useState(false);
  const [isTablet, setIsTablet] = useState(false);

  useEffect(() => {
    const handleResize = () => {
      setIsMobile(window.innerWidth < 768);
      setIsTablet(window.innerWidth >= 768 && window.innerWidth < 1024);
    };

    handleResize(); // Initial check
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  return { isMobile, isTablet };
};

// Usage in ChatWidget
const ChatWidget = ({ apiBaseUrl }) => {
  const { isMobile, isTablet } = useResponsive();

  const widgetConfig = {
    width: isMobile ? '100vw' : isTablet ? '350px' : '380px',
    height: isMobile ? '100vh' : '600px',
    position: isMobile ? 'fullscreen' : 'floating'
  };

  return (
    <div className={styles.chatWidget} data-mobile={isMobile}>
      {/* Widget content */}
    </div>
  );
};
```

**Alternatives Considered**:

1. **Material-UI (MUI) Components**:
   - **Approach**: Use MUI for pre-built chat UI components (Card, TextField, Button)
   - **Trade-offs**: Fast development, consistent design system, but adds ~200KB bundle size
   - **Code Example**:
     ```javascript
     import { Card, TextField, Button, Avatar } from '@mui/material';

     const ChatWidget = () => (
       <Card sx={{ position: 'fixed', bottom: 20, right: 20 }}>
         <TextField label="Ask a question" />
         <Button variant="contained">Send</Button>
       </Card>
     );
     ```
   - **Why Not Chosen**: Bundle size impact, potential theme conflicts with Docusaurus

2. **Tailwind CSS**:
   - **Approach**: Use utility classes for styling
   - **Trade-offs**: Rapid prototyping, but requires Docusaurus Tailwind plugin setup
   - **Code Example**:
     ```javascript
     <div className="fixed bottom-5 right-5 w-96 h-[600px] bg-white dark:bg-gray-800 rounded-xl shadow-2xl">
       {/* Widget content */}
     </div>
     ```
   - **Why Not Chosen**: Additional build configuration, preference for Docusaurus native tokens

3. **Inline Styles with Theme Context**:
   - **Approach**: Use JavaScript objects for styles, accessing theme programmatically
   - **Trade-offs**: Dynamic theming, but less performant and harder to maintain
   - **Code Example**:
     ```javascript
     import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

     const ChatWidget = () => {
       const { siteConfig } = useDocusaurusContext();
       const primaryColor = siteConfig.themeConfig.colors.primary;

       return <div style={{ background: primaryColor }}>...</div>;
     };
     ```
   - **Why Not Chosen**: Performance concerns, CSS Modules provide better encapsulation

---

## 5. Backend Connection

### Decision: Environment-Based API Configuration with Fetch API

**Rationale**: Use environment variables to configure the API base URL for different deployment environments (localhost development, Vercel production). The standard Fetch API provides sufficient functionality for HTTP requests without additional dependencies. This approach is simple, standard, and compatible with Docusaurus's build process.

**Implementation**:

**Step 1: Environment Configuration**

```bash
# frontend/.env.development
REACT_APP_API_URL=http://localhost:8000

# frontend/.env.production
REACT_APP_API_URL=https://humanoid-robotics-api.vercel.app
```

**Step 2: API Client Module**

```javascript
// frontend/src/services/chatApi.js

class ChatApiClient {
  constructor(baseUrl) {
    this.baseUrl = baseUrl || process.env.REACT_APP_API_URL || 'http://localhost:8000';
  }

  async sendMessage({ query, selectedText = null, sessionId = null }) {
    try {
      const response = await fetch(`${this.baseUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          selected_text: selectedText,
          session_id: sessionId,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Chat API error:', error);
      throw error;
    }
  }

  async streamMessage({ query, selectedText = null, sessionId = null }) {
    try {
      const response = await fetch(`${this.baseUrl}/api/chat/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          selected_text: selectedText,
          session_id: sessionId,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return response.body.getReader();
    } catch (error) {
      console.error('Chat stream error:', error);
      throw error;
    }
  }

  async healthCheck() {
    try {
      const response = await fetch(`${this.baseUrl}/health`);
      return response.ok;
    } catch {
      return false;
    }
  }
}

export default new ChatApiClient();
```

**Step 3: Integration in ChatWidget**

```javascript
// frontend/src/components/ChatWidget/ChatWidget.jsx
import React, { useState, useEffect } from 'react';
import chatApi from '../../services/chatApi';

const ChatWidget = () => {
  const [isBackendHealthy, setIsBackendHealthy] = useState(null);
  const [messages, setMessages] = useState([]);
  const [sessionId, setSessionId] = useState(null);

  // Check backend health on mount
  useEffect(() => {
    const checkHealth = async () => {
      const healthy = await chatApi.healthCheck();
      setIsBackendHealthy(healthy);

      if (!healthy) {
        console.warn('Backend API is not available');
      }
    };

    checkHealth();
  }, []);

  const sendMessage = async () => {
    if (!input.trim()) return;

    try {
      const response = await chatApi.sendMessage({
        query: input,
        selectedText: selectedText || null,
        sessionId: sessionId
      });

      // Update session ID for multi-turn conversations
      if (response.session_id) {
        setSessionId(response.session_id);
      }

      setMessages(prev => [...prev, {
        role: 'assistant',
        content: response.response,
        sources: response.sources || [],
        timestamp: new Date().toISOString()
      }]);
    } catch (error) {
      setMessages(prev => [...prev, {
        role: 'error',
        content: 'Failed to connect to backend. Please try again.',
        timestamp: new Date().toISOString()
      }]);
    }
  };

  return (
    <div className="chat-widget">
      {isBackendHealthy === false && (
        <div className="connection-warning">
          ⚠️ Backend unavailable. Running in offline mode.
        </div>
      )}
      {/* Rest of widget UI */}
    </div>
  );
};
```

**Environment Variable Access in Docusaurus**:

```javascript
// frontend/docusaurus.config.js
require('dotenv').config();

const config = {
  // ... other config

  customFields: {
    apiBaseUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
  },
};

// Usage in components
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const ChatWidget = () => {
  const { siteConfig } = useDocusaurusContext();
  const apiUrl = siteConfig.customFields.apiBaseUrl;

  // Use apiUrl for API calls
};
```

**CORS Configuration for Backend (FastAPI)**:

```python
# backend/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# Configure CORS for Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "https://your-docusaurus-site.vercel.app",  # Production
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    # Handle chat request
    pass
```

**Alternatives Considered**:

1. **Axios Library**:
   - **Approach**: Use Axios for HTTP requests with interceptors and automatic transforms
   - **Trade-offs**: Richer feature set (interceptors, automatic JSON parsing), but adds dependency
   - **Code Example**:
     ```javascript
     import axios from 'axios';

     const api = axios.create({
       baseURL: process.env.REACT_APP_API_URL,
       timeout: 10000,
     });

     api.interceptors.response.use(
       response => response.data,
       error => Promise.reject(error)
     );

     export default api;
     ```
   - **Why Not Chosen**: Fetch API is sufficient for simple requests, avoids extra dependency

2. **React Query (TanStack Query)**:
   - **Approach**: Use React Query for server state management and caching
   - **Trade-offs**: Excellent caching, automatic retries, loading states, but higher complexity
   - **Code Example**:
     ```javascript
     import { useQuery, useMutation } from '@tanstack/react-query';

     const useSendMessage = () => {
       return useMutation({
         mutationFn: (data) => chatApi.sendMessage(data),
         onSuccess: (response) => {
           // Handle success
         },
       });
     };
     ```
   - **Why Not Chosen**: Overkill for simple chat interface, adds complexity

3. **GraphQL with Apollo Client**:
   - **Approach**: Use GraphQL for more structured API queries
   - **Trade-offs**: Type-safe queries, efficient data fetching, but requires backend GraphQL support
   - **Why Not Chosen**: FastAPI backend uses REST, GraphQL adds unnecessary complexity

---

## Summary: Recommended Implementation Path

### Phase 1: Core Chat Widget (MVP)
1. Create custom React chat component with CSS Modules
2. Integrate via Docusaurus Root wrapper (client module)
3. Connect to FastAPI backend using Fetch API
4. Environment-based API configuration

### Phase 2: Selected-Text Mode
1. Implement global selection listener
2. Add floating button trigger
3. Edge case handling (code blocks, special characters)
4. Pre-fill chat input with selected context

### Phase 3: Polish & Optimization
1. Responsive design refinements
2. Dark mode theming
3. Error handling and offline mode
4. Source citations and module references

### Key Files to Create

```
frontend/
├── src/
│   ├── components/
│   │   └── ChatWidget/
│   │       ├── ChatWidget.jsx
│   │       ├── ChatWidget.module.css
│   │       ├── TextSelectionHandler.jsx
│   │       └── MessageList.jsx
│   ├── services/
│   │   └── chatApi.js
│   ├── theme/
│   │   └── Root.js
│   └── css/
│       └── custom.css (augment for chat widget)
├── .env.development
├── .env.production
└── docusaurus.config.js (update with customFields)
```

### Testing Strategy

1. **Local Development**: Test with `npm start` + FastAPI backend on localhost:8000
2. **Production Simulation**: Build with `npm run build` and test environment variable substitution
3. **Responsive Testing**: Test on mobile (375px), tablet (768px), desktop (1920px)
4. **Edge Cases**: Test selection in code blocks, long text, special characters
5. **Integration Testing**: Verify API connection, session persistence, error states

---

## Additional Considerations

### Performance Optimization
- Lazy load chat widget component to reduce initial bundle size
- Implement message virtualization for long conversation histories
- Use React.memo() for message components to prevent unnecessary re-renders
- Debounce selection events to reduce event handler overhead

### Accessibility
- Ensure keyboard navigation (Tab, Enter, Esc)
- Add ARIA labels for screen readers
- Provide focus indicators for interactive elements
- Support voice input (optional enhancement)

### Security
- Sanitize user input before sending to backend
- Validate API responses to prevent XSS attacks
- Implement rate limiting on frontend (prevent spam)
- Use HTTPS in production for API communication

### Analytics & Monitoring
- Track widget open/close events
- Log query types and response times
- Monitor error rates and backend connectivity
- A/B test widget positioning and design variations

---

**Next Steps**: Proceed with implementation of Phase 1 (Core Chat Widget) followed by incremental enhancements in Phases 2-3.
