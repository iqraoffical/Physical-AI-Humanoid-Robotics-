import type {ReactNode} from 'react';
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type Message = {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
};

type ChatResponse = {
  response: string;
  citations: Array<{
    content: string;
    source: string;
    page_number?: number;
  }>;
  session_id: string;
  confidence: number;
  query_id: string;
};

const RagChatbotPanel = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = React.useState<Message[]>([
    {
      id: '1',
      content: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the textbook content, and I\'ll help you understand the concepts better!',
      role: 'assistant',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = React.useState('');
  const [isLoading, setIsLoading] = React.useState(false);
  const [sessionId, setSessionId] = React.useState<string | null>(null);
  const [apiError, setApiError] = React.useState<string | null>(null);

  // Toggle chatbot panel
  const togglePanel = () => {
    setIsOpen(!isOpen);
  };

  // Handle form submission
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setApiError(null); // Reset any previous errors

    try {
      // Call the backend API with timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout

      const response = await fetch('https://iqra-physical-robotics-rag-deploy.hf.space/api/v1/query)', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          session_id: sessionId || null,
          include_citations: true,
        }),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}: ${response.statusText}`);
      }

      const data: ChatResponse = await response.json();

      // Update session ID if we received a new one
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      // Add assistant message with citations
      const citationText = data.citations && data.citations.length > 0
        ? `\n\nCitations:\n${data.citations.map((cit, idx) => `${idx + 1}. ${cit.content.substring(0, 100)}...`).join('\n')}`
        : '';

      // Format the assistant response to be more engaging and interactive
      let formattedResponse = data.response + citationText;

      // Add follow-up questions or hints if the response seems generic
      if (!formattedResponse.includes('?') && !formattedResponse.includes('Try') && !formattedResponse.includes('Consider')) {
        const followUpSuggestions = [
          "Would you like me to explain this concept in more detail?",
          "Can I provide an example to help illustrate this?",
          "Would you like to know how this connects to other concepts in the book?",
          "Do you have any follow-up questions about this topic?"
        ];
        const randomSuggestion = followUpSuggestions[Math.floor(Math.random() * followUpSuggestions.length)];
        formattedResponse += `\n\n💡 ${randomSuggestion}`;
      }

      const assistantMessage: Message = {
        id: Date.now().toString(),
        content: formattedResponse,
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
      setInputValue('');
    } catch (error: any) {
      console.error('Error:', error);

      // Check if it's a timeout error
      if (error.name === 'AbortError') {
        const errorMessage: Message = {
          id: Date.now().toString(),
          content: 'Request timed out. The server might be busy. Please try again.',
          role: 'assistant',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      } else {
        setApiError('API connection error');
        const errorMessage: Message = {
          id: Date.now().toString(),
          content: `I can only answer using the book content. ${error.message || 'Please try rephrasing your question.'}`,
          role: 'assistant',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } finally {
      setIsLoading(false);
    }
  };

  // Function to clear chat
  const handleClearChat = () => {
    setMessages([
      {
        id: '1',
        content: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the textbook content, and I\'ll help you understand the concepts better!',
        role: 'assistant',
        timestamp: new Date()
      }
    ]);
    setSessionId(null);
  };

  // Close panel when clicking outside
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      const panel = document.getElementById('rag-chatbot-panel');
      const button = document.getElementById('rag-chatbot-toggle');

      if (isOpen && panel && button &&
          !panel.contains(e.target as Node) &&
          !button.contains(e.target as Node)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  return (
    <>
      {/* Toggle Button */}
      <button
        id="rag-chatbot-toggle"
        className={clsx(styles.toggleButton, {
          [styles.open]: isOpen
        })}
        onClick={togglePanel}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
      >
        {isOpen ? '×' : '🤖'}
      </button>

      {/* Chatbot Panel */}
      {isOpen && (
        <div
          id="rag-chatbot-panel"
          className={styles.chatbotPanel}
        >
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <h3>AI Tutor</h3>
              <button
                onClick={handleClearChat}
                className={styles.clearChatButton}
                title="Clear chat"
              >
                Clear
              </button>
            </div>
            <p>Physical AI & Humanoid Robotics Assistant</p>
          </div>

          {apiError && (
            <div className={styles.apiError}>
              <p>⚠️ The chatbot service is temporarily unavailable. Please try again later.</p>
            </div>
          )}

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={clsx(
                  styles.message,
                  styles[message.role]
                )}
              >
                <div className={styles.messageContent}>
                  {message.role === 'assistant' && (
                    <span className={styles.botIcon}>🎓</span>
                  )}
                  {message.role === 'user' && (
                    <span className={styles.userIcon}>👤</span>
                  )}
                  <span className={styles.messageText}>{message.content}</span>
                </div>
                <div className={styles.messageTimestamp}>
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={clsx(styles.message, styles.assistant)}>
                <div className={styles.messageContent}>
                  <span className={styles.botIcon}>🎓</span>
                  <span className={styles.messageText}>Thinking...</span>
                </div>
              </div>
            )}
          </div>

          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about the textbook content..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.chatButton}
              disabled={isLoading || !inputValue.trim()}
            >
              {isLoading ? '...' : '→'}
            </button>
          </form>

          <div className={styles.chatFooter}>
            <small>I can only answer using book content.</small>
          </div>
        </div>
      )}
    </>
  );
};

export default RagChatbotPanel;