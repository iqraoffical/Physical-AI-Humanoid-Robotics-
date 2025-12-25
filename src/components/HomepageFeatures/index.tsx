import type {ReactNode} from 'react';
import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type Message = {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
};

type FeatureItem = {
  title: string;
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
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

const FeatureList: FeatureItem[] = [
  {
    title: 'AI-Powered Q&A',
    description: (
      <>
        Ask questions about the textbook content and get accurate answers powered by our RAG system.
      </>
    ),
  },
  {
    title: 'Citation Support',
    description: (
      <>
        All answers come with citations so you can verify information and dive deeper into the content.
      </>
    ),
  },
  {
    title: 'Smart Context',
    description: (
      <>
        Our system understands context to provide relevant and comprehensive answers to your questions.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  const [messages, setMessages] = React.useState<Message[]>([
    { id: '1', content: 'Ask me anything about the textbook content!', role: 'assistant', timestamp: new Date() }
  ]);
  const [inputValue, setInputValue] = React.useState('');
  const [isLoading, setIsLoading] = React.useState(false);
  const [sessionId, setSessionId] = React.useState<string | null>(null);

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

    try {
      // Call the backend API
      const response = await fetch('https://iqra-physical-robotics-rag-deploy.hf.space/api/v1/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          session_id: sessionId || null,
          include_citations: true,
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      // Update session ID if we received a new one
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      // Add assistant message
      const assistantMessage: Message = {
        id: Date.now().toString(),
        content: data.response,
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
      setInputValue('');
    } catch (error) {
      console.error('Error:', error);
      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>

        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <Heading as="h2">Textbook Q&A Chatbot</Heading>
            <p>Ask questions about the textbook content</p>
          </div>

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={clsx(
                  styles.message,
                  styles[message.role]
                )}
              >
                <div className={styles.messageContent}>{message.content}</div>
                <div className={styles.messageTimestamp}>
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={clsx(styles.message, styles.assistant)}>
                <div className={styles.messageContent}>Thinking...</div>
              </div>
            )}
          </div>

          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about the textbook..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.chatButton}
              disabled={isLoading || !inputValue.trim()}
            >
              Send
            </button>
          </form>
        </div>
      </div>
    </section>
  );
}
