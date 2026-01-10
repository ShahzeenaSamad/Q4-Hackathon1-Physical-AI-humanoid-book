import React, { useState, useEffect } from 'react';
import ChatWindow from './ChatWindow';
import styles from './styles.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapter_id: string;
    section_title: string;
  }>;
}

const ChatbotWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);

  const { siteConfig } = useDocusaurusContext();
  // @ts-ignore
  const backendUrl = siteConfig.themeConfig.chatWidget?.backendUrl || 'http://localhost:8001';  // Using port 8001 for demo

  useEffect(() => {
    // Load session from local storage
    const savedSessionId = localStorage.getItem('chat_session_id');
    if (savedSessionId) {
      setSessionId(savedSessionId);
      loadSessionHistory(savedSessionId);
    }
  }, []);

  const loadSessionHistory = async (id: string) => {
    try {
      const response = await fetch(`${backendUrl}/api/chat/sessions/${id}`);
      if (response.ok) {
        const data = await response.json();
        // Convert API message format to UI message format
        const formattedMessages = data.messages.map((msg: any) => ({
          role: msg.role,
          content: msg.content,
          sources: msg.sources
        }));
        setMessages(formattedMessages);
      }
    } catch (error) {
      console.error('Failed to load session history:', error);
    }
  };

  const handleSendMessage = async (text: string) => {
    // Add user message to UI immediately
    const userMsg: Message = { role: 'user', content: text };
    setMessages(prev => [...prev, userMsg]);
    setIsLoading(true);

    try {
      const response = await fetch(`${backendUrl}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: text,
          session_id: sessionId,
          top_k: 3
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
        console.error('API Error:', errorData);
        throw new Error(errorData.detail || 'API request failed');
      }

      const data = await response.json();

      // Save session ID if it's new
      if (data.session_id && data.session_id !== sessionId) {
        setSessionId(data.session_id);
        localStorage.setItem('chat_session_id', data.session_id);
      }

      // Add bot response to UI
      const botMsg: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources
      };
      setMessages(prev => [...prev, botMsg]);
    } catch (error) {
      console.error('Chatbot error:', error);
      const errorMessage = error instanceof Error ? error.message : 'Unknown error occurred';
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: `I apologize, but I encountered an error while processing your question: ${errorMessage}\n\nPlease try again or rephrase your question.`
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.widgetContainer}>
      {isOpen ? (
        <ChatWindow
          onClose={() => setIsOpen(false)}
          messages={messages}
          onSendMessage={handleSendMessage}
          isLoading={isLoading}
        />
      ) : (
        <button className={styles.toggleButton} onClick={() => setIsOpen(true)}>
          <svg width="30" height="30" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;
