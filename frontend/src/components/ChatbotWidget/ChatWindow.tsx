import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapter_id: string;
    section_title: string;
  }>;
}

interface ChatWindowProps {
  onClose: () => void;
  messages: Message[];
  onSendMessage: (message: string) => void;
  isLoading: boolean;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ onClose, messages, onSendMessage, isLoading }) => {
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue.trim());
      setInputValue('');
    }
  };

  return (
    <div className={styles.chatWindow}>
      <div className={styles.header}>
        <h3>Physical AI Assistant</h3>
        <button className={styles.closeButton} onClick={onClose}>×</button>
      </div>

      <div className={styles.messageList}>
        {messages.length === 0 && (
          <div className={styles.botMessage + " " + styles.message}>
            Hello! I'm your Physical AI tutor. Ask me anything about ROS 2, Gazebo, NVIDIA Isaac, or VLA systems!
          </div>
        )}
        {messages.map((msg, idx) => (
          <div
            key={idx}
            className={`${styles.message} ${msg.role === 'user' ? styles.userMessage : styles.botMessage}`}
          >
            {msg.content}
            {msg.sources && msg.sources.length > 0 && (
              <div className={styles.sourceLinks}>
                <strong>Sources:</strong>
                {msg.sources.map((src, sIdx) => (
                  <span key={sIdx} className={styles.sourceItem}>
                    [{src.section_title}]
                  </span>
                ))}
              </div>
            )}
          </div>
        ))}
        {isLoading && (
          <div className={`${styles.message} ${styles.botMessage} ${styles.loading}`}>
            Thinking...
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form className={styles.inputArea} onSubmit={handleSubmit}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question..."
          disabled={isLoading}
        />
        <button type="submit" className={styles.sendButton} disabled={isLoading || !inputValue.trim()}>
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatWindow;
