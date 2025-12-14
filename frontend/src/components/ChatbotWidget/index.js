import React, { useState, useEffect, useRef } from 'react';
import './styles.css';

export default function ChatbotWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      text: "Hi! I'm C-3PO, your protocol droid assistant. How can I help you today?",
      isUser: false,
      time: new Date().toLocaleTimeString('en-US', { hour: 'numeric', minute: '2-digit', hour12: true }),
      quickReplies: [
        'What can you do?',
        'Tell me about Physical AI',
        'Explain ROS 2'
      ]
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const [showNotification, setShowNotification] = useState(true);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isTyping]);

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setShowNotification(false);
      setTimeout(() => inputRef.current?.focus(), 300);
    }
  };

  const formatTime = () => {
    return new Date().toLocaleTimeString('en-US', {
      hour: 'numeric',
      minute: '2-digit',
      hour12: true
    });
  };

  const addMessage = (text, isUser = false, quickReplies = null) => {
    const newMessage = {
      text,
      isUser,
      time: formatTime(),
      quickReplies
    };
    setMessages(prev => [...prev, newMessage]);
  };

  const sendMessage = async (text = inputValue) => {
    const messageText = text.trim();
    if (!messageText) return;

    // Add user message
    addMessage(messageText, true);
    setInputValue('');
    setIsTyping(true);

    try {
      // Call your backend API
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ text: messageText }),
      });

      if (!response.ok) {
        throw new Error('Network response was not ok');
      }

      const data = await response.json();

      // Simulate typing delay for better UX
      setTimeout(() => {
        setIsTyping(false);
        addMessage(data.response || data.message || 'I received your message!');
      }, 500);
    } catch (error) {
      console.error('Error sending message:', error);
      setTimeout(() => {
        setIsTyping(false);
        addMessage('Sorry, I encountered an error. Please make sure the backend is running on http://localhost:8000');
      }, 500);
    }
  };

  const handleQuickReply = (text) => {
    sendMessage(text);
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`chatbot-float-button ${isOpen ? 'active' : ''}`}
        onClick={toggleChatbot}
        aria-label="Toggle chatbot"
      >
        <svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
          <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z"/>
        </svg>
        {showNotification && !isOpen && (
          <span className="chatbot-notification-badge">1</span>
        )}
      </button>

      {/* Chatbot Sidebar */}
      <div className={`chatbot-sidebar ${isOpen ? 'open' : ''}`}>
        {/* Header */}
        <div className="chatbot-header">
          <div className="chatbot-bot-avatar">
            🤖
            <div className="chatbot-online-indicator"></div>
          </div>
          <div className="chatbot-bot-info">
            <div className="chatbot-bot-name">C-3PO Assistant</div>
            <div className="chatbot-bot-status">Online • Typically replies instantly</div>
          </div>
          <button className="chatbot-close-button" onClick={toggleChatbot} aria-label="Close chatbot">
            <svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
              <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/>
            </svg>
          </button>
        </div>

        {/* Messages Container */}
        <div className="chatbot-messages-container">
          {messages.map((message, index) => (
            <div key={index} className={`chatbot-message ${message.isUser ? 'user' : 'bot'}`}>
              <div className="chatbot-message-avatar">
                {message.isUser ? '👤' : '🤖'}
              </div>
              <div className="chatbot-message-content">
                <div className="chatbot-message-bubble">{message.text}</div>
                {message.quickReplies && (
                  <div className="chatbot-quick-replies">
                    {message.quickReplies.map((reply, idx) => (
                      <button
                        key={idx}
                        className="chatbot-quick-reply"
                        onClick={() => handleQuickReply(reply)}
                      >
                        {reply}
                      </button>
                    ))}
                  </div>
                )}
                <div className="chatbot-message-time">{message.time}</div>
              </div>
            </div>
          ))}

          {/* Typing Indicator */}
          {isTyping && (
            <div className="chatbot-typing-indicator">
              <div className="chatbot-message-avatar">🤖</div>
              <div className="chatbot-typing-dots">
                <div className="chatbot-typing-dot"></div>
                <div className="chatbot-typing-dot"></div>
                <div className="chatbot-typing-dot"></div>
              </div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        {/* Input Area */}
        <div className="chatbot-input-area">
          <input
            ref={inputRef}
            type="text"
            className="chatbot-message-input"
            placeholder="Type your message..."
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
          />
          <button
            className="chatbot-send-button"
            onClick={() => sendMessage()}
            disabled={!inputValue.trim()}
            aria-label="Send message"
          >
            <svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
              <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z"/>
            </svg>
          </button>
        </div>
      </div>
    </>
  );
}
