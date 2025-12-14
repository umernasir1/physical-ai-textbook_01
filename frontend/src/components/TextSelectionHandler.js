import React, { useEffect, useState } from 'react';
import './TextSelectionHandler.css';

export default function TextSelectionHandler({ onAskAboutText }) {
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 10) {  // Only show for selections longer than 10 characters
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(text);
        setButtonPosition({
          top: rect.top + window.scrollY - 50,
          left: rect.left + window.scrollX + (rect.width / 2)
        });
        setShowButton(true);
      } else {
        setShowButton(false);
      }
    };

    const handleClickOutside = (e) => {
      if (!e.target.closest('.text-selection-button')) {
        setShowButton(false);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const handleAskClick = () => {
    if (onAskAboutText && selectedText) {
      onAskAboutText(selectedText);
      setShowButton(false);
      window.getSelection().removeAllRanges();
    }
  };

  if (!showButton) return null;

  return (
    <button
      className="text-selection-button"
      style={{
        position: 'absolute',
        top: `${buttonPosition.top}px`,
        left: `${buttonPosition.left}px`,
        transform: 'translateX(-50%)',
        zIndex: 10000
      }}
      onClick={handleAskClick}
    >
      <span className="button-icon">💬</span>
      <span className="button-text">Ask about this</span>
    </button>
  );
}
