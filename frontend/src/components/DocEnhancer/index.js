import React, { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './styles.css';

export default function DocEnhancer() {
  const {siteConfig} = useDocusaurusContext();
  const API_BASE_URL = siteConfig.customFields?.BACKEND_API_URL || 'http://localhost:8000/v1';

  const [showPanel, setShowPanel] = useState(true);
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationLanguage, setTranslationLanguage] = useState('ur'); // Urdu
  const [skillLevel, setSkillLevel] = useState('intermediate');
  const [originalContent, setOriginalContent] = useState(null);

  // Save original content on mount
  useEffect(() => {
    const contentElement = document.querySelector('.markdown');
    if (contentElement && !originalContent) {
      setOriginalContent(contentElement.innerHTML);
    }
  }, []);

  const handlePersonalize = async () => {
    try {
      setIsPersonalizing(true);

      const contentElement = document.querySelector('.markdown');
      if (!contentElement) {
        alert('No content found to personalize');
        return;
      }

      const currentContent = originalContent || contentElement.innerHTML;
      const textContent = contentElement.innerText;

      const response = await fetch(`${API_BASE_URL}/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('access_token') || ''}`
        },
        body: JSON.stringify({
          content: textContent.substring(0, 5000), // Limit to 5000 chars
          skill_level: skillLevel
        })
      });

      if (!response.ok) {
        throw new Error('Personalization failed');
      }

      const data = await response.json();

      // Create personalized content HTML
      const personalizedHTML = `
        <div class="personalized-content-notice">
          <strong>📊 Personalized Content (${skillLevel.toUpperCase()} level)</strong>
          <button class="restore-btn" onclick="window.location.reload()">Restore Original</button>
        </div>
        <div class="personalized-text">
          ${data.personalized_content.replace(/\n/g, '<br/>')}
        </div>
      `;

      contentElement.innerHTML = personalizedHTML;

    } catch (error) {
      console.error('Personalization error:', error);
      const errorMessage = error.message || String(error) || 'Unknown error occurred';
      alert(`Failed to personalize content: ${errorMessage}`);
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleTranslate = async () => {
    try {
      setIsTranslating(true);

      const contentElement = document.querySelector('.markdown');
      if (!contentElement) {
        alert('No content found to translate');
        return;
      }

      // Save original if not already saved
      if (!originalContent) {
        setOriginalContent(contentElement.innerHTML);
      }

      // Get all text content from the page
      const textContent = contentElement.innerText;

      // Call translation API
      const response = await fetch(`${API_BASE_URL}/translation/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: textContent,
          target_language: translationLanguage
        })
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      const translatedText = data.translated_content;

      // Apply RTL if needed
      const isRTL = translationLanguage === 'ur' || translationLanguage === 'ar';
      if (isRTL) {
        contentElement.setAttribute('dir', 'rtl');
        contentElement.style.textAlign = 'right';
      }

      // Add a small indicator at the very top
      const indicator = document.createElement('div');
      indicator.style.cssText = 'background: #667eea; color: white; padding: 8px 12px; border-radius: 4px; margin-bottom: 15px; display: inline-flex; gap: 8px; align-items: center; font-size: 13px;';
      indicator.innerHTML = `🌐 ${translationLanguage.toUpperCase()} <button onclick="window.location.reload()" style="background: white; color: #667eea; border: none; padding: 3px 8px; border-radius: 3px; cursor: pointer; font-weight: 600; font-size: 11px; margin-left: 8px;">↺ Restore</button>`;

      // Walk through the DOM and replace all text nodes with translated content
      const replaceTextInElement = (element) => {
        const walker = document.createTreeWalker(
          element,
          NodeFilter.SHOW_TEXT,
          {
            acceptNode: function(node) {
              // Only accept text nodes that have actual content
              if (node.textContent.trim().length > 0) {
                return NodeFilter.FILTER_ACCEPT;
              }
              return NodeFilter.FILTER_REJECT;
            }
          },
          false
        );

        const textNodes = [];
        let node;
        while (node = walker.nextNode()) {
          textNodes.push(node);
        }

        // Replace the first text node with all translated content
        // Clear the rest to avoid duplication
        if (textNodes.length > 0) {
          textNodes[0].textContent = translatedText;
          for (let i = 1; i < textNodes.length; i++) {
            textNodes[i].textContent = '';
          }
        }
      };

      // Replace text content
      replaceTextInElement(contentElement);

      // Insert indicator at the beginning
      contentElement.insertBefore(indicator, contentElement.firstChild);

    } catch (error) {
      console.error('Translation error:', error);
      const errorMessage = error.message || String(error) || 'Unknown error occurred';
      alert(`Failed to translate content: ${errorMessage}`);
    } finally {
      setIsTranslating(false);
    }
  };

  const restoreOriginal = () => {
    const contentElement = document.querySelector('.markdown');
    if (contentElement && originalContent) {
      contentElement.innerHTML = originalContent;
    }
  };

  if (!showPanel) {
    return (
      <button
        className="doc-enhancer-toggle collapsed"
        onClick={() => setShowPanel(true)}
        title="Show Enhancement Options"
      >
        ⚙️
      </button>
    );
  }

  return (
    <div className="doc-enhancer-panel">
      <div className="panel-header">
        <h3>📚 Content Enhancement</h3>
        <button
          className="close-btn"
          onClick={() => setShowPanel(false)}
          title="Hide Panel"
        >
          ✕
        </button>
      </div>

      <div className="panel-section">
        <h4>🎯 Personalize Content</h4>
        <p className="section-desc">Adapt content to your skill level</p>
        <select
          value={skillLevel}
          onChange={(e) => setSkillLevel(e.target.value)}
          className="skill-select"
        >
          <option value="beginner">Beginner - Basic concepts</option>
          <option value="intermediate">Intermediate - Standard</option>
          <option value="advanced">Advanced - In-depth</option>
        </select>
        <button
          className="action-btn personalize-btn"
          onClick={handlePersonalize}
          disabled={isPersonalizing}
        >
          {isPersonalizing ? (
            <>
              <span className="spinner-small"></span>
              Personalizing...
            </>
          ) : (
            <>
              <span>🎨</span>
              Personalize Content
            </>
          )}
        </button>
      </div>

      <div className="panel-section">
        <h4>🌐 Translate Content</h4>
        <p className="section-desc">Read in your preferred language</p>
        <select
          value={translationLanguage}
          onChange={(e) => setTranslationLanguage(e.target.value)}
          className="language-select"
        >
          <option value="ur">اردو (Urdu)</option>
          <option value="hi">हिंदी (Hindi)</option>
          <option value="es">Español (Spanish)</option>
          <option value="fr">Français (French)</option>
          <option value="ar">العربية (Arabic)</option>
          <option value="zh">中文 (Chinese)</option>
        </select>
        <button
          className="action-btn translate-btn"
          onClick={handleTranslate}
          disabled={isTranslating}
        >
          {isTranslating ? (
            <>
              <span className="spinner-small"></span>
              Translating...
            </>
          ) : (
            <>
              <span>🔤</span>
              Translate to {translationLanguage === 'ur' ? 'Urdu' : translationLanguage.toUpperCase()}
            </>
          )}
        </button>
      </div>

      <div className="panel-footer">
        <button
          className="restore-original-btn"
          onClick={restoreOriginal}
        >
          ↺ Restore Original
        </button>
      </div>
    </div>
  );
}
