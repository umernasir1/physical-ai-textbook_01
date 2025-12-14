import React, { useRef } from 'react';
import { useTranslation } from './TranslationProvider';
import styles from './InlineTranslator.module.css';

const InlineTranslator = ({ contentRef }) => {
  const {
    isTranslated,
    targetLanguage,
    setTargetLanguage,
    loading,
    error,
    languages,
    translateContent,
    resetTranslation,
  } = useTranslation();

  const handleTranslate = () => {
    if (!contentRef || !contentRef.current) {
      console.error('Content reference not available');
      return;
    }

    // Extract all text content from the page
    const contentElement = contentRef.current;
    const textContent = extractTextContent(contentElement);

    console.log('Extracted text length:', textContent?.length);
    console.log('First 500 chars:', textContent?.substring(0, 500));

    if (textContent && textContent.trim().length > 0) {
      translateContent(textContent);
    } else {
      console.error('No content extracted to translate');
    }
  };

  // Extract text content from DOM element, preserving structure
  const extractTextContent = (element) => {
    if (!element) return '';

    // Use innerText to get all visible text content
    // This preserves line breaks and gets all rendered text
    let text = element.innerText || element.textContent || '';

    // Clean up excessive whitespace while preserving structure
    text = text
      .split('\n')
      .map(line => line.trim())
      .filter(line => line.length > 0)
      .join('\n');

    return text;
  };

  return (
    <div className={styles.translatorBar}>
      <div className={styles.controls}>
        {!isTranslated ? (
          <>
            <label htmlFor="language-select" className={styles.label}>
              Translate to:
            </label>
            <select
              id="language-select"
              value={targetLanguage}
              onChange={(e) => setTargetLanguage(e.target.value)}
              className={styles.select}
              disabled={loading}
            >
              {languages.filter(lang => lang.code !== 'en').map((lang) => (
                <option key={lang.code} value={lang.code}>
                  {lang.name}
                </option>
              ))}
            </select>
            <button
              onClick={handleTranslate}
              disabled={loading}
              className={styles.translateButton}
            >
              {loading ? 'Translating...' : 'Translate Page'}
            </button>
          </>
        ) : (
          <>
            <span className={styles.translatedLabel}>
              Page translated to {languages.find(l => l.code === targetLanguage)?.name}
            </span>
            <button
              onClick={resetTranslation}
              className={styles.resetButton}
            >
              Show Original
            </button>
          </>
        )}
      </div>

      {error && (
        <div className={styles.error}>
          Translation error: {error}
        </div>
      )}
    </div>
  );
};

export default InlineTranslator;
