import React, { useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './Translator.module.css';

const Translator = ({ textToTranslate = "This is a placeholder text to be translated." }) => {
  const { siteConfig } = useDocusaurusContext();
  const { BACKEND_API_URL } = siteConfig.customFields;

  const [targetLanguage, setTargetLanguage] = useState('ur'); // Default to Urdu
  const [translatedText, setTranslatedText] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const languages = [
    { code: 'ur', name: 'Urdu' },
    { code: 'es', name: 'Spanish' },
    { code: 'fr', name: 'French' },
    // Add more languages as needed
  ];

  const handleTranslate = async () => {
    setLoading(true);
    setError(null);
    setTranslatedText('');

    try {
      const response = await fetch(`${BACKEND_API_URL}/translation/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Assuming no authentication is needed for simple translation,
          // otherwise, you would include an Authorization header
        },
        body: JSON.stringify({
          content: textToTranslate,
          target_language: targetLanguage,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslatedText(data.translated_content);
    } catch (err) {
      setError(err.message);
      console.error('Translation error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <h3>Translate Content</h3>
      <div className={styles.controls}>
        <label htmlFor="language-select" className={styles.label}>Translate to:</label>
        <select
          id="language-select"
          value={targetLanguage}
          onChange={(e) => setTargetLanguage(e.target.value)}
          className={styles.select}
        >
          {languages.map((lang) => (
            <option key={lang.code} value={lang.code}>
              {lang.name}
            </option>
          ))}
        </select>
        <button onClick={handleTranslate} disabled={loading} className={styles.button}>
          {loading ? 'Translating...' : 'Translate'}
        </button>
      </div>

      {error && <p className={styles.error}>Error: {error}</p>}

      {translatedText && (
        <div className={styles.translatedOutput}>
          <h4>Translated Text ({targetLanguage.toUpperCase()}):</h4>
          <p>{translatedText}</p>
        </div>
      )}
    </div>
  );
};

export default Translator;
