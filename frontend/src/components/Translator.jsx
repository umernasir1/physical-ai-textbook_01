import React, { useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

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
          text: textToTranslate,
          target_language: targetLanguage,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslatedText(data.translated_text);
    } catch (err) {
      setError(err.message);
      console.error('Translation error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={styles.container}>
      <h3>Translate Content</h3>
      <div style={styles.controls}>
        <label htmlFor="language-select" style={styles.label}>Translate to:</label>
        <select
          id="language-select"
          value={targetLanguage}
          onChange={(e) => setTargetLanguage(e.target.value)}
          style={styles.select}
        >
          {languages.map((lang) => (
            <option key={lang.code} value={lang.code}>
              {lang.name}
            </option>
          ))}
        </select>
        <button onClick={handleTranslate} disabled={loading} style={styles.button}>
          {loading ? 'Translating...' : 'Translate'}
        </button>
      </div>

      {error && <p style={styles.error}>Error: {error}</p>}

      {translatedText && (
        <div style={styles.translatedOutput}>
          <h4>Translated Text ({targetLanguage.toUpperCase()}):</h4>
          <p>{translatedText}</p>
        </div>
      )}
    </div>
  );
};

const styles = {
  container: {
    padding: '15px',
    border: '1px solid #ddd',
    borderRadius: '8px',
    marginBottom: '20px',
    backgroundColor: '#f9f9f9',
  },
  controls: {
    display: 'flex',
    alignItems: 'center',
    gap: '10px',
    marginBottom: '15px',
  },
  label: {
    fontWeight: 'bold',
  },
  select: {
    padding: '8px',
    borderRadius: '4px',
    border: '1px solid #ccc',
  },
  button: {
    backgroundColor: '#28a745',
    color: 'white',
    padding: '8px 15px',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    fontSize: '14px',
  },
  error: {
    color: 'red',
    marginTop: '10px',
  },
  translatedOutput: {
    marginTop: '20px',
    padding: '10px',
    borderTop: '1px solid #eee',
    backgroundColor: '#e9ffe9',
    borderRadius: '4px',
  },
};

export default Translator;
