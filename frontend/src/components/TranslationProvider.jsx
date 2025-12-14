import React, { createContext, useContext, useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const TranslationContext = createContext();

export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    throw new Error('useTranslation must be used within TranslationProvider');
  }
  return context;
};

export const TranslationProvider = ({ children }) => {
  const { siteConfig } = useDocusaurusContext();
  const { BACKEND_API_URL } = siteConfig.customFields;

  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [currentLanguage, setCurrentLanguage] = useState('en');
  const [targetLanguage, setTargetLanguage] = useState('ur');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const languages = [
    { code: 'en', name: 'English' },
    { code: 'ur', name: 'Urdu' },
    { code: 'es', name: 'Spanish' },
    { code: 'fr', name: 'French' },
    { code: 'de', name: 'German' },
    { code: 'ar', name: 'Arabic' },
    { code: 'zh', name: 'Chinese' },
    { code: 'ja', name: 'Japanese' },
  ];

  const translateContent = async (content) => {
    if (!content || content.trim() === '') {
      setError('No content to translate');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await fetch(`${BACKEND_API_URL}/translation/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: content,
          target_language: targetLanguage,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
      setIsTranslated(true);
      setCurrentLanguage(targetLanguage);
    } catch (err) {
      setError(err.message);
      console.error('Translation error:', err);
    } finally {
      setLoading(false);
    }
  };

  const resetTranslation = () => {
    setIsTranslated(false);
    setTranslatedContent('');
    setCurrentLanguage('en');
    setError(null);
  };

  const value = {
    isTranslated,
    translatedContent,
    currentLanguage,
    targetLanguage,
    setTargetLanguage,
    loading,
    error,
    languages,
    translateContent,
    resetTranslation,
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};
