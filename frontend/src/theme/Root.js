import React from 'react';
import ChatbotWidget from '@site/src/components/ChatbotWidget';
import { AuthProvider } from '@site/src/components/AuthProvider';
import { TranslationProvider } from '@site/src/components/TranslationProvider';

// This component wraps the entire application
// Provides authentication and translation context to all pages
export default function Root({ children }) {
  return (
    <AuthProvider>
      <TranslationProvider>
        {children}
        <ChatbotWidget />
      </TranslationProvider>
    </AuthProvider>
  );
}
