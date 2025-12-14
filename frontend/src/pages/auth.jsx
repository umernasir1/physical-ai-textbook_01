import React, { useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

function AuthPageContent() {
  // Dynamically import ModernAuth component to avoid SSR issues
  const [ModernAuth, setModernAuth] = React.useState(null);

  useEffect(() => {
    // Load ModernAuth component only on client side
    import('../components/ModernAuth').then((module) => {
      setModernAuth(() => module.default);
    });
  }, []);

  if (!ModernAuth) {
    return (
      <div style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '100vh',
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        color: 'white',
        fontSize: '1.2rem',
        fontWeight: '600'
      }}>
        Loading authentication...
      </div>
    );
  }

  return <ModernAuth />;
}

function AuthPage() {
  return (
    <BrowserOnly fallback={
      <div style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '100vh',
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        color: 'white',
        fontSize: '1.2rem',
        fontWeight: '600'
      }}>
        Loading...
      </div>
    }>
      {() => <AuthPageContent />}
    </BrowserOnly>
  );
}

export default AuthPage;
