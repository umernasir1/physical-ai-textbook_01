import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

function AuthPageContent() {
  const [isLogin, setIsLogin] = useState(true);
  // Dynamically import auth components to avoid SSR issues
  const [authComponents, setAuthComponents] = React.useState(null);

  useEffect(() => {
    // Load auth components only on client side
    import('../components/Auth').then((module) => {
      setAuthComponents(module);
    });
  }, []);

  const switchToSignup = () => setIsLogin(false);
  const switchToLogin = () => setIsLogin(true);

  if (!authComponents) {
    return <div style={{ padding: '2rem', textAlign: 'center' }}>Loading authentication...</div>;
  }

  const { Login, Signup } = authComponents;

  return (
    <main>
      {isLogin ? (
        <Login switchToSignup={switchToSignup} />
      ) : (
        <Signup switchToLogin={switchToLogin} />
      )}
    </main>
  );
}

function AuthPage() {
  return (
    <Layout
      title="Authentication"
      description="Login or Sign up to the Physical AI Textbook"
    >
      <BrowserOnly fallback={<div style={{ padding: '2rem', textAlign: 'center' }}>Loading...</div>}>
        {() => <AuthPageContent />}
      </BrowserOnly>
    </Layout>
  );
}

export default AuthPage;
