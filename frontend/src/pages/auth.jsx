import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { Login, Signup } from '../components/Auth';
import { useAuth } from '../components/AuthProvider';
import { useHistory } from '@docusaurus/router';

function AuthPage() {
  const [isLogin, setIsLogin] = useState(true);
  const { isAuthenticated } = useAuth();
  const history = useHistory();

  useEffect(() => {
    if (isAuthenticated) {
      history.push('/'); // Redirect authenticated users to home page
    }
  }, [isAuthenticated, history]);

  if (isAuthenticated) {
    return null; // Don't render anything if authenticated, wait for redirect
  }

  const switchToSignup = () => setIsLogin(false);
  const switchToLogin = () => setIsLogin(true);

  return (
    <Layout
      title="Authentication"
      description="Login or Sign up to the Physical AI Textbook"
    >
      <main>
        {isLogin ? (
          <Login switchToSignup={switchToSignup} />
        ) : (
          <Signup switchToLogin={switchToLogin} />
        )}
      </main>
    </Layout>
  );
}

export default AuthPage;
