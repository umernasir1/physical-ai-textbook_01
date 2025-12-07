import React, { createContext, useContext, useState, useEffect } from 'react';

const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState(null); // Could store user object from backend
  const [loading, setLoading] = useState(true);

  // Function to check token validity and fetch user info
  const checkAuthStatus = async () => {
    const token = localStorage.getItem('access_token');
    if (token) {
      // In a real app, you would verify this token with your backend
      // and fetch user details. For now, we'll just assume it's valid if present.
      setIsAuthenticated(true);
      setUser({ email: 'user@example.com' }); // Placeholder user
    } else {
      setIsAuthenticated(false);
      setUser(null);
    }
    setLoading(false);
  };

  useEffect(() => {
    checkAuthStatus();
  }, []);

  const login = (token, userData) => {
    localStorage.setItem('access_token', token);
    setIsAuthenticated(true);
    setUser(userData);
  };

  const logout = () => {
    localStorage.removeItem('access_token');
    setIsAuthenticated(false);
    setUser(null);
  };

  // Add a function to update user context after signup
  const signupSuccess = (token, userData) => {
    localStorage.setItem('access_token', token);
    setIsAuthenticated(true);
    setUser(userData);
  };

  const contextValue = {
    isAuthenticated,
    user,
    loading,
    login,
    logout,
    signupSuccess,
  };

  if (loading) {
    return <div>Loading authentication...</div>; // Or a spinner
  }

  return (
    <AuthContext.Provider value={contextValue}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
