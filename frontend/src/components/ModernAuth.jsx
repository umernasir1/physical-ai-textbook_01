import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from './AuthProvider';
import './ModernAuth.css';

const useApiBaseUrl = () => {
  const { siteConfig } = useDocusaurusContext();
  return siteConfig.customFields?.BACKEND_API_URL || 'http://localhost:8000/api/v1';
};

const ModernAuth = () => {
  const [isLogin, setIsLogin] = useState(true);
  const [formData, setFormData] = useState({
    fullName: '',
    email: '',
    password: '',
    confirmPassword: '',
    rememberMe: false,
    acceptTerms: false
  });
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [errors, setErrors] = useState({});
  const [loading, setLoading] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState(0);
  const [darkMode, setDarkMode] = useState(false);

  const history = useHistory();
  const API_BASE_URL = useApiBaseUrl();
  const auth = useAuth();

  // Password strength calculator
  const calculatePasswordStrength = (password) => {
    let strength = 0;
    if (password.length >= 8) strength += 25;
    if (password.match(/[a-z]/) && password.match(/[A-Z]/)) strength += 25;
    if (password.match(/\d/)) strength += 25;
    if (password.match(/[^a-zA-Z\d]/)) strength += 25;
    return strength;
  };

  // Validate email
  const validateEmail = (email) => {
    const re = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return re.test(email);
  };

  // Handle input changes
  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    const newValue = type === 'checkbox' ? checked : value;

    setFormData(prev => ({
      ...prev,
      [name]: newValue
    }));

    // Clear error for this field
    setErrors(prev => ({
      ...prev,
      [name]: ''
    }));

    // Update password strength
    if (name === 'password') {
      setPasswordStrength(calculatePasswordStrength(value));
    }
  };

  // Validate form
  const validateForm = () => {
    const newErrors = {};

    if (!isLogin && !formData.fullName.trim()) {
      newErrors.fullName = 'Full name is required';
    }

    if (!formData.email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!validateEmail(formData.email)) {
      newErrors.email = 'Invalid email format';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    }

    if (!isLogin) {
      if (!formData.confirmPassword) {
        newErrors.confirmPassword = 'Please confirm your password';
      } else if (formData.password !== formData.confirmPassword) {
        newErrors.confirmPassword = 'Passwords do not match';
      }

      if (!formData.acceptTerms) {
        newErrors.acceptTerms = 'You must accept the terms and conditions';
      }
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!validateForm()) {
      // Shake animation on error
      document.querySelector('.auth-form').classList.add('shake');
      setTimeout(() => {
        document.querySelector('.auth-form').classList.remove('shake');
      }, 500);
      return;
    }

    setLoading(true);

    try {
      if (isLogin) {
        // Login
        const response = await fetch(`${API_BASE_URL}/auth/token`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
          },
          body: new URLSearchParams({
            username: formData.email,
            password: formData.password,
          }).toString(),
        });

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.detail || 'Login failed');
        }

        const data = await response.json();

        // Use AuthProvider's login method
        auth.login(data.access_token, { email: formData.email });

        // Success animation
        showToast('Login successful!', 'success');
        setTimeout(() => history.push('/physical-ai-textbook/'), 1500);
      } else {
        // Signup
        const response = await fetch(`${API_BASE_URL}/auth/signup`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email: formData.email,
            password: formData.password,
            full_name: formData.fullName
          }),
        });

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.detail || 'Signup failed');
        }

        const data = await response.json();

        // Use AuthProvider's signupSuccess method
        auth.signupSuccess(data.access_token, {
          email: formData.email,
          full_name: formData.fullName
        });

        // Success animation
        showToast('Account created successfully!', 'success');
        setTimeout(() => history.push('/physical-ai-textbook/'), 1500);
      }
    } catch (error) {
      console.error('Auth error:', error);
      showToast(error.message, 'error');
    } finally {
      setLoading(false);
    }
  };

  // Show toast notification
  const showToast = (message, type) => {
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.textContent = message;
    document.body.appendChild(toast);

    setTimeout(() => toast.classList.add('show'), 100);
    setTimeout(() => {
      toast.classList.remove('show');
      setTimeout(() => toast.remove(), 300);
    }, 3000);
  };

  // Toggle between login and signup
  const toggleMode = () => {
    setIsLogin(!isLogin);
    setFormData({
      fullName: '',
      email: '',
      password: '',
      confirmPassword: '',
      rememberMe: false,
      acceptTerms: false
    });
    setErrors({});
    setPasswordStrength(0);
  };

  // Social login handler
  const handleSocialLogin = (provider) => {
    showToast(`${provider} login coming soon!`, 'info');
  };

  return (
    <div className={`modern-auth-container ${darkMode ? 'dark-mode' : ''}`}>
      {/* Background Effects */}
      <div className="auth-background">
        <div className="gradient-orb orb-1"></div>
        <div className="gradient-orb orb-2"></div>
        <div className="gradient-orb orb-3"></div>
        <div className="grid-pattern"></div>
      </div>

      {/* Dark Mode Toggle */}
      <button
        className="theme-toggle"
        onClick={() => setDarkMode(!darkMode)}
        aria-label="Toggle dark mode"
      >
        {darkMode ? '☀️' : '🌙'}
      </button>

      {/* Main Content */}
      <div className="auth-content">
        {/* Left Side - Branding */}
        <div className="auth-left">
          <div className="brand-content">
            <h1 className="brand-title">
              Physical AI
              <span className="brand-subtitle">& Humanoid Robotics</span>
            </h1>
            <p className="brand-description">
              Join thousands of learners exploring the future of robotics and artificial intelligence
            </p>
            <div className="feature-list">
              <div className="feature-item">
                <span className="feature-icon">🤖</span>
                <span>AI-Powered Learning</span>
              </div>
              <div className="feature-item">
                <span className="feature-icon">📚</span>
                <span>Comprehensive Curriculum</span>
              </div>
              <div className="feature-item">
                <span className="feature-icon">⚡</span>
                <span>Hands-On Projects</span>
              </div>
            </div>
          </div>
        </div>

        {/* Right Side - Form */}
        <div className="auth-right">
          <div className={`auth-form-container ${isLogin ? 'login-mode' : 'signup-mode'}`}>
            <div className="auth-form">
              {/* Header */}
              <div className="form-header">
                <h2 className="form-title">
                  {isLogin ? 'Welcome Back' : 'Create Account'}
                </h2>
                <p className="form-subtitle">
                  {isLogin
                    ? 'Enter your credentials to access your account'
                    : 'Sign up to start your learning journey'}
                </p>
              </div>

              {/* Social Login */}
              <div className="social-login">
                <button
                  className="social-btn google"
                  onClick={() => handleSocialLogin('Google')}
                  type="button"
                >
                  <svg viewBox="0 0 24 24" width="20" height="20">
                    <path fill="#4285F4" d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"/>
                    <path fill="#34A853" d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"/>
                    <path fill="#FBBC05" d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"/>
                    <path fill="#EA4335" d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"/>
                  </svg>
                  Continue with Google
                </button>
              </div>

              <div className="divider">
                <span>or</span>
              </div>

              {/* Form Fields */}
              <form onSubmit={handleSubmit}>
                {!isLogin && (
                  <div className="form-group">
                    <label htmlFor="fullName">Full Name</label>
                    <div className="input-wrapper">
                      <input
                        type="text"
                        id="fullName"
                        name="fullName"
                        value={formData.fullName}
                        onChange={handleChange}
                        className={errors.fullName ? 'error' : ''}
                        placeholder="John Doe"
                      />
                      <span className="input-icon">👤</span>
                    </div>
                    {errors.fullName && <span className="error-message">{errors.fullName}</span>}
                  </div>
                )}

                <div className="form-group">
                  <label htmlFor="email">Email Address</label>
                  <div className="input-wrapper">
                    <input
                      type="email"
                      id="email"
                      name="email"
                      value={formData.email}
                      onChange={handleChange}
                      className={errors.email ? 'error' : ''}
                      placeholder="you@example.com"
                    />
                    <span className="input-icon">✉️</span>
                  </div>
                  {errors.email && <span className="error-message">{errors.email}</span>}
                </div>

                <div className="form-group">
                  <label htmlFor="password">Password</label>
                  <div className="input-wrapper">
                    <input
                      type={showPassword ? 'text' : 'password'}
                      id="password"
                      name="password"
                      value={formData.password}
                      onChange={handleChange}
                      className={errors.password ? 'error' : ''}
                      placeholder="Enter your password"
                    />
                    <button
                      type="button"
                      className="password-toggle"
                      onClick={() => setShowPassword(!showPassword)}
                    >
                      {showPassword ? '👁️' : '👁️‍🗨️'}
                    </button>
                  </div>
                  {errors.password && <span className="error-message">{errors.password}</span>}

                  {!isLogin && formData.password && (
                    <div className="password-strength">
                      <div className="strength-bar">
                        <div
                          className={`strength-fill strength-${passwordStrength}`}
                          style={{ width: `${passwordStrength}%` }}
                        ></div>
                      </div>
                      <span className="strength-text">
                        {passwordStrength < 50 ? 'Weak' : passwordStrength < 75 ? 'Medium' : 'Strong'}
                      </span>
                    </div>
                  )}
                </div>

                {!isLogin && (
                  <div className="form-group">
                    <label htmlFor="confirmPassword">Confirm Password</label>
                    <div className="input-wrapper">
                      <input
                        type={showConfirmPassword ? 'text' : 'password'}
                        id="confirmPassword"
                        name="confirmPassword"
                        value={formData.confirmPassword}
                        onChange={handleChange}
                        className={errors.confirmPassword ? 'error' : ''}
                        placeholder="Confirm your password"
                      />
                      <button
                        type="button"
                        className="password-toggle"
                        onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                      >
                        {showConfirmPassword ? '👁️' : '👁️‍🗨️'}
                      </button>
                    </div>
                    {errors.confirmPassword && <span className="error-message">{errors.confirmPassword}</span>}
                  </div>
                )}

                {isLogin ? (
                  <div className="form-options">
                    <label className="checkbox-label">
                      <input
                        type="checkbox"
                        name="rememberMe"
                        checked={formData.rememberMe}
                        onChange={handleChange}
                      />
                      <span className="checkbox-custom"></span>
                      Remember me
                    </label>
                    <a href="#" className="forgot-link">Forgot password?</a>
                  </div>
                ) : (
                  <div className="form-options">
                    <label className="checkbox-label">
                      <input
                        type="checkbox"
                        name="acceptTerms"
                        checked={formData.acceptTerms}
                        onChange={handleChange}
                      />
                      <span className="checkbox-custom"></span>
                      I accept the <a href="#">Terms & Conditions</a>
                    </label>
                    {errors.acceptTerms && <span className="error-message">{errors.acceptTerms}</span>}
                  </div>
                )}

                <button
                  type="submit"
                  className="submit-btn"
                  disabled={loading}
                >
                  {loading ? (
                    <span className="spinner"></span>
                  ) : (
                    <>
                      {isLogin ? 'Sign In' : 'Create Account'}
                      <span className="btn-arrow">→</span>
                    </>
                  )}
                </button>
              </form>

              {/* Toggle Mode */}
              <div className="form-footer">
                <p>
                  {isLogin ? "Don't have an account?" : "Already have an account?"}
                  {' '}
                  <button
                    onClick={toggleMode}
                    className="toggle-mode-btn"
                  >
                    {isLogin ? 'Sign up' : 'Sign in'}
                  </button>
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ModernAuth;
