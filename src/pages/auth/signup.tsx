import React, { useState } from 'react';
import { useNavigate } from '@docusaurus/router';
import { signIn } from '../auth-client';

export default function SignUpPage() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      // First, create the user with basic credentials
      const response = await fetch('/api/auth/sign-up', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.message || 'Sign up failed');
      }

      // Then update the user's background information
      const userResponse = await fetch('/api/user-background', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          softwareBackground,
          hardwareBackground,
        }),
      });

      if (!userResponse.ok) {
        throw new Error('Failed to save background information');
      }

      // Redirect to sign in or home page
      window.location.href = '/auth/signin';
    } catch (err: any) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
      <h1>Sign Up</h1>
      <form onSubmit={handleSubmit}>
        {error && <div style={{ color: 'red', marginBottom: '1rem' }}>{error}</div>}
        
        <div style={{ marginBottom: '1rem' }}>
          <label htmlFor="email" style={{ display: 'block', marginBottom: '0.5rem' }}>
            Email
          </label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
            }}
          />
        </div>
        
        <div style={{ marginBottom: '1rem' }}>
          <label htmlFor="password" style={{ display: 'block', marginBottom: '0.5rem' }}>
            Password
          </label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
            }}
          />
        </div>
        
        <div style={{ marginBottom: '1rem' }}>
          <label htmlFor="softwareBackground" style={{ display: 'block', marginBottom: '0.5rem' }}>
            Software Background
          </label>
          <textarea
            id="softwareBackground"
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value)}
            placeholder="Describe your software background (programming languages, frameworks, experience, etc.)"
            required
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              minHeight: '80px',
            }}
          />
        </div>
        
        <div style={{ marginBottom: '1rem' }}>
          <label htmlFor="hardwareBackground" style={{ display: 'block', marginBottom: '0.5rem' }}>
            Hardware Background
          </label>
          <textarea
            id="hardwareBackground"
            value={hardwareBackground}
            onChange={(e) => setHardwareBackground(e.target.value)}
            placeholder="Describe your hardware background (electronics, robotics, experience, etc.)"
            required
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              minHeight: '80px',
            }}
          />
        </div>
        
        <button
          type="submit"
          disabled={loading}
          style={{
            padding: '0.75rem 1.5rem',
            backgroundColor: '#007cba',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: loading ? 'not-allowed' : 'pointer',
          }}
        >
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
      
      <div style={{ marginTop: '1rem' }}>
        Already have an account? <a href="/auth/signin">Sign In</a>
      </div>
    </div>
  );
}