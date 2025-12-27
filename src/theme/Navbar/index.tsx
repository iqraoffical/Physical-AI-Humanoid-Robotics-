import React from 'react';
import Navbar from '@theme-original/Navbar';
import { useAuth } from '@site/src/contexts/AuthContext';
import Link from '@docusaurus/Link';

export default function NavbarWrapper(props) {
  const { user, isLoading, signIn, signOut } = useAuth();

  return (
    <>
      <Navbar {...props} />
      {!isLoading && (
        <div style={{
          position: 'absolute',
          right: '1rem',
          top: '0.5rem',
          display: 'flex',
          alignItems: 'center',
          gap: '1rem'
        }}>
          {user ? (
            <>
              <span style={{ color: '#fff' }}>Welcome, {user.email}</span>
              <button 
                onClick={signOut}
                style={{
                  padding: '0.25rem 0.75rem',
                  backgroundColor: '#e74c3c',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer'
                }}
              >
                Sign Out
              </button>
            </>
          ) : (
            <Link
              to="/auth/signin"
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: '#3498db',
                color: 'white',
                textDecoration: 'none',
                borderRadius: '4px'
              }}
            >
              Sign In
            </Link>
          )}
        </div>
      )}
    </>
  );
}