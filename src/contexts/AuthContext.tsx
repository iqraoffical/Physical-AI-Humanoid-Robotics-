import React, { createContext, useContext, ReactNode, useState, useEffect } from 'react';
import { useSession } from '../auth-client';

interface AuthContextType {
  user: any;
  isLoading: boolean;
  signIn: () => void;
  signOut: () => void;
  updateUserBackground: (softwareBg: string, hardwareBg: string) => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const { data: session, isLoading } = useSession();
  const [user, setUser] = useState<any>(null);

  useEffect(() => {
    if (session) {
      setUser(session.user);
    } else {
      setUser(null);
    }
  }, [session]);

  const signIn = async () => {
    // Redirect to sign in page
    window.location.href = '/auth/signin';
  };

  const signOut = async () => {
    // Sign out and redirect to home
    window.location.href = '/api/auth/sign-out';
  };

  const updateUserBackground = async (softwareBg: string, hardwareBg: string) => {
    if (!user) return;
    
    try {
      const response = await fetch('/api/user-background', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          userId: user.id,
          softwareBackground: softwareBg,
          hardwareBackground: hardwareBg,
        }),
      });

      if (response.ok) {
        // Update local user state
        setUser({
          ...user,
          softwareBackground: softwareBg,
          hardwareBackground: hardwareBg,
        });
      }
    } catch (error) {
      console.error('Error updating user background:', error);
    }
  };

  return (
    <AuthContext.Provider value={{ user, isLoading, signIn, signOut, updateUserBackground }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}