import { createAuthClient } from "better-auth/client";
import { useEffect, useState } from "react";

// Initialize client only in browser environment
let client: any = null;

if (typeof window !== 'undefined') {
  client = createAuthClient({
    baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3000", // Update with your actual URL
  });
}

export const signIn = client ? client.signIn : null;
export const signOut = client ? client.signOut : null;

// Custom hook to handle session with SSR compatibility
export const useSession = () => {
  const [data, setData] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    if (client) {
      client.useSession().then((result: any) => {
        setData(result.data);
        setIsLoading(result.isLoading);
      });
    } else {
      setIsLoading(false);
    }
  }, []);

  return { data, isLoading };
};