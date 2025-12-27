import { betterAuth } from "better-auth";
import { postgresAdapter } from "@better-auth/postgres-adapter";
import { neon } from "drizzle-orm/neon-http";

// Initialize your database connection
// Note: You'll need to install drizzle-orm and @neondatabase/serverless if using Neon
// npm install drizzle-orm @neondatabase/serverless

const db = neon(process.env.NEON_DATABASE_URL!);

export const auth = betterAuth({
  database: postgresAdapter(db, {
    provider: "pg", // or "mysql" if using MySQL
  }),
  secret: process.env.BETTER_AUTH_SECRET || "fallback-secret-for-development",
  // Define custom fields for user background
  user: {
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: false,
      },
      hardwareBackground: {
        type: "string", 
        required: false,
      }
    }
  },
  socialProviders: {
    // Add social providers if needed
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
  }
});