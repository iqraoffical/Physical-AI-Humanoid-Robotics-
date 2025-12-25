#!/usr/bin/env python3
"""
Backend startup script for the RAG Chatbot API
This script starts the FastAPI server with uvicorn
"""

import subprocess
import sys
import os
from pathlib import Path

def main():
    print("Starting RAG Chatbot API server...")
    print(f"Current working directory: {os.getcwd()}")

    # Run uvicorn with the correct module path
    cmd = [
        sys.executable, "-m", "uvicorn",
        "app.main:app",
        "--reload",
        "--host", "127.0.0.1",
        "--port", "8000"
    ]

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error starting server: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nServer stopped by user.")
        sys.exit(0)

if __name__ == "__main__":
    main()