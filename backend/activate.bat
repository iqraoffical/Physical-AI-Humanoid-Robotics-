@echo off
REM Activate the virtual environment
call "C:\Users\iqra\Desktop\Hecathon_Book\backend\rag_chatbot_api\venv\Scripts\activate.bat"

REM Print a message to confirm activation
echo Virtual environment activated. You can now run your Python applications.

REM To run the FastAPI application, use:
echo To start the FastAPI app: uvicorn app.main:app --reload

REM Keep the command prompt open
cmd /k