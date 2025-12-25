# # Use official Python runtime as a parent image
# FROM python:3.9-slim

# # Set environment variables
# ENV PYTHONUNBUFFERED=1 \
#     PYTHONDONTWRITEBYTECODE=1 \
#     PIP_NO_CACHE_DIR=1 \
#     PIP_DISABLE_PIP_VERSION_CHECK=1 \
#     GRADIO_SERVER_NAME=0.0.0.0 \
#     GRADIO_SERVER_PORT=7860 \
#     PORT=7860

# # Set work directory
# WORKDIR /app

# # Install system dependencies
# RUN apt-get update \
#     && apt-get install -y --no-install-recommends \
#         build-essential \
#         curl \
#         git \
#     && rm -rf /var/lib/apt/lists/*

# # Copy requirements file
# COPY requirements.txt .

# # Install Python dependencies
# RUN pip install --no-cache-dir --upgrade pip && pip install --no-cache-dir -r requirements.txt

# # Copy project
# COPY . .

# # Expose port (Hugging Face Spaces uses port 7860 for Gradio apps)
# EXPOSE 7860

# # Run the application
# CMD ["python", "space.py"]


FROM python:3.9-slim

WORKDIR /app

COPY backend/rag_chatbot_api/ .

RUN pip install --no-cache-dir -r requirements.txt

EXPOSE 8000

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
