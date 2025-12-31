from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.api import (
    query_endpoint,
    selected_text_endpoint,
    health_endpoint,
    auth_endpoint,
    personalization_endpoint,
)
from app.middleware.logging_middleware import setup_logging, LoggingMiddleware


def create_app() -> FastAPI:
    app = FastAPI(
        title="RAG Chatbot API for AI Textbook",
        description="An API that answers questions using only content from an AI textbook",
        version="1.0.0",
        openapi_url=f"{settings.API_PREFIX}/openapi.json",
        docs_url=f"{settings.API_PREFIX}/docs",
        redoc_url=f"{settings.API_PREFIX}/redoc",
    )

    # Logging middleware
    app.add_middleware(LoggingMiddleware)

    # ✅ FIXED CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://localhost:3000",
            "http://127.0.0.1:3000",
        ],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # API routes
    app.include_router(query_endpoint.router, prefix=settings.API_PREFIX, tags=["query"])
    app.include_router(selected_text_endpoint.router, prefix=settings.API_PREFIX, tags=["selected-text"])
    app.include_router(health_endpoint.router, prefix=settings.API_PREFIX, tags=["health"])
    app.include_router(auth_endpoint.router, prefix=settings.API_PREFIX, tags=["auth"])
    app.include_router(personalization_endpoint.router, prefix=settings.API_PREFIX, tags=["personalization"])

    # Root endpoint
    @app.get("/")
    def root():
        return {
            "message": "RAG Chatbot API is running",
            "docs": f"{settings.API_PREFIX}/docs",
            "query_endpoint": f"{settings.API_PREFIX}/query"
        }

    return app


app = create_app()


@app.on_event("startup")
async def startup_event():
    setup_logging()
    print("Starting up RAG Chatbot API...")


@app.on_event("shutdown")
async def shutdown_event():
    print("Shutting down RAG Chatbot API...")
