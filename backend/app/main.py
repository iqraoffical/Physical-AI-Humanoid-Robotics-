from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.api import query_endpoint, selected_text_endpoint, health_endpoint
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

    # Add middleware
    app.add_middleware(LoggingMiddleware)

    # --- ADD CORS HERE ---
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            
            " http://127.0.0.1:8000",
            "https://physical-ai-humanoid-robotics-anui.vercel.app",
        ],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )


    # Include API routes
    app.include_router(query_endpoint.router, prefix=settings.API_PREFIX, tags=["query"])
    app.include_router(selected_text_endpoint.router, prefix=settings.API_PREFIX, tags=["selected-text"])
    app.include_router(health_endpoint.router, prefix=settings.API_PREFIX, tags=["health"])

    return app


app = create_app()


@app.on_event('startup')
async def startup_event():
    setup_logging()  # Set up logging configuration
    print("Starting up RAG Chatbot API...")


@app.on_event('shutdown')
async def shutdown_event():
    print("Shutting down RAG Chatbot API...")