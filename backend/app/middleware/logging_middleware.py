import time
import logging
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response


class LoggingMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        # Log request
        start_time = time.time()
        response = await call_next(request)
        process_time = time.time() - start_time
        
        # Extract path and method
        path = request.url.path
        method = request.method
        
        # Log the request and response
        logging.info(f"{method} {path} - {response.status_code} - {process_time:.4f}s")
        
        return response


def setup_logging():
    """Configure logging for the application"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )