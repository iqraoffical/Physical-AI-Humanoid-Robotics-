from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from app.config import settings
import cohere
import logging


class RetrievalService:
    def __init__(self):
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=False  # Using REST API for better compatibility
        )
        self.cohere_client = cohere.Client(settings.COHERE_API_KEY)

        # Initialize collection if it doesn't exist
        try:
            self.qdrant_client.get_collection(settings.QDRANT_COLLECTION_NAME)
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
            )
            logging.info(f"Created Qdrant collection: {settings.QDRANT_COLLECTION_NAME}")

    def search(self, query_text: str, top_k: int = 3, threshold: float = 0.5) -> List[Dict[str, Any]]:
        """
        Search for relevant chunks in the vector database based on the query
        
        Args:
            query_text: The user's query
            top_k: Number of top results to return
            threshold: Minimum similarity score threshold
        
        Returns:
            List of relevant chunks with metadata
        """
        try:
            # Generate embedding for the query using Cohere
            response = self.cohere_client.embed(
                texts=[query_text],
                model=settings.cohere_embedding_model
            )
            query_embedding = response.embeddings[0]
            
            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=settings.qdrant_collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold,
                with_payload=True
            )
            
            # Format results
            results = []
            for hit in search_results:
                if hit.score >= threshold:  # Double-check threshold
                    result = {
                        "content": hit.payload.get("content", ""),
                        "chapter": hit.payload.get("chapter"),
                        "section": hit.payload.get("section"),
                        "page": hit.payload.get("page"),
                        "url": hit.payload.get("url"),
                        "chunk_index": hit.payload.get("chunk_index"),
                        "score": hit.score
                    }
                    results.append(result)
            
            return results
        except Exception as e:
            logging.error(f"Error during Qdrant search: {str(e)}")
            return []

    def embed_and_search_selected_text(self, query_text: str, selected_text: str, top_k: int = 3) -> List[Dict[str, Any]]:
        """
        Embed the selected text temporarily and search for relevant information
        """
        try:
            # Embed the query
            query_response = self.cohere_client.embed(
                texts=[query_text],
                model=settings.cohere_embedding_model
            )
            query_embedding = query_response.embeddings[0]
            
            # Embed the selected text for temporary search context
            text_response = self.cohere_client.embed(
                texts=[selected_text],
                model=settings.cohere_embedding_model
            )
            text_embedding = text_response.embeddings[0]
            
            # Perform a contextual search using both the query and selected text embeddings
            # For this implementation, we'll use the selected text as the primary context
            # and find the most relevant parts of it based on the query
            
            # In a real implementation, we might create a temporary collection or use
            # a different approach, but for this example, we'll simulate the result
            return [{
                "content": selected_text,
                "source": "selected_text",
                "context": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                "score": 0.9  # High confidence since this is the exact selected text
            }]
        except Exception as e:
            logging.error(f"Error during selected text search: {str(e)}")
            return []