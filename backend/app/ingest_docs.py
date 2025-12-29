import os
import logging
from pathlib import Path
from typing import List, Optional
from llama_index.core import SimpleDirectoryReader
from llama_index.core.node_parser import SentenceSplitter
from llama_index.llms.cohere import Cohere
from llama_index.embeddings.cohere import CohereEmbedding
from llama_index.vector_stores.qdrant import QdrantVectorStore
from llama_index.core import VectorStoreIndex, StorageContext
from qdrant_client import QdrantClient
from app.config import settings

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def build_index(
    documents_dir: str,
    chunk_size: int = 1024,
    chunk_overlap: int = 200
) -> VectorStoreIndex:
    """
    Build a new index from documents and store in Qdrant.

    Args:
        documents_dir: Path to directory containing documents to ingest
        chunk_size: Size of text chunks for splitting
        chunk_overlap: Overlap between text chunks

    Returns:
        VectorStoreIndex: The created index
    """
    # Create documents directory if it doesn't exist
    docs_path = Path(documents_dir)
    if not docs_path.exists():
        raise ValueError(f"Documents directory does not exist: {documents_dir}")

    # Read documents
    logger.info(f"Reading documents from {documents_dir}")
    documents = SimpleDirectoryReader(
        input_dir=documents_dir,
        recursive=True,
        required_exts=[".pdf", ".txt", ".docx", ".md"]  # Default extensions
    ).load_data()

    if not documents:
        raise ValueError(f"No documents found in {documents_dir}")

    logger.info(f"Loaded {len(documents)} documents")

    # Initialize Cohere LLM and embedding model
    llm = Cohere(model=settings.COHERE_GENERATION_MODEL, api_key=settings.COHERE_API_KEY)
    embed_model = CohereEmbedding(
        model_name=settings.COHERE_EMBEDDING_MODEL,
        cohere_api_key=settings.COHERE_API_KEY
    )

    # Set up node parser
    parser = SentenceSplitter(chunk_size=chunk_size, chunk_overlap=chunk_overlap)
    nodes = parser.get_nodes_from_documents(documents)

    # Initialize Qdrant client
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )

    # Create Qdrant vector store
    vector_store = QdrantVectorStore(
        client=client,
        collection_name=settings.QDRANT_COLLECTION_NAME
    )

    # Create storage context
    storage_context = StorageContext.from_defaults(vector_store=vector_store)

    # Create index from nodes and store in Qdrant
    logger.info("Creating index and storing in Qdrant...")
    index = VectorStoreIndex(
        nodes=nodes,
        storage_context=storage_context,
        llm=llm,
        embed_model=embed_model
    )

    logger.info("Document ingestion completed successfully")
    return index


def ingest_documents(
    documents_dir: str,
    chunk_size: int = 1024,
    chunk_overlap: int = 200
) -> None:
    """
    Main function to ingest documents into Qdrant vector store.

    Args:
        documents_dir: Path to directory containing documents to ingest
        chunk_size: Size of text chunks for splitting
        chunk_overlap: Overlap between text chunks
    """
    try:
        # Build the index
        index = build_index(
            documents_dir=documents_dir,
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap
        )

        # Note: ref_doc_info is not supported by all vector store integrations
        # For Qdrant, we can't access ref_doc_info, so we'll just log success
        logger.info("Index created successfully")

    except Exception as e:
        logger.error(f"Error during document ingestion: {str(e)}")
        raise


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Ingest documents for RAG Chatbot API")
    parser.add_argument(
        "--documents_dir",
        type=str,
        default="./data",
        help="Path to directory containing documents to ingest (default: ./data)"
    )
    parser.add_argument(
        "--chunk_size",
        type=int,
        default=1024,
        help="Size of text chunks for splitting (default: 1024)"
    )
    parser.add_argument(
        "--chunk_overlap",
        type=int,
        default=200,
        help="Overlap between text chunks (default: 200)"
    )

    args = parser.parse_args()

    # Check if required environment variables are set via settings
    # This will trigger validation and load from .env file
    try:
        _ = settings.COHERE_API_KEY
        _ = settings.QDRANT_URL
        _ = settings.QDRANT_API_KEY
    except Exception as e:
        logger.error(f"Missing required environment variables: {str(e)}")
        exit(1)

    logger.info("Starting document ingestion process...")
    ingest_documents(
        documents_dir=args.documents_dir,
        chunk_size=args.chunk_size,
        chunk_overlap=args.chunk_overlap
    )
    logger.info("Document ingestion completed successfully!")