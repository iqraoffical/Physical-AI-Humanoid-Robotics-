from typing import Dict, List, Any


def format_citation(citation_data: Dict[str, Any]) -> str:
    """
    Format a citation according to the specification
    """
    if citation_data.get("source") == "selected_text":
        # Handle selected text citations
        context = citation_data.get("context", "")
        return f"Source: User selected text - {context[:100]}{'...' if len(context) > 100 else ''}"
    else:
        # Handle book content citations
        chapter = citation_data.get("chapter", "Unknown")
        section = citation_data.get("section", "Unknown")
        page = citation_data.get("page", "N/A")
        url = citation_data.get("url", "")
        
        citation = f"Chapter {chapter} – Section {section} (Page {page})"
        if url:
            citation += f" - {url}"
        return citation


def validate_citation_format(citation_data: Dict[str, Any]) -> bool:
    """
    Validate that the citation follows the required format
    """
    if citation_data.get("source") == "selected_text":
        # For selected text, validate context field
        return "context" in citation_data
    else:
        # For book content, validate chapter/section/page structure
        required_fields = ["chapter", "section", "page"]
        return all(field in citation_data for field in required_fields)


def extract_citations_from_response(response_text: str, context_chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Extract citation information from the response and context
    """
    citations = []
    for chunk in context_chunks:
        citation = {
            "chapter": chunk.get("chapter"),
            "section": chunk.get("section"),
            "page": chunk.get("page"),
            "url": chunk.get("url")
        }
        citations.append(citation)
    return citations