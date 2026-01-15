"""
Cohere Client - Interacts with the Cohere API for completions and embeddings
"""

import cohere
from typing import List, Dict, Any
from src.core.config import settings

class CohereClient:
    """A client for interacting with the Cohere API."""

    def __init__(self):
        self.client = cohere.Client(api_key=settings.COHERE_API_KEY)
        self.completion_model = settings.COMPLETION_MODEL
        self.embedding_model = settings.EMBEDDING_MODEL

    def generate_response(self, prompt: str, search_results: List[Dict[str, Any]] = None) -> str:
        """
        Generates a response using the specified completion model on Cohere.
        """
        try:
            if search_results:
                # Use Cohere's RAG capabilities with documents
                documents = [
                    {"title": doc["title"], "snippet": doc["text"]}
                    for doc in search_results
                ]

                response = self.client.chat(
                    message=prompt,
                    model=self.completion_model,
                    documents=documents,
                    prompt_truncation="AUTO"
                )
            else:
                # Simple chat without documents
                response = self.client.chat(
                    message=prompt,
                    model=self.completion_model
                )

            return response.text.strip()
        except Exception as e:
            print(f"Error generating response from Cohere: {e}")
            import traceback
            traceback.print_exc()
            return "Sorry, I couldn't generate a response at this time."

    def get_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generates embeddings for a list of texts using the specified embedding model on Cohere.
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.embedding_model,
                input_type=input_type
            )
            return response.embeddings
        except Exception as e:
            print(f"Error getting embeddings from Cohere: {e}")
            import traceback
            traceback.print_exc()
            return [[] for _ in texts]

# Global cohere client instance
cohere_client = CohereClient()
