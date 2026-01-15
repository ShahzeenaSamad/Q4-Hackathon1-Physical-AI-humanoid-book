"""
OpenRouter Client - Interacts with the OpenRouter API for completions and embeddings
"""

import openai
from typing import List, Dict, Any
from src.core.config import settings

class OpenRouterClient:
    """A client for interacting with the OpenRouter API."""

    def __init__(self):
        self.client = openai.OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=settings.OPENROUTER_API_KEY,
        )
        self.completion_model = settings.COMPLETION_MODEL
        self.embedding_model = settings.EMBEDDING_MODEL

    def generate_response(self, prompt: str, search_results: List[Dict[str, Any]] = None) -> str:
        """
        Generates a response using the specified completion model on OpenRouter.
        """
        messages = []
        if search_results:
            context = "\n".join([f"Source: {res['title']}\nContent: {res['text']}" for res in search_results])
            messages.append({"role": "system", "content": f"You are a helpful assistant. Use the following context to answer the question.\n\n{context}"})
        
        messages.append({"role": "user", "content": prompt})

        try:
            response = self.client.chat.completions.create(
                model=self.completion_model,
                messages=messages,
                extra_headers={
                    "HTTP-Referer": settings.BACKEND_URL,
                    "X-Title": "Physical AI Textbook"
                }
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            # Handle exceptions, e.g., API errors
            print(f"Error generating response from OpenRouter: {e}")
            import traceback
            traceback.print_exc()
            return "Sorry, I couldn't generate a response at this time."

    def get_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generates embeddings for a list of texts using the specified embedding model on OpenRouter.
        The `input_type` parameter is kept for compatibility with the existing RAG service, but it's not directly used by the OpenAI API.
        """
        try:
            # Try with dimensions parameter first
            try:
                response = self.client.embeddings.create(
                    model=self.embedding_model,
                    input=texts,
                    dimensions=settings.EMBEDDING_DIMENSION,
                )
            except Exception:
                # Fallback without dimensions if not supported
                response = self.client.embeddings.create(
                    model=self.embedding_model,
                    input=texts,
                )
            return [embedding.embedding for embedding in response.data]
        except Exception as e:
            # Handle exceptions, e.g., API errors
            print(f"Error getting embeddings from OpenRouter: {e}")
            import traceback
            traceback.print_exc()
            return [[] for _ in texts]

# Global openrouter client instance
openrouter_client = OpenRouterClient()
