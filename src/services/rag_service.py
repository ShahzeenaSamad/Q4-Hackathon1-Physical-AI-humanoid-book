"""
RAG Service - Retrieval-Augmented Generation for textbook Q&A
"""

from typing import List, Dict, Optional
from src.core.config import settings
from src.services.cohere_client import cohere_client
from src.services.qdrant_service import qdrant_service


class RAGService:
    """Service for answering questions using RAG over textbook content"""

    def __init__(self):
        self.cohere = cohere_client
        self.qdrant = qdrant_service
        self.completion_model = settings.COMPLETION_MODEL

    def answer_question(
        self,
        question: str,
        module_filter: Optional[str] = None,
        top_k: int = 5,
        selected_text: Optional[str] = None
    ) -> Dict:
        """
        Answer a question using RAG over textbook chapters.
        If selected_text is provided, it overrides general context.
        """
        sources = []
        context_docs = []

        # Step 1: Handle Selective Reasoning vs general RAG
        if selected_text:
            context_docs.append({
                "title": "User Selected Text",
                "text": selected_text
            })
            model_preamble = "You are a specialized AI assistant. Answer the user's question ONLY using the 'User Selected Text' provided. If the answer is not in the text, politely say so. Always cite the Source: User Selected Text."
        else:
            # Step 1a: Generate embedding for the query
            query_vector = self.cohere.get_embeddings([question], input_type="search_query")[0]

            # Step 1b: Retrieve relevant chunks from Qdrant
            search_results = self.qdrant.search(
                query_vector=query_vector,
                limit=top_k
            )

            if not search_results:
                return {
                    "answer": "I couldn't find relevant information in the textbook. Please try asking about Physical AI, ROS 2, Isaac Sim, or VLA.",
                    "sources": [],
                    "context_used": False
                }

            # Step 1c: Format results for Cohere
            for i, res in enumerate(search_results):
                payload = res.payload
                context_docs.append({
                    "title": payload.get("title", f"Chapter {i}"),
                    "text": payload.get("content", "")
                })
                sources.append({
                    "title": payload.get("title"),
                    "chapter_id": payload.get("chapter_id"),
                    "module": payload.get("module"),
                    "score": res.score
                })

            model_preamble = "You are a specialized AI assistant for the Physical AI & Humanoid Robotics Textbook. Answer accurately based on the provided context. Cite your sources."

        # Step 2: Generate response
        if selected_text:
            # Selective Reasoning: Use single focused call
            prompt = f"Selected Text: {selected_text}\n\nQuestion: {question}"
            answer = self.cohere.generate_response(prompt=prompt)
        else:
            # General RAG: Use documents for grounding
            answer = self.cohere.generate_response(
                prompt=question,
                search_results=context_docs
            )

        return {
            "answer": answer,
            "sources": sources,
            "context_used": True,
            "model_name": self.completion_model
        }

    def suggest_related_topics(self, question: str, limit: int = 5) -> List[Dict]:
        """
        Suggest related chapters/topics based on a question

        Args:
            question: User's query
            limit: Number of suggestions to return

        Returns:
            List of chapter metadata dicts
        """
        question_embedding = self.cohere.get_embeddings([question], input_type="search_query")[0]

        results = self.qdrant.search(
            query_vector=question_embedding,
            limit=limit
        )

        return [
            {
                "chapter_id": r.payload["chapter_id"],
                "title": r.payload["title"],
                "module": r.payload["module"],
                "relevance_score": round(r.score, 3)
            }
            for r in results
        ]

    def get_chapter_summary(self, chapter_id: str) -> Optional[Dict]:
        """
        Get summary information about a specific chapter

        Args:
            chapter_id: Chapter identifier

        Returns:
            Dict with chapter metadata or None if not found
        """
        chapter = self.qdrant.get_chapter_by_id(chapter_id)
        if chapter:
            payload = chapter.payload
            return {
                "chapter_id": payload["chapter_id"],
                "title": payload["title"],
                "module": payload["module"],
                "estimated_time": payload.get("estimated_time"),
                "prerequisites": payload.get("prerequisites", [])
            }
        return None
