"""
Vector Store Service - Interface with Qdrant for semantic search
"""

from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from src.core.config import settings


class VectorStoreService:
    """Service for storing and retrieving embeddings from Qdrant"""

    COLLECTION_NAME = "textbook_chapters"

    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_CLUSTER_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Create collection if it doesn't exist"""
        collections = self.client.get_collections().collections
        collection_names = [col.name for col in collections]

        if self.COLLECTION_NAME not in collection_names:
            self.client.create_collection(
                collection_name=self.COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=settings.EMBEDDING_DIMENSION,  # 1536 for text-embedding-3-small
                    distance=Distance.COSINE
                )
            )
            print(f"Created Qdrant collection: {self.COLLECTION_NAME}")

    def upsert_chapter(self, chapter_data: Dict):
        """
        Insert or update a chapter in the vector store

        Args:
            chapter_data: Dict with 'id', 'vector', and 'payload' keys
        """
        point = PointStruct(
            id=chapter_data["id"],
            vector=chapter_data["vector"],
            payload=chapter_data["payload"]
        )

        self.client.upsert(
            collection_name=self.COLLECTION_NAME,
            points=[point]
        )

    def upsert_chapters_batch(self, chapters_data: List[Dict]):
        """Batch insert/update multiple chapters"""
        points = [
            PointStruct(
                id=chapter["id"],
                vector=chapter["vector"],
                payload=chapter["payload"]
            )
            for chapter in chapters_data
        ]

        self.client.upsert(
            collection_name=self.COLLECTION_NAME,
            points=points
        )

    def search(
        self,
        query_vector: List[float],
        limit: int = 3,
        score_threshold: float = 0.7,
        module_filter: Optional[str] = None
    ) -> List[Dict]:
        """
        Search for similar chapters using vector similarity

        Args:
            query_vector: Embedding vector of the query
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0-1)
            module_filter: Optional module name to filter results

        Returns:
            List of dicts with 'id', 'score', and 'payload' for each match
        """
        search_filter = None
        if module_filter:
            search_filter = Filter(
                must=[
                    FieldCondition(
                        key="module",
                        match=MatchValue(value=module_filter)
                    )
                ]
            )

        results = self.client.query_points(
            collection_name=self.COLLECTION_NAME,
            query=query_vector,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=search_filter
        ).points

        return [
            {
                "id": hit.id,
                "score": hit.score,
                "payload": hit.payload
            }
            for hit in results
        ]

    def get_chapter_by_id(self, chapter_id: str) -> Optional[Dict]:
        """Retrieve a specific chapter by its ID"""
        try:
            point = self.client.retrieve(
                collection_name=self.COLLECTION_NAME,
                ids=[chapter_id]
            )
            if point:
                return {
                    "id": point[0].id,
                    "payload": point[0].payload
                }
            return None
        except Exception:
            return None

    def delete_chapter(self, chapter_id: str):
        """Delete a chapter from the vector store"""
        self.client.delete(
            collection_name=self.COLLECTION_NAME,
            points_selector=[chapter_id]
        )

    def get_collection_info(self) -> Dict:
        """Get information about the collection"""
        info = self.client.get_collection(collection_name=self.COLLECTION_NAME)
        return {
            "name": self.COLLECTION_NAME,
            "vectors_count": info.vectors_count,
            "points_count": info.points_count,
            "status": info.status
        }
