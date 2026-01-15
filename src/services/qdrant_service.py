from qdrant_client import QdrantClient
from qdrant_client.http import models
from src.core.config import settings
import logging

logger = logging.getLogger(__name__)

class QdrantService:
    """
    Qdrant vector store service for semantic search.
    """

    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_CLUSTER_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=60,  # Increase timeout to 60 seconds
        )
        self.collection_name = "physical_ai_textbook"
        self._ensure_collection()

    def _ensure_collection(self):
        """Create collection if it doesn't exist"""
        try:
            collections = self.client.get_collections().collections
            exists = any(c.name == self.collection_name for c in collections)

            if not exists:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=settings.EMBEDDING_DIMENSION,
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error ensuring Qdrant collection: {str(e)}")

    def upsert_chunks(self, points: list):
        """Insert or update chunks in Qdrant"""
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
        except Exception as e:
            logger.error(f"Error upserting to Qdrant: {str(e)}")
            raise

    def search(self, query_vector: list[float], limit: int = 5):
        """Perform semantic search"""
        try:
            result = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit
            )
            return result.points
        except Exception as e:
            logger.error(f"Error searching Qdrant: {str(e)}")
            import traceback
            traceback.print_exc()
            raise

    def get_chapter_by_id(self, chapter_id: str):
        """Retrieve a specific chapter by its ID"""
        try:
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="chapter_id",
                            match=models.MatchValue(value=chapter_id),
                        )
                    ]
                ),
                limit=1,
                with_payload=True,
                with_vectors=False,
            )
            return results[0][0] if results[0] else None
        except Exception as e:
            logger.error(f"Error fetching chapter from Qdrant: {str(e)}")
            return None

# Singleton instance
qdrant_service = QdrantService()
