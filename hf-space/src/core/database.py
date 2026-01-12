from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import DeclarativeBase
from src.core.config import settings

# Create async engine
# Neon requires SSL. asyncpg uses a different approach than psycopg2 for SSL.
db_url = settings.NEON_DATABASE_URL
if "postgresql://" in db_url and "postgresql+asyncpg://" not in db_url:
    db_url = db_url.replace("postgresql://", "postgresql+asyncpg://")

# Remove sslmode from URL as asyncpg doesn't support it as a query param
if "sslmode=" in db_url:
    import re
    db_url = re.sub(r"[\?&]sslmode=[^&]+", "", db_url)

engine = create_async_engine(
    db_url,
    echo=settings.ENVIRONMENT == "development",
    future=True,
    connect_args={"ssl": True} if "neon.tech" in db_url else {}
)

# Create session maker
async_session_maker = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

class Base(DeclarativeBase):
    """Base class for SQLAlchemy models"""
    pass

async def get_db():
    """Dependency for getting async database sessions"""
    async with async_session_maker() as session:
        try:
            yield session
        finally:
            await session.close()
