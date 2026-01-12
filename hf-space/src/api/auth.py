from fastapi import APIRouter, HTTPException, Depends, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from pydantic import BaseModel, EmailStr, Field
from typing import List, Optional
import uuid

from src.core.database import get_db
from src.core.security import get_password_hash, create_access_token, verify_password
from src.models.user import User
from src.models.personalization_profile import PersonalizationProfile

router = APIRouter()

# --- Schemas ---
class SignupRequest(BaseModel):
    email: EmailStr
    password: str = Field(..., min_length=8)
    name: str = Field(..., min_length=2)
    software_background: str = "beginner"
    hardware_background: str = "none"
    learning_goals: List[str] = []
    preferred_complexity: str = "standard"

class SigninRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user_id: uuid.UUID
    name: str

# --- Endpoints ---
@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: SignupRequest, db: AsyncSession = Depends(get_db)):
    """User signup with background questions for personalization"""
    # 1. Check if user already exists
    result = await db.execute(select(User).where(User.email == request.email))
    if result.scalar_one_or_none():
        raise HTTPException(status_code=409, detail="User with this email already exists")

    # 2. Create user
    new_user = User(
        email=request.email,
        name=request.name,
        hashed_password=get_password_hash(request.password)
    )
    db.add(new_user)
    await db.flush() # Get user id

    # 3. Create profile with background answers
    new_profile = PersonalizationProfile(
        user_id=new_user.id,
        software_background=request.software_background,
        hardware_background=request.hardware_background,
        learning_goals=request.learning_goals,
        preferred_complexity=request.preferred_complexity
    )
    db.add(new_profile)

    await db.commit()
    await db.refresh(new_user)

    # 4. Generate token
    token = create_access_token(subject=new_user.id)
    return {
        "access_token": token,
        "user_id": new_user.id,
        "name": new_user.name
    }

@router.post("/signin", response_model=AuthResponse)
async def signin(request: SigninRequest, db: AsyncSession = Depends(get_db)):
    """Standard user signin"""
    result = await db.execute(select(User).where(User.email == request.email))
    user = result.scalar_one_or_none()

    if not user or not verify_password(request.password, user.hashed_password):
        raise HTTPException(status_code=401, detail="Invalid email or password")

    token = create_access_token(subject=user.id)
    return {
        "access_token": token,
        "user_id": user.id,
        "name": user.name
    }
