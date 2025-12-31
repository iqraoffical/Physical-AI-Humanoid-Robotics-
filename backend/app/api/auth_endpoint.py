from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from typing import Optional
import uuid
import logging
import hashlib
import secrets
from pydantic import BaseModel

from app.db import get_db
from app.models.user import User
from app.config import settings


router = APIRouter()

class SignUpRequest(BaseModel):
    email: str
    password: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

class SignInRequest(BaseModel):
    email: str
    password: str

class UpdateBackgroundRequest(BaseModel):
    email: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

def hash_password(password: str, salt: str = None) -> tuple[str, str]:
    """Hash a password with a salt"""
    if salt is None:
        salt = secrets.token_hex(16)
    hashed = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt.encode('utf-8'), 100000)
    return hashed.hex(), salt

@router.post("/auth/sign-up")
async def sign_up(
    request: SignUpRequest,
    db: Session = Depends(get_db)
):
    """Create a new user account"""
    try:
        # Check if user already exists
        existing_user = db.query(User).filter(User.email == request.email).first()
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User with this email already exists"
            )

        # Hash the password
        password_hash, salt = hash_password(request.password)
        full_hash = f"{password_hash}:{salt}"

        # Create new user
        user = User(
            email=request.email,
            password_hash=full_hash,
            software_background=request.software_background,
            hardware_background=request.hardware_background
        )

        db.add(user)
        db.commit()
        db.refresh(user)

        return {
            "message": "User created successfully",
            "user_id": user.id,
            "email": user.email
        }

    except HTTPException:
        raise
    except Exception as e:
        logging.error(f"Error creating user: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while creating user"
        )

@router.post("/auth/sign-in")
async def sign_in(
    request: SignInRequest,
    db: Session = Depends(get_db)
):
    """Authenticate a user"""
    try:
        # Find the user
        user = db.query(User).filter(User.email == request.email).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )

        # Verify password
        stored_hash, salt = user.password_hash.split(':')
        provided_hash, _ = hash_password(request.password, salt)

        if stored_hash != provided_hash:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )

        return {
            "message": "Sign in successful",
            "user_id": user.id,
            "email": user.email,
            "software_background": user.software_background,
            "hardware_background": user.hardware_background
        }

    except HTTPException:
        raise
    except Exception as e:
        logging.error(f"Error during sign in: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred during sign in"
        )

@router.post("/user-background")
async def update_user_background(
    request: UpdateBackgroundRequest,
    db: Session = Depends(get_db)
):
    """
    Update user's software and hardware background information
    """
    try:
        # Find the user in the database
        user = db.query(User).filter(User.email == request.email).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        # Update user background information
        if request.software_background is not None:
            user.software_background = request.software_background
        if request.hardware_background is not None:
            user.hardware_background = request.hardware_background

        # Commit changes to the database
        db.commit()
        db.refresh(user)

        return {
            "message": "User background updated successfully",
            "user_id": user.id,
            "email": user.email,
            "software_background": user.software_background,
            "hardware_background": user.hardware_background
        }

    except HTTPException:
        raise
    except Exception as e:
        logging.error(f"Error updating user background: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while updating user background"
        )