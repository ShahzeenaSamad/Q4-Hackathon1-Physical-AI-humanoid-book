import os
from src.main import app

# Hugging Face Spaces will look for an 'app' object
# Our FastAPI app will be served through this

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))