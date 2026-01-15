from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

class ChatInput(BaseModel):
    message: str

class ChatResponse(BaseModel):
    response: str

@app.post("/chat", response_model=ChatResponse)
async def chat(chat_input: ChatInput):
    """
    Endpoint to receive a chat message and return a response.
    """
    # Aap apni chatbot ki logic yahan likh sakte hain
    # Abhi ke liye, yeh sirf user ka message wapas bhej raha hai
    response_message = f"Aapne kaha: {chat_input.message}"
    return ChatResponse(response=response_message)

@app.get("/")
def read_root():
    return {"message": "Welcome to the Chatbot API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=7860)
