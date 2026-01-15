from fastapi import FastAPI 
import uvicorn 
 
app = FastAPI(title="Physical AI Chatbot", version="1.0") 
 
@app.get("/health") 
async def health_check(): 
    return {"status": "healthy", "service": "Physical AI Chatbot API"} 
 
@app.get("/") 
async def root(): 
    return {"message": "Physical AI Chatbot API is running"} 
 
@app.post("/api/chat") 
async def chat(): 
    return {"response": "Hello from Physical AI Chatbot"} 
 
if __name__ == "__main__": 
    uvicorn.run(app, host="0.0.0.0", port=7860) 
