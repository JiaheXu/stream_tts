import requests
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from openai import OpenAI

app = FastAPI()
llm_client = OpenAI(base_url="http://localhost:8000/v1", api_key="EMPTY")  # your local llama2 server

KOKORO_URL = "http://localhost:8880/v1/audio/speech"

def stream_tts(text_chunk: str):
    """Send text chunk to kokoro and yield audio stream"""
    response = requests.post(
        KOKORO_URL,
        json={
            "input": text_chunk,
            "voice": "zf_xiaoxiao",
            "language": "Chinese",
            "response_format": "pcm"
        },
        stream=True
    )
    for chunk in response.iter_content(chunk_size=1024):
        if chunk:
            yield chunk

@app.get("/chat")
def chat(query: str):
    """Stream LLM output → stream to Kokoro"""
    llm_stream = llm_client.chat.completions.create(
        model="llama2-7b",
        messages=[{"role": "user", "content": query}],
        stream=True
    )

    buffer = ""
    for event in llm_stream:
        if event.choices[0].delta.content:
            token = event.choices[0].delta.content
            buffer += token

            # Check for sentence boundary
            if token in ["。", ".", "?", "！", "!"]:
                # Stream audio for this chunk
                yield from stream_tts(buffer.strip())
                buffer = ""

    # Flush remaining
    if buffer:
        yield from stream_tts(buffer.strip())

# Start server: uvicorn filename:app --reload

