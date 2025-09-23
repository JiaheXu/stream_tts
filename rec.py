from openai import OpenAI
client = OpenAI(base_url="http://192.168.0.100:8880/v1", api_key="not-needed")

with client.audio.speech.with_streaming_response.create(
    model="kokoro",  
    voice="af_bella+af_sky", # see /api/src/core/openai_mappings.json to customize
    input="Hello world!",
    response_format="mp3"
) as response:
    response.stream_to_file("output.mp3")

response.stream_to_file("output.mp3")
