import requests
import numpy as np
import sounddevice as sd
from scipy.signal import resample_poly
from scipy.io.wavfile import write


response = requests.post(
    "http://192.168.1.117:8880/v1/audio/speech",
    json={
        "input": "大家好，我是中文语音合成示例。",
        "voice": "zf_xiaoxiao",
        "language": "Chinese",
        "response_format": "pcm"
    },
    stream=True
)

#for chunk in response.iter_content(chunk_size=1024):
#    if chunk:
#        # Process streaming chunks
#        print("got data")
#        pass
        
# Source sample rate (depends on your TTS server, often 22050 or 24000 Hz)
SRC_SR = 22050
TARGET_SR = 48000
CHANNELS = 1  # adjust if stereo

sd.default.samplerate = TARGET_SR
sd.default.channels = CHANNELS
all_audio = []
all_audio_resampled = []
# Open an output stream
with sd.OutputStream(samplerate=TARGET_SR, channels=CHANNELS, dtype='float32') as stream:
    for chunk in response.iter_content(chunk_size=16384):
        if not chunk:
            continue

        # Convert raw bytes → int16 numpy array (server sends PCM16)
        audio = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0  # normalize [-1,1]

        # Resample to 48 kHz
        resampled = resample_poly(audio, up=320, down=147).astype(np.float32)

        # Play audio
        stream.write(resampled)

        # Save for later
        all_audio.append(audio)
        all_audio_resampled.append(resampled)

# Concatenate full arrays
final_audio = np.concatenate(all_audio).astype(np.float32)
final_audio_resampled = np.concatenate(all_audio_resampled).astype(np.float32)
