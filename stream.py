#!/usr/bin/env python3
"""
UDP → TTS Audio Streamer
------------------------
Listens for full-sentence text messages over UDP (port 8888),
sends them to a FastAPI TTS service, streams the audio to speakers,
and saves each result as a WAV file.
"""

import os
import socket
import logging
import requests
import numpy as np
import sounddevice as sd
from scipy.signal import resample_poly
from scipy.io.wavfile import write
from typing import Optional
from datetime import datetime

# ============================
# Configuration
# ============================
UDP_IP: str = "0.0.0.0"
UDP_PORT: int = 8888

TTS_URL: str = "http://192.168.1.102:8880/v1/audio/speech"
VOICE: str = "zf_xiaoxiao"
LANGUAGE: str = "Chinese"
RESPONSE_FORMAT: str = "pcm"

SRC_SR: int = 22050   # Source sample rate from TTS server
TARGET_SR: int = 48000  # Target device sample rate
CHANNELS: int = 1

OUTPUT_DIR: str = "output_audio"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ============================
# Logging Setup
# ============================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("UDP-TTS")


# ============================
# TTS Client
# ============================
class TTSClient:
    def __init__(self, url: str, voice: str, language: str, response_format: str):
        self.url = url
        self.voice = voice
        self.language = language
        self.response_format = response_format

    def synthesize(self, text: str) -> Optional[requests.Response]:
        """Send text to TTS server and return streaming response."""
        try:
            response = requests.post(
                self.url,
                json={
                    "input": text,
                    "voice": self.voice,
                    "language": self.language,
                    "response_format": self.response_format,
                },
                stream=True,
                timeout=30,
            )
            response.raise_for_status()
            return response
        except Exception as e:
            logger.error(f"TTS request failed: {e}")
            return None


# ============================
# Audio Player
# ============================
class AudioPlayer:
    def __init__(self, target_sr: int = TARGET_SR, channels: int = CHANNELS):
        self.target_sr = target_sr
        self.channels = channels
        sd.default.samplerate = self.target_sr
        sd.default.channels = self.channels

    def play_and_save(self, response: requests.Response, text: str, src_sr: int = SRC_SR) -> None:
        """Play audio stream in real-time and save it as WAV."""
        if response is None:
            return

        all_audio = []

        with sd.OutputStream(samplerate=self.target_sr, channels=self.channels, dtype="float32") as stream:
            for chunk in response.iter_content(chunk_size=16384):
                if not chunk:
                    continue
                try:
                    # Convert raw bytes → float32 numpy array
                    audio = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
                    # Resample to target rate
                    resampled = resample_poly(audio, up=self.target_sr, down=src_sr).astype(np.float32)

                    # Play
                    stream.write(resampled)

                    # Collect for saving
                    all_audio.append(resampled)
                except Exception as e:
                    logger.warning(f"Failed to process audio chunk: {e}")

        # Save to WAV if audio exists
        if all_audio:
            final_audio = np.concatenate(all_audio).astype(np.float32)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_text = "".join(c if c.isalnum() else "_" for c in text)[:30]  # keep filename safe & short
            filename = os.path.join(OUTPUT_DIR, f"{timestamp}_{safe_text}.wav")
            write(filename, self.target_sr, final_audio)
            logger.info(f"💾 Saved audio to {filename}")


# ============================
# UDP Listener
# ============================
class UDPServer:
    def __init__(self, ip: str, port: int, tts_client: TTSClient, audio_player: AudioPlayer):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.tts_client = tts_client
        self.audio_player = audio_player
        logger.info(f"Listening for UDP messages on {ip}:{port}")

    def run(self) -> None:
        """Main server loop: receive sentence, stream TTS audio, save file."""
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                sentence = data.decode("utf-8").strip()
                if sentence:
                    logger.info(f"Received from {addr}: {sentence}")
                    response = self.tts_client.synthesize(sentence)
                    self.audio_player.play_and_save(response, sentence)
            except Exception as e:
                logger.error(f"Error in UDP loop: {e}")


# ============================
# Entry Point
# ============================
def main() -> None:
    tts_client = TTSClient(TTS_URL, VOICE, LANGUAGE, RESPONSE_FORMAT)
    audio_player = AudioPlayer()
    server = UDPServer(UDP_IP, UDP_PORT, tts_client, audio_player)
    server.run()


if __name__ == "__main__":
    main()

