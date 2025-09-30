#!/usr/bin/env python3
"""
UDP → TTS Audio Streamer with Heartbeat, Commands, Queue, and Multi-Voice
-------------------------------------------------------------------------
Commands (JSON over UDP port 8888):

  {"cmd": "speak", "message": "你好", "voice": "zf_xiaoyi", "volume": 2.0}
  {"cmd": "set_volume", "volume": 1.5}
  {"cmd": "stop"}
  {"cmd": "start"}
  {"cmd": "play", "message": "/path/to/audio.wav"}
"""

import os
import socket
import logging
import requests
import numpy as np
import sounddevice as sd
from scipy.signal import resample_poly
from scipy.io.wavfile import write, read
from typing import Optional
from datetime import datetime
import argparse
import threading
import time
import json
import random
import queue

# ============================
# Default Configuration
# ============================
UDP_IP: str = "0.0.0.0"
UDP_PORT: int = 8888

TTS_URL: str = "http://192.168.1.100:8880/v1/audio/speech"
VOICE: str = "zf_xiaoyi"
LANGUAGE: str = "Chinese"
RESPONSE_FORMAT: str = "pcm"

SRC_SR: int = 22050
TARGET_SR: int = 48000
CHANNELS: int = 1

VOLUME: float = 2.0
HEARTBEAT_HZ: float = 10.0

NOTIFY_IP: str = "192.168.1.100"
NOTIFY_PORT: int = 8889

OUTPUT_DIR: str = "output_audio"
START_SOUNDS_DIR: str = "start_sounds"
os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(START_SOUNDS_DIR, exist_ok=True)

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
        self.default_voice = voice
        self.language = language
        self.response_format = response_format

    def synthesize(self, text: str, voice: Optional[str] = None) -> Optional[requests.Response]:
        """Send text to TTS server with optional voice override."""
        try:
            response = requests.post(
                self.url,
                json={
                    "input": text,
                    "voice": voice if voice else self.default_voice,
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
    def __init__(self, target_sr: int = TARGET_SR, channels: int = CHANNELS,
                 volume: float = VOLUME, heartbeat_hz: float = HEARTBEAT_HZ,
                 notify_ip: str = NOTIFY_IP, notify_port: int = NOTIFY_PORT):
        self.target_sr = target_sr
        self.channels = channels
        self.volume = volume
        self.heartbeat_hz = heartbeat_hz
        self._stop_flag = threading.Event()

        sd.default.samplerate = self.target_sr
        sd.default.channels = self.channels

        # UDP socket for heartbeat
        self.notify_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.notify_addr = (notify_ip, notify_port)

        self._heartbeat_thread = None
        self._heartbeat_running = False

    def stop(self):
        logger.info("🛑 Stop requested")
        self._stop_flag.set()

    def _send_state(self, state: int):
        try:
            self.notify_sock.sendto(bytes([state]), self.notify_addr)
        except Exception as e:
            logger.warning(f"Failed to send state {state}: {e}")

    def _heartbeat_loop(self):
        period = 1.0 / self.heartbeat_hz
        while self._heartbeat_running:
            self._send_state(1)
            time.sleep(period)

    def _start_heartbeat(self):
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def _stop_heartbeat(self):
        self._heartbeat_running = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=0.5)
        self._send_state(0)

    def play_audio_array(self, audio_data: np.ndarray, samplerate: int):
        if audio_data is None or len(audio_data) == 0:
            return
        try:
            self._stop_flag.clear()
            self._start_heartbeat()
            with sd.OutputStream(samplerate=self.target_sr, channels=self.channels, dtype="float32") as stream:
                for i in range(0, len(audio_data), 1024):
                    if self._stop_flag.is_set():
                        logger.info("Playback stopped by user.")
                        break
                    chunk = audio_data[i:i+1024]
                    stream.write(chunk)
        finally:
            self._stop_heartbeat()

    def play_and_save_tts(self, response: requests.Response, text: str, src_sr: int = SRC_SR):
        if response is None:
            return
        all_audio = []
        try:
            self._stop_flag.clear()
            self._start_heartbeat()
            with sd.OutputStream(samplerate=self.target_sr, channels=self.channels, dtype="float32") as stream:
                for chunk in response.iter_content(chunk_size=16384):
                    if not chunk or self._stop_flag.is_set():
                        break
                    try:
                        audio = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
                        audio *= self.volume
                        audio = np.clip(audio, -1.0, 1.0)
                        resampled = resample_poly(audio, up=self.target_sr, down=src_sr).astype(np.float32)
                        stream.write(resampled)
                        all_audio.append(resampled)
                    except Exception as e:
                        logger.warning(f"Failed to process audio chunk: {e}")
        finally:
            self._stop_heartbeat()
        if all_audio:
            final_audio = np.concatenate(all_audio).astype(np.float32)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_text = "".join(c if c.isalnum() else "_" for c in text)[:30]
            filename = os.path.join(OUTPUT_DIR, f"{timestamp}_{safe_text}.wav")
            write(filename, self.target_sr, final_audio)
            logger.info(f"💾 Saved audio to {filename} (vol={self.volume}x)")

    def play_wav_file(self, filepath: str):
        if not os.path.exists(filepath):
            logger.error(f"❌ File not found: {filepath}")
            return
        sr, data = read(filepath)
        if data.dtype != np.float32:
            data = data.astype(np.float32) / np.iinfo(data.dtype).max
        if sr != self.target_sr:
            data = resample_poly(data, up=self.target_sr, down=sr).astype(np.float32)
        self.play_audio_array(data, self.target_sr)

    def play_random_start(self):
        files = [f for f in os.listdir(START_SOUNDS_DIR) if f.lower().endswith(".wav")]
        if not files:
            logger.warning("No start sounds in start_sounds/")
            return
        filepath = os.path.join(START_SOUNDS_DIR, random.choice(files))
        logger.info(f"▶️ Playing start sound: {filepath}")
        self.play_wav_file(filepath)


# ============================
# Audio Worker (Queue Consumer)
# ============================
class AudioWorker(threading.Thread):
    def __init__(self, tts_client: TTSClient, audio_player: AudioPlayer, cmd_queue: queue.Queue):
        super().__init__(daemon=True)
        self.tts_client = tts_client
        self.audio_player = audio_player
        self.cmd_queue = cmd_queue

    def run(self):
        while True:
            cmd, message, volume, voice = self.cmd_queue.get()
            try:
                if cmd == "speak" and message:
                    self.audio_player.volume = float(volume)
                    logger.info(f"🗣️ Speak: '{message}' (vol={self.audio_player.volume}, voice={voice or self.tts_client.default_voice})")
                    response = self.tts_client.synthesize(message, voice)
                    self.audio_player.play_and_save_tts(response, message)

                elif cmd == "set_volume":
                    self.audio_player.volume = float(volume)
                    logger.info(f"🔊 Volume set to {self.audio_player.volume}x")

                elif cmd == "stop":
                    self.audio_player.stop()

                elif cmd == "start":
                    self.audio_player.play_random_start()

                elif cmd == "play" and message:
                    self.audio_player.play_wav_file(message)

                else:
                    logger.warning(f"⚠️ Unknown command: {cmd}")
            except Exception as e:
                logger.error(f"Error handling {cmd}: {e}")
            finally:
                self.cmd_queue.task_done()


# ============================
# UDP Listener
# ============================
class UDPServer:
    def __init__(self, ip: str, port: int, cmd_queue: queue.Queue):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.cmd_queue = cmd_queue
        logger.info(f"Listening for UDP on {ip}:{port}")

    def run(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                text = data.decode("utf-8").strip()
                if not text:
                    continue
                try:
                    payload = json.loads(text)
                except json.JSONDecodeError:
                    logger.warning(f"Invalid JSON from {addr}: {text}")
                    continue

                cmd = payload.get("cmd", "").lower()
                message = payload.get("message", "")
                volume = payload.get("volume", 2.0)
                voice = payload.get("voice", None)

                self.cmd_queue.put((cmd, message, volume, voice))
            except Exception as e:
                logger.error(f"UDP error: {e}")


# ============================
# Entry Point with CLI
# ============================
def main():
    parser = argparse.ArgumentParser(description="UDP → TTS Audio Streamer with Queue & Multi-Voice")
    parser.add_argument("--volume", type=float, default=VOLUME, help="Default playback volume")
    parser.add_argument("--tts_url", type=str, default=TTS_URL, help="TTS server URL")
    parser.add_argument("--voice", type=str, default=VOICE, help="Default TTS voice")
    parser.add_argument("--language", type=str, default=LANGUAGE, help="TTS language")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP port")
    parser.add_argument("--heartbeat_hz", type=float, default=HEARTBEAT_HZ, help="Heartbeat frequency (Hz)")
    parser.add_argument("--notify_ip", type=str, default=NOTIFY_IP, help="Heartbeat notify IP")
    parser.add_argument("--notify_port", type=int, default=NOTIFY_PORT, help="Heartbeat notify port")
    args = parser.parse_args()

    tts_client = TTSClient(args.tts_url, args.voice, args.language, RESPONSE_FORMAT)
    audio_player = AudioPlayer(volume=args.volume,
                               heartbeat_hz=args.heartbeat_hz,
                               notify_ip=args.notify_ip,
                               notify_port=args.notify_port)

    cmd_queue = queue.Queue()
    worker = AudioWorker(tts_client, audio_player, cmd_queue)
    worker.start()

    server = UDPServer(UDP_IP, args.port, cmd_queue)
    server.run()


if __name__ == "__main__":
    main()

