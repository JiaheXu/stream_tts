#!/usr/bin/env python3
"""
UDP → TTS Audio Streamer with Heartbeat, Commands, Queue, and Multi-Voice
-------------------------------------------------------------------------
Commands (JSON over UDP port 8888):

  {"cmd": "speak", "text": "你好", "voice": "zf_xiaoyi", "volume": 2.0}
  {"cmd": "set_volume", "volume": 1.5}
  {"cmd": "stop"}
  {"cmd": "start"}
  {"cmd": "play", "text": "bgm_filename_without_ext"}
"""

import os
import io
import socket
import logging
import requests
import numpy as np
import sounddevice as sd
import soundfile as sf   # 🔥 needed for decoding wav/ogg/flac
from scipy.signal import resample_poly
from scipy.io.wavfile import read
import argparse
import threading
import time
import json
import random
import queue

# ============================
# Config
# ============================
UDP_IP = "0.0.0.0"
UDP_PORT = 8888

TTS_URL = "http://192.168.1.100:8880/v1/audio/speech"
VOICE = "zf_xiaoyi"
LANGUAGE = "Chinese"
RESPONSE_FORMAT = "wav"   # safer than pcm, most servers give wav

SRC_SR = 22050
TARGET_SR = 48000
CHANNELS = 1

VOLUME = 2.0
HEARTBEAT_HZ = 10.0

NOTIFY_IP = "0.0.0.0"
NOTIFY_PORT = 8889

START_SOUNDS_DIR = "start_sounds"
os.makedirs(START_SOUNDS_DIR, exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s",
)
logger = logging.getLogger("UDP-TTS")


# ============================
# TTS Client
# ============================
class TTSClient:
    def __init__(self, url, voice, language, response_format):
        self.url = url
        self.default_voice = voice
        self.language = language
        self.response_format = response_format

    def synthesize(self, text, voice=None, speed=1.0):
        try:
            response = requests.post(
                self.url,
                json={
                    "input": text,
                    "voice": voice if voice else self.default_voice,
                    "language": self.language,
                    "response_format": self.response_format,
                    "speed": speed,
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
    def __init__(self, target_sr=TARGET_SR, channels=CHANNELS,
                 volume=VOLUME, heartbeat_hz=HEARTBEAT_HZ,
                 notify_ip=NOTIFY_IP, notify_port=NOTIFY_PORT):
        self.target_sr = target_sr
        self.channels = channels
        self.volume = volume
        self.heartbeat_hz = heartbeat_hz
        self._stop_flag = threading.Event()
        self.is_playing = False

        sd.default.samplerate = self.target_sr
        sd.default.channels = self.channels

        self.notify_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.notify_addr = (notify_ip, notify_port)

        self._heartbeat_thread = None
        self._heartbeat_running = False

    def stop(self):
        logger.info("🛑 Stop requested")
        self._stop_flag.set()
        self.is_playing = False

    def reset(self):
        """Reset state so player can accept new commands after stop"""
        self._stop_flag.clear()
        self.is_playing = False

    def _send_state(self, state: int):
        try:
            self.notify_sock.sendto(bytes([state]), self.notify_addr)
        except Exception:
            pass

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

    def _play_stream(self, generator):
        """Generic callback player from a generator of audio chunks"""
        self._stop_flag.clear()
        self.is_playing = True
        self._start_heartbeat()

        def callback(outdata, frames, time_info, status):
            try:
                chunk = next(generator)
            except StopIteration:
                raise sd.CallbackStop()
            if self._stop_flag.is_set():
                outdata[:] = np.zeros((frames, self.channels), dtype=np.float32)
                raise sd.CallbackStop()
            if chunk.ndim == 1:
                chunk = np.expand_dims(chunk, axis=1)
            if len(chunk) < frames:
                outdata[:len(chunk)] = chunk
                outdata[len(chunk):] = 0
                raise sd.CallbackStop()
            else:
                outdata[:] = chunk[:frames]

        try:
            with sd.OutputStream(
                samplerate=self.target_sr,
                channels=self.channels,
                dtype="float32",
                blocksize=1024,
                callback=callback,
            ):
                while self.is_playing and not self._stop_flag.is_set():
                    sd.sleep(50)
        finally:
            self.is_playing = False
            self._stop_heartbeat()

    def play_wav_file(self, filepath: str):
        if not os.path.exists(filepath):
            logger.error(f"❌ File not found: {filepath}")
            return
        sr, data = read(filepath)
        if data.dtype != np.float32:
            data = data.astype(np.float32) / np.iinfo(data.dtype).max
        if sr != self.target_sr:
            data = resample_poly(data, up=self.target_sr, down=sr).astype(np.float32)

        def gen():
            for i in range(0, len(data), 1024):
                yield data[i:i+1024]

        self._play_stream(gen())

    def play_tts(self, text, voice, speed, tts_client):
        response = tts_client.synthesize(text, voice, speed)
        if response is None:
            return

        # collect full audio into memory
        audio_bytes = io.BytesIO()
        for chunk in response.iter_content(chunk_size=16384):
            if chunk:
                audio_bytes.write(chunk)
        audio_bytes.seek(0)

        try:
            data, sr = sf.read(audio_bytes, dtype="float32")
        except Exception as e:
            logger.error(f"Failed to decode TTS audio: {e}")
            return

        if sr != self.target_sr:
            data = resample_poly(data, up=self.target_sr, down=sr).astype(np.float32)

        def gen():
            for i in range(0, len(data), 1024):
                if self._stop_flag.is_set():
                    break
                yield data[i:i+1024]

        self._play_stream(gen())

    def play_random_start(self):
        files = [f for f in os.listdir(START_SOUNDS_DIR) if f.lower().endswith(".wav")]
        if not files:
            logger.warning("No start sounds in start_sounds/")
            return
        filepath = os.path.join(START_SOUNDS_DIR, random.choice(files))
        logger.info(f"▶️ Playing start sound: {filepath}")
        self.play_wav_file(filepath)


# ============================
# Audio Worker (Non-blocking, Queue-clearing stop)
# ============================
class AudioWorker(threading.Thread):
    def __init__(self, tts_client, audio_player, cmd_queue):
        super().__init__(daemon=True)
        self.tts_client = tts_client
        self.audio_player = audio_player
        self.cmd_queue = cmd_queue
        self.play_thread = None

    def _start_playback(self, target, *args):
        if self.play_thread and self.play_thread.is_alive():
            self.audio_player.stop()
            self.play_thread.join()
        self.play_thread = threading.Thread(target=target, args=args, daemon=True)
        self.play_thread.start()

    def _clear_queue(self):
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
                self.cmd_queue.task_done()
            except queue.Empty:
                break

    def run(self):
        while True:
            cmd, text, volume, voice, speed = self.cmd_queue.get()
            try:
                if cmd == "speak" and text:
                    self.audio_player.volume = float(volume)
                    self._start_playback(self.audio_player.play_tts, text, voice, speed, self.tts_client)

                elif cmd == "set_volume":
                    self.audio_player.volume = float(volume)
                    logger.info(f"🔊 Volume set to {self.audio_player.volume}x")

                elif cmd == "stop":
                    logger.info("🛑 STOP received: halting playback and clearing queue")
                    self.audio_player.stop()
                    if self.play_thread and self.play_thread.is_alive():
                        self.play_thread.join()
                    self._clear_queue()
                    self.audio_player.reset()   # <-- make sure reset happens here after join


                elif cmd == "start":
                    self._start_playback(self.audio_player.play_random_start)

                elif cmd == "play" and text:
                    filepath = './bgm_wav/' + text + '.wav'
                    self._start_playback(self.audio_player.play_wav_file, filepath)

                else:
                    logger.warning(f"⚠️ Unknown command: {cmd}")
            except Exception as e:
                logger.error(f"Error handling {cmd}: {e}")
            finally:
                self.cmd_queue.task_done()


# ============================
# UDP Server
# ============================
class UDPServer:
    def __init__(self, ip, port, cmd_queue):
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
                    continue

                cmd = payload.get("cmd", "").lower()
                text = payload.get("text", "")
                volume = payload.get("volume", 2.0)
                voice = payload.get("voice", None)
                speed = payload.get("speed", 1.0)

                self.cmd_queue.put((cmd, text, volume, voice, speed))
            except Exception as e:
                logger.error(f"UDP error: {e}")


# ============================
# Entry Point
# ============================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--volume", type=float, default=VOLUME)
    parser.add_argument("--tts_url", type=str, default=TTS_URL)
    parser.add_argument("--voice", type=str, default=VOICE)
    parser.add_argument("--language", type=str, default=LANGUAGE)
    parser.add_argument("--port", type=int, default=UDP_PORT)
    args = parser.parse_args()

    tts_client = TTSClient(args.tts_url, args.voice, args.language, RESPONSE_FORMAT)
    audio_player = AudioPlayer(volume=args.volume)

    cmd_queue = queue.Queue()
    worker = AudioWorker(tts_client, audio_player, cmd_queue)
    worker.start()

    server = UDPServer(UDP_IP, args.port, cmd_queue)
    server.run()


if __name__ == "__main__":
    main()
