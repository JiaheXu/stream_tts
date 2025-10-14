#!/usr/bin/env python3
import os
import socket
import logging
import requests
import numpy as np
import sounddevice as sd
from scipy.signal import resample_poly
from typing import Optional
import argparse
import threading
import json
import queue
from datetime import datetime

# ============================
# Config
# ============================
UDP_IP = "0.0.0.0"
UDP_PORT = 8888

TTS_URL = "http://192.168.1.100:8880/v1/audio/speech"
VOICE = "zf_xiaoyi"
LANGUAGE = "Chinese"

SRC_SR = 22050
TARGET_SR = 48000
CHANNELS = 1
VOLUME = 2.0

CMD_LOG = "cmd_log.json"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s",
)
logger = logging.getLogger("UDP-TTS")


# ============================
# TTS Client (in-memory audio)
# ============================
class TTSClient:
    def __init__(self, url: str, voice: str, language: str):
        self.url = url
        self.default_voice = voice
        self.language = language

    def synthesize(self, text: str, voice: Optional[str] = None) -> Optional[np.ndarray]:
        """Request TTS and return audio samples (float32)."""
        try:
            response = requests.post(
                self.url,
                json={
                    "input": text,
                    "voice": voice if voice else self.default_voice,
                    "language": self.language,
                    "response_format": "pcm",
                },
                timeout=30,
            )
            response.raise_for_status()
            audio = np.frombuffer(response.content, dtype=np.int16).astype(np.float32) / 32768.0
            audio = np.clip(audio * VOLUME, -1.0, 1.0)
            resampled = resample_poly(audio, up=TARGET_SR, down=SRC_SR).astype(np.float32)
            return resampled
        except Exception as e:
            logger.error(f"TTS request failed: {e}")
            return None


# ============================
# Audio Player with Queue
# ============================
class AudioPlayer(threading.Thread):
    def __init__(self, target_sr: int = TARGET_SR, channels: int = CHANNELS):
        super().__init__(daemon=True)
        self.target_sr = target_sr
        self.channels = channels
        self.audio_queue = queue.Queue()
        self._stop_flag = threading.Event()

        sd.default.samplerate = self.target_sr
        sd.default.channels = self.channels

    def run(self):
        while True:
            audio = self.audio_queue.get()
            try:
                if audio is None:  # stop marker
                    self.audio_queue.task_done()
                    continue
                self._stop_flag.clear()
                self._play_audio_array(audio)
            finally:
                self.audio_queue.task_done()

    def _play_audio_array(self, audio: np.ndarray):
        if audio is None or len(audio) == 0:
            return
        try:
            with sd.OutputStream(samplerate=self.target_sr, channels=self.channels, dtype="float32") as stream:
                for i in range(0, len(audio), 1024):
                    if self._stop_flag.is_set():
                        logger.info("Playback stopped mid-audio")
                        break
                    stream.write(audio[i:i+1024])
        except Exception as e:
            logger.error(f"Error during playback: {e}")

    def enqueue(self, audio: np.ndarray):
        self.audio_queue.put(audio)

    def stop_all(self):
        logger.info("🛑 Stop requested — clearing audio queue")
        self._stop_flag.set()
        self._clear_queue()

    def _clear_queue(self):
        try:
            while True:
                self.audio_queue.get_nowait()
                self.audio_queue.task_done()
        except queue.Empty:
            pass
        logger.info("🧹 Audio queue cleared")


# ============================
# Audio Worker
# ============================
class AudioWorker(threading.Thread):
    def __init__(self, tts_client: TTSClient, audio_player: AudioPlayer, cmd_queue: queue.Queue):
        super().__init__(daemon=True)
        self.tts_client = tts_client
        self.audio_player = audio_player
        self.cmd_queue = cmd_queue

    def log_command(self, cmd_dict: dict):
        entry = {"time": datetime.now().isoformat(), **cmd_dict}
        try:
            if not os.path.exists(CMD_LOG):
                with open(CMD_LOG, "w") as f:
                    json.dump([entry], f, indent=2, ensure_ascii=False)
            else:
                with open(CMD_LOG, "r") as f:
                    data = json.load(f)
                data.append(entry)
                with open(CMD_LOG, "w") as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error(f"Failed to log command: {e}")

    def run(self):
        while True:
            cmd, text, volume, voice = self.cmd_queue.get()
            try:
                self.log_command({"cmd": cmd, "text": text, "volume": volume, "voice": voice})
                if cmd == "speak" and text:
                    logger.info(f"🗣️ Speak: '{text}'")
                    audio = self.tts_client.synthesize(text, voice)
                    if audio is not None:
                        self.audio_player.enqueue(audio)

                elif cmd == "set_volume":
                    logger.info(f"🔊 (Volume set, but not applied in memory mode)")

                elif cmd == "stop":
                    self.audio_player.stop_all()

                elif cmd == "play" and text:
                    path = os.path.join("bgm_wav", text + ".wav")
                    if os.path.exists(path):
                        import soundfile as sf
                        data, sr = sf.read(path, dtype="float32")
                        if sr != TARGET_SR:
                            data = resample_poly(data, up=TARGET_SR, down=sr).astype(np.float32)
                        self.audio_player.enqueue(data)
                    else:
                        logger.error(f"❌ File not found: {path}")

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
                    logger.warning(f"Invalid JSON: {text}")
                    continue

                cmd = payload.get("cmd", "").lower()
                text = payload.get("text", "")
                volume = payload.get("volume", 2.0)
                voice = payload.get("voice", None)

                self.cmd_queue.put((cmd, text, volume, voice))
            except Exception as e:
                logger.error(f"UDP error: {e}")


# ============================
# Main
# ============================
def main():
    parser = argparse.ArgumentParser(description="UDP → TTS Audio Queue Player (no wav saving)")
    parser.add_argument("--tts_url", type=str, default=TTS_URL, help="TTS server URL")
    parser.add_argument("--voice", type=str, default=VOICE, help="Default TTS voice")
    args = parser.parse_args()

    tts_client = TTSClient(args.tts_url, args.voice, LANGUAGE)
    audio_player = AudioPlayer()
    audio_player.start()

    cmd_queue = queue.Queue()
    worker = AudioWorker(tts_client, audio_player, cmd_queue)
    worker.start()

    server = UDPServer(UDP_IP, UDP_PORT, cmd_queue)
    server.run()


if __name__ == "__main__":
    main()
