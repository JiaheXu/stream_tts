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
import random 
from datetime import datetime
import time
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
<<<<<<< HEAD
VOLUME = 5.0
=======
VOLUME = 2.0
>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697
HEARTBEAT_HZ: float = 10.0
NOTIFY_IP: str = "192.168.1.100"
NOTIFY_PORT: int = 8889

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

<<<<<<< HEAD
    def synthesize(self, text: str, voice: Optional[str] = None, speed: float = 1.0) -> Optional[np.ndarray]:
        """Request TTS from fastKokoro with speed control."""
        try:
            text = text or ""
            text_preview = text.replace("\n", " ")
            if len(text_preview) > 120:
                text_preview = text_preview[:120] + "…"
            payload = {
                "input": text,
                "voice": voice if voice else self.default_voice,
                "language": self.language,
                "speed": float(speed),
                "response_format": "pcm",
            }

            logger.info(
                "🎤 TTS request → voice=%s, speed=%s, text_len=%d, text_preview='%s'",
                payload["voice"],
                payload["speed"],
                len(text),
                text_preview,
=======
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
>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697
            )

            response = requests.post(self.url, json=payload, timeout=30)
            response.raise_for_status()
<<<<<<< HEAD

            logger.info(
                "📥 TTS response → status=%s, bytes=%d, content_type=%s",
                response.status_code,
                len(response.content),
                response.headers.get("content-type", "unknown"),
            )

            audio = np.frombuffer(response.content, dtype=np.int16).astype(np.float32) / 32768.0
            audio = np.clip(audio * VOLUME, -1.0, 1.0)

            # Assume Kokoro outputs SRC_SR audio, resample to output device rate
            if SRC_SR != TARGET_SR:
                src_samples = len(audio)
                audio = resample_poly(audio, up=TARGET_SR, down=SRC_SR).astype(np.float32)
                logger.info(
                    "🔁 Resample → src_samples=%d (%.2fs @%d), dst_samples=%d (%.2fs @%d)",
                    src_samples,
                    src_samples / float(SRC_SR) if SRC_SR else 0.0,
                    SRC_SR,
                    len(audio),
                    len(audio) / float(TARGET_SR) if TARGET_SR else 0.0,
                    TARGET_SR,
                )
            else:
                logger.info(
                    "🔊 Audio length → samples=%d (%.2fs @%d)",
                    len(audio),
                    len(audio) / float(TARGET_SR) if TARGET_SR else 0.0,
                    TARGET_SR,
                )

            return audio

=======
            audio = np.frombuffer(response.content, dtype=np.int16).astype(np.float32) / 32768.0
            audio = np.clip(audio * VOLUME, -1.0, 1.0)
            resampled = resample_poly(audio, up=TARGET_SR, down=SRC_SR).astype(np.float32)
            return resampled
>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697
        except Exception as e:
            logger.error(f"TTS request failed: {e}")
            return None


# ============================
# Audio Player with Queue
# ============================
class AudioPlayer(threading.Thread):
    def __init__(self, target_sr: int = TARGET_SR, channels: int = CHANNELS, 
                 heartbeat_hz: float = HEARTBEAT_HZ,
                 notify_ip: str = NOTIFY_IP, 
                 notify_port: int = NOTIFY_PORT):
        super().__init__(daemon=True)
        self.target_sr = target_sr
        self.channels = channels
        self.audio_queue = queue.Queue()
        self._stop_flag = threading.Event()

        sd.default.samplerate = self.target_sr
        sd.default.channels = self.channels

        self.heartbeat_hz = heartbeat_hz
        self.notify_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.notify_addr = (notify_ip, notify_port)

        self._heartbeat_thread = None
        self._heartbeat_running = False

    def stop(self):
        logger.info("🛑 Stop requested")
        self._stop_flag.set()

    def _send_state(self, state: int):
        """Send heartbeat state (1=playing, 0=idle)."""
        try:
            self.notify_sock.sendto(bytes([state]), self.notify_addr)
        except Exception as e:
            logger.warning(f"Failed to send state {state}: {e}")

    def _heartbeat_loop(self):
        period = 1.0 / self.heartbeat_hz
        while self._heartbeat_running:
            self._send_state(1)   # keep-alive while playing
            time.sleep(period)

    def _start_heartbeat(self):
        if not self._heartbeat_running:
            self._heartbeat_running = True
            self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self._heartbeat_thread.start()

    def _stop_heartbeat(self):
        if self._heartbeat_running:
            self._heartbeat_running = False
            if self._heartbeat_thread:
                self._heartbeat_thread.join(timeout=0.5)
<<<<<<< HEAD
            self._send_state(0)
=======
            self._send_state(0)   # send idle once
>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697

    def run(self):
        while True:
            audio = self.audio_queue.get()
            try:
<<<<<<< HEAD
                if audio is None:
=======
                if audio is None:  # stop marker
>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697
                    self.audio_queue.task_done()
                    continue
                self._stop_flag.clear()
                self._start_heartbeat()
                self._play_audio_array(audio)
            finally:
                self._stop_heartbeat()
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
<<<<<<< HEAD
=======

>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697


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
            cmd, text, volume, voice, speed = self.cmd_queue.get()
            try:
<<<<<<< HEAD
                self.log_command({
                    "cmd": cmd,
                    "text": text,
                    "volume": volume,
                    "voice": voice,
                    "speed": speed
                })

                if cmd == "speak" and text:
                    logger.info(f"🗣️ Speak: '{text}' | speed={speed}")
                    audio = self.tts_client.synthesize(text, voice, speed)
=======
                self.log_command({"cmd": cmd, "text": text, "volume": volume, "voice": voice})
                if cmd == "speak" and text:
                    logger.info(f"🗣️ Speak: '{text}'")
                    audio = self.tts_client.synthesize(text, voice)
>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697
                    if audio is not None:
                        self.audio_player.enqueue(audio)

                elif cmd == "set_volume":
                    logger.info(f"🔊 (Volume set, but not applied in memory mode)")

                elif cmd == "stop":
                    self.audio_player.stop_all()

                elif cmd == "play" and text:
                    if text == "random":
                        folder = "bgm_wav"
                        try:
                            files = [f for f in os.listdir(folder) if f.lower().endswith(".wav")]
                            if not files:
                                logger.error("❌ No wav files found in bgm_wav")
                            else:
                                chosen = random.choice(files)
                                path = os.path.join(folder, chosen)
                                logger.info(f"🎲 Random play: {chosen}")
                                import soundfile as sf
                                data, sr = sf.read(path, dtype="float32")
                                if sr != TARGET_SR:
                                    data = resample_poly(data, up=TARGET_SR, down=sr).astype(np.float32)
                                self.audio_player.enqueue(data)
                        except Exception as e:
                            logger.error(f"Error playing random wav: {e}")
                    else:
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
                volume = payload.get("volume", 1.2)
                voice = payload.get("voice", None)
                speed = float(payload.get("speed", 1.0))

                self.cmd_queue.put((cmd, text, volume, voice, speed))
            except Exception as e:
                logger.error(f"UDP error: {e}")


# ============================
# Main
# ============================
def main():
<<<<<<< HEAD
    parser = argparse.ArgumentParser(description="UDP → Kokoro-TTS Audio Queue Player (speed supported)")
=======
    parser = argparse.ArgumentParser(description="UDP → TTS Audio Queue Player (no wav saving)")
>>>>>>> 37141a77d764720c4bc6482609a1fe7adc701697
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
