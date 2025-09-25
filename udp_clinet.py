#!/usr/bin/env python3
"""
UDP Client Test
---------------
Sends a text message to UDP server (stream.py) for TTS playback.
"""

import socket

# ============================
# Configuration
# ============================
SERVER_IP = "0.0.0.0"   # Change if stream.py runs on another machine
SERVER_PORT = 8888
MESSAGE = "大家好，这是一个UDP测试消息。"

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(MESSAGE.encode("utf-8"), (SERVER_IP, SERVER_PORT))
        print(f"✅ Sent message: {MESSAGE}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
