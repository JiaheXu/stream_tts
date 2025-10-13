#!/usr/bin/env python3
"""
UDP Client Test for stream_pc.py
--------------------------------
Sends JSON commands to the TTS UDP server (stream_pc.py).
"""

import socket
import json
import time

SERVER_IP = "127.0.0.1"   # Change if stream_pc.py runs on another machine
SERVER_PORT = 8888

def send_command(command: dict):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        data = json.dumps(command).encode("utf-8")
        sock.sendto(data, (SERVER_IP, SERVER_PORT))
        print(f"✅ Sent command: {command}")
    finally:
        sock.close()

def main():
    # Example sequence of test commands
    send_command({"cmd": "play", "text":"start"})   # play random start sound
    time.sleep(2)
    send_command({"cmd": "stop"})  # stop playback
    time.sleep(1)    
    send_command({"cmd": "speak", "text": "你好，欢迎来到大唐芙蓉园", "voice": "zf_xiaoyi", "volume": 2.0})

    # send_command({"cmd": "stop"})  # stop playback
    
    # send_command({"cmd": "set_volume", "volume": 0.8})
    # time.sleep(1)

    # send_command({"cmd": "speak", "text": "这是第二条测试语音", "voice": "zf_xiaoyi"})
    # time.sleep(5)

    # send_command({"cmd": "stop"})  # stop playback

if __name__ == "__main__":
    main()
