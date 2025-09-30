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
SERVER_IP = "192.168.1.120"   # Change if stream.py runs on another machine
SERVER_PORT = 8888
MESSAGE = "地点：城市中央剧场·水晶厅** ✨ **亮点特色：** 1. 全息投影+真人舞蹈跨界融合，呈现宇宙起源神话 2. 国际获奖团队运用AI实时生成视觉特效 3. 观众可通过座椅扶手的感应装置参与互动段落"

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(MESSAGE.encode("utf-8"), (SERVER_IP, SERVER_PORT))
        print(f"✅ Sent message: {MESSAGE}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
