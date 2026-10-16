from scapy.all import sniff, UDP, Raw

def handle_pkt(pkt):
    if UDP in pkt and pkt[UDP].dport == 43893:
        if Raw in pkt:
            try:
                data = pkt[Raw].load.decode(errors="replace")
                print(f"[{pkt[IP].src}:{pkt[UDP].sport}] {data}")
            except Exception:
                print(f"[{pkt[IP].src}:{pkt[UDP].sport}] <binary data>")

print("✅ Sniffing UDP packets on port 43893 (no binding)...")
while(True):
    sniff(filter="udp port 43893", prn=handle_pkt, store=0)
