import serial
import time
import json

PORT = "COM9"      # Change this to talker laptop 433 COM port
BAUD = 57600

PING_INTERVAL = 1.0
TIMEOUT = 2.0

ser = serial.Serial(PORT, BAUD, timeout=0.1)
print(f"Sending pings on {PORT} at {BAUD} baud...")

seq = 0
sent_packets = 0
received_packets = 0
total_bytes = 0

while True:
    send_time = time.time()

    ping = {
        "type": "ping",
        "seq": seq,
        "time": send_time,
        "msg": "hello_433_test"
    }

    packet = json.dumps(ping) + "\n"
    ser.write(packet.encode())

    sent_packets += 1
    total_bytes += len(packet)

    print(f"\nSent ping {seq}")

    start_wait = time.time()
    got_reply = False

    while time.time() - start_wait < TIMEOUT:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        try:
            reply = json.loads(line)

            if reply.get("type") == "pong" and reply.get("seq") == seq:
                receive_time = time.time()

                rtt_ms = (receive_time - send_time) * 1000
                one_way_ms = rtt_ms / 2
                received_packets += 1

                speed_bps = total_bytes / (receive_time - send_time + 0.0001)
                packet_loss = ((sent_packets - received_packets) / sent_packets) * 100

                print(f"Reply received for ping {seq}")
                print(f"RTT: {rtt_ms:.2f} ms")
                print(f"Estimated one-way delay: {one_way_ms:.2f} ms")
                print(f"Packet loss: {packet_loss:.1f}%")
                print(f"Packet size: {len(packet)} bytes")

                got_reply = True
                break

        except:
            pass

    if not got_reply:
        print(f"Ping {seq} TIMEOUT")

    seq += 1
    time.sleep(PING_INTERVAL)