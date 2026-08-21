import serial
import json
import time
import statistics

RADIO_PORT = "COM9"   # change this
BAUD = 57600

PING_COUNT = 100
PING_INTERVAL = 0.5
TIMEOUT = 2.0

ser = serial.Serial(RADIO_PORT, BAUD, timeout=TIMEOUT)

latencies_ms = []

print(f"PING using {RADIO_PORT} at {BAUD}...")
print("Press Ctrl+C to stop.\n")

try:
    for seq in range(PING_COUNT):
        msg = {
            "type": "ping",
            "seq": seq,
            "tx_time": time.time()
        }

        send_time = time.perf_counter()

        ser.write((json.dumps(msg) + "\n").encode("utf-8"))
        ser.flush()

        line = ser.readline().decode("utf-8", errors="ignore").strip()

        recv_time = time.perf_counter()

        if not line:
            print(f"Ping {seq:03d}: TIMEOUT")
            time.sleep(PING_INTERVAL)
            continue

        try:
            reply = json.loads(line)

            if reply.get("type") == "pong" and reply.get("seq") == seq:
                rtt_ms = (recv_time - send_time) * 1000
                latencies_ms.append(rtt_ms)

                print(f"Ping {seq:03d}: RTT = {rtt_ms:.2f} ms")
            else:
                print(f"Ping {seq:03d}: Wrong reply: {reply}")

        except Exception:
            print(f"Ping {seq:03d}: Bad reply: {line}")

        time.sleep(PING_INTERVAL)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    ser.close()

    if latencies_ms:
        print("\n===== Ping Summary =====")
        print(f"Packets sent: {PING_COUNT}")
        print(f"Replies: {len(latencies_ms)}")
        print(f"Packet loss: {(1 - len(latencies_ms) / PING_COUNT) * 100:.1f}%")
        print(f"Min RTT: {min(latencies_ms):.2f} ms")
        print(f"Avg RTT: {statistics.mean(latencies_ms):.2f} ms")
        print(f"Max RTT: {max(latencies_ms):.2f} ms")
        print(f"Std dev: {statistics.stdev(latencies_ms):.2f} ms" if len(latencies_ms) > 1 else "Std dev: N/A")
    else:
        print("No replies received.")