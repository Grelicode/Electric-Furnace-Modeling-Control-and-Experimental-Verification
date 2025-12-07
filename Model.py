import paho.mqtt.client as mqtt      # Biblioteka klienta MQTT
import struct                        # Do pakowania/rozpakowywania floatów
import time                          # Do pobierania aktualnego czasu
import sys
from collections import deque        # Kolejka FIFO do opóźnienia


BROKER, PORT = "192.168.0.110", 1883

TOPIC_IN   = "py/in"   
TOPIC_OUT  = "py/out"  
TOPIC_MON  = "py/mon"   

TS = 1.0          # [s] krok modelu (PLC wysyła dane co 1 s)
CS = 4200.0       # [J/(kg·°C)]
RHO = 1.0         # [kg/L]
V1 = 1.6          # [L]
PNOM = 12000      # [W]
T0_DELAY = 10.0   # [s] stałe opóźnienie t0

DELAY_LEN = int(round(T0_DELAY / TS))  # długość kolejki opóźnienia

# Stan modelu
Tstar = 25.0   
v1 = 25.0      
v2 = 25.0     

delay_queue = deque([25.0] * DELAY_LEN)


def k_of_Ph(Ph_percent: float) -> float:
    return -0.0002347 * Ph_percent + 1.012


def tau_of_F(F_lmin: float) -> float:
    F = max(F_lmin, 0.0001)              
    tau = 19.08 * (F ** (-0.4293)) - 4.042
    return max(0.1, tau)                 


def furnace_step(Tin: float, F_lmin: float, Ph_percent: float) -> float:
    global Tstar, v1, v2, delay_queue
    Ph = min(max(Ph_percent, 0.0), 100.0)
    F = max(F_lmin, 0.0)

    # Wzmocnienie i stała czasowa
    k = k_of_Ph(Ph)
    tau = tau_of_F(F)
    # Moc elektryczna [W]
    q_el = (Ph * PNOM) / 100.0
    dTdt = (F / (60.0 * V1)) * (Tin - Tstar) + q_el / (CS * RHO * V1)
    dv1_dt = (Tstar * k - v1) / tau  # wyliczenie przyrostow, przemonozenie przez K grzalki (9)
    dv2_dt = (v1 - v2) / tau

    Tstar = Tstar + TS * dTdt
    v1 = v1 + TS * dv1_dt
    v2 = v2 + TS * dv2_dt
    delay_queue.append(v2) 
    Thout = delay_queue.popleft()



    return float(Thout)



def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] connected rc={rc}")
    client.subscribe(TOPIC_IN, qos=0)
    print(f"[MQTT] subscribed to {TOPIC_IN}")


def on_message(client, userdata, msg):
    try:
       
        if msg.topic != TOPIC_IN or len(msg.payload) < 12:
            return

        Tin, Fout, power = struct.unpack('>fff', msg.payload[:12])

    
        Ph_pct = max(0.0, min(100.0, power))

        ts = time.strftime("%H:%M:%S")
        print(
            f"[{ts}] IN  Tin={Tin:.3f}°C | "
            f"Fout={Fout:.3f} L/min | "
            f"Power={power:.3f} (→ {Ph_pct:.1f}%)"
        )

   
        Thout = furnace_step(Tin, Fout, Ph_pct)

        
        client.publish(TOPIC_OUT, struct.pack('>f', Thout), qos=0, retain=False)

        
        client.publish(
            TOPIC_MON,
            struct.pack('>ffff', Tin, Fout, power, Thout),
            qos=0,
            retain=False,
        )

        print(f"[{ts}] OUT Thout={Thout:.3f}°C  → {TOPIC_OUT}")

    except Exception as e:
        print("on_message err:", e)



def main():
    client = mqtt.Client(client_id="PIEC")

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    print("[READY] IN:py/in  OUT:py/out  MON:py/mon  (float32 BE)")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterruppt - closing...")
        client.loop_stop()
        client.disconnect()
        sys.exit(0)


if __name__ == "__main__":
    main()
