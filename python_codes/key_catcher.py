import serial
import keyboard
import threading
import time

PORT = 'COM4'
BAUD = 1000000  # Must match FPGA UART BAUD_RATE parameter (1 Mbaud)

KEY_MAP = {
    # Movement
    'w':          0,   # forward
    's':          1,   # backward
    'a':          2,   # turn left
    'd':          3,   # turn right
    'left':       2,   # turn left (arrow)
    'right':      3,   # turn right (arrow)
    'up':         0,   # forward (arrow)
    'down':       1,   # backward (arrow)

    # Combat
    'ctrl':       4,   # fire
    'space':      5,   # use / open door
    'shift':      6,   # run

    # Strafe
    'alt':        9,   # strafe modifier
    ',':          10,  # strafe left
    '.':          11,  # strafe right

    # Weapons
    '1':          12,  # fist / chainsaw
    '2':          13,  # pistol
    '3':          14,  # shotgun
    '4':          15,  # chaingun
    '5':          16,  # rocket launcher
    '6':          17,  # plasma rifle
    '7':          18,  # BFG 9000

    # Menu / System
    'esc':        7,   # menu
    'enter':      8,   # confirm
    'tab':        19,  # automap
    'f1':         20,  # help
    'f2':         21,  # save
    'f3':         22,  # load
    'f5':         23,  # detail toggle
    'f6':         24,  # quicksave
    'f7':         25,  # end game
    'f8':         26,  # messages toggle
    'f9':         27,  # quickload
    'f10':        28,  # quit
    'f11':        29,  # gamma correction
    'pause':      30,  # pause game
    '-':          31,  # screen size down
    '=':          31,  # screen size up -- reuse or remap if needed
}

ser = serial.Serial(PORT, baudrate=BAUD, timeout=0)
held = set()

def rx_thread():
    while True:
        try:
            waiting = ser.in_waiting
            if waiting > 0:
                data = ser.read(waiting)
                cleaned = bytes(b for b in data if b != 0x00)
                text = cleaned.decode('ascii', errors='replace')
                print(text, end='', flush=True)
            else:
                time.sleep(0.001)
        except Exception as e:
            print(f"\n[RX ERROR] {e}", flush=True)
            break
        
def on_key(e):
    name = e.name.lower()
    if name not in KEY_MAP:
        return
    idx = KEY_MAP[name]
    if e.event_type == 'down' and name not in held:
        held.add(name)
        ser.write(bytes([0x01, idx]))
        print(f"[TX] press   → {name} (bit {idx})", flush=True)
    elif e.event_type == 'up' and name in held:
        held.discard(name)
        ser.write(bytes([0x00, idx]))
        print(f"[TX] release → {name} (bit {idx})", flush=True)

print(f"Connected to {PORT} @ {BAUD} baud")
print("Keys: W/A/S/D, Ctrl, Space, Shift, Esc, Enter")
print("Press Ctrl+C to quit\n", flush=True)

t = threading.Thread(target=rx_thread, daemon=True)
t.start()

keyboard.hook(on_key)

try:
    while True:
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    keyboard.unhook_all()
    ser.close()
    print("\nDisconnected.")