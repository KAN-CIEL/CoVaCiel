import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

buffer = ""

print("En attente de messages...")

while True:
    data = ser.read().decode(errors="ignore")
    if not data:
        continue

    buffer += data

    # On traite uniquement si on a un ';'
    if ';' in buffer:
        parts = buffer.split(';')

        for raw_msg in parts[:-1]:  # messages complets
            raw_msg = raw_msg.strip()

            # Verifie que le message commence par '$'
            if not raw_msg.startswith('$'):
                continue  # on ignore

            # On enleve le '$'
            msg = raw_msg[1:]   # exemple : "$STOP" -> "STOP"

            # Traitement des commandes
            if msg == "STOP":
                print("voiture arrete")

            elif msg == "GO":
                print("voiture avance")

            else:
                print("Message inconnu :", msg)

        # On garde le dernier morceau incomplet
        buffer = parts[-1]
