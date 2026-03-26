import serial
import threading
import time

PORT = "/dev/ttyUSB1"
BAUD = 9600

# Ouverture du port série USB
ser = serial.Serial(PORT, BAUD, timeout=0.01)

# ---------------------------------------------------------
#  TABLE DES COMMANDES (maître → esclave)
# ---------------------------------------------------------

CMD = {
    0x01: "PING",
    0x02: "GET_DEBUG",
    0x03: "GET_STATUS",
    0x04: "GET_MOTEUR",
    0x05: "SET_MOTEUR",
    0x06: "GET_SERVO",
    0x07: "SET_SERVO",
    0x08: "GET_DISTANCE",
    0x09: "GET_BATTERIE",
    0x0A: "GET_VITESSE",
    0x0F: "RESET_ERREUR"
}

# ---------------------------------------------------------
#  THREAD DE RECEPTION (8 octets)
# ---------------------------------------------------------

##
# @brief Thread de réception UART (asynchrone)
#
# Lit des trames de 8 octets :
# - Octet 1 : commande
# - Octets 2 à 7 : data (6 octets)
# - Octet 8 : checksum (ignoré pour l’instant)
#
# La réception est totalement indépendante de l’émission.
#
def rx_thread():
    buffer = b""

    while True:
        # Lecture non bloquante : on lit seulement ce qui manque
        data = ser.read(8 - len(buffer))

        if data:
            buffer += data

        # Quand on a exactement 8 octets, on traite la trame
        if len(buffer) == 8:
            cmd     = buffer[0]      # Octet 1 : commande
            data    = buffer[1:7]    # Octets 2 à 7 : data (6 octets)
            chk_rx  = buffer[7]      # Octet 8 : checksum (ignoré)

            # Affichage de la commande reçue
            if cmd in CMD:
                print(f"Reçu commande {CMD[cmd]} (0x{cmd:02X})")
            else:
                print(f"Reçu commande inconnue 0x{cmd:02X}")

            print(f"    Data : {data}")
            print(f"    Checksum (ignoré) : 0x{chk_rx:02X}")

            # On vide le buffer pour la prochaine trame
            buffer = b""


# ---------------------------------------------------------
#  THREAD D'EMISSION (8 octets)
# ---------------------------------------------------------

##
# @brief Thread d’émission UART (asynchrone)
#
# Envoie une trame de 8 octets toutes les 2 secondes :
# - Octet 1 : commande
# - Octets 2 à 7 : data (6 octets)
# - Octet 8 : checksum (0 pour l’instant)
#
# L’émission est totalement indépendante de la réception.
#
def tx_thread():
    while True:
        # Exemple d'envoi : commande PING (0x01)
        cmd  = 0x01
        data = b"\x00\x00\x00\x00\x00\x00"  # 6 octets de data

        # Checksum ignoré → on met 0
        chk = 0x00

        # Construction de la trame de 8 octets
        packet = bytes([cmd]) + data + bytes([chk])

        # Envoi sur l’UART
        ser.write(packet)
        print("Envoyé :", packet)

        time.sleep(2)


# ---------------------------------------------------------
#  LANCEMENT DES THREADS RX/TX
# ---------------------------------------------------------

t1 = threading.Thread(target=rx_thread, daemon=True)
t2 = threading.Thread(target=tx_thread, daemon=True)

t1.start()
t2.start()

print("UART USB prêt. RX et TX totalement indépendants.")

# Boucle principale vide
while True:
    time.sleep(1)
