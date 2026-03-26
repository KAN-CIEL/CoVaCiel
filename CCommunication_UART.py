import serial
import threading
import time

from CSignal_XBEE import CSignal
from CCerveauVoiture import CCerveau

class CCommunication:
    def __init__(self, port="/dev/ttyS0", baud=9600, timeout=0.1):
        self.PORT = port
        self.BAUD = baud
        self.timeout = timeout
        self.ser = serial.Serial(self.PORT, self.BAUD, timeout=self.timeout)
        self.rx_thread = None
        self.running = False

        self.CMD_EMISSION = {
            0x01: "PING", 0x02: "GET_DEBUG", 0x03: "GET_STATUS",
            0x04: "GET_MOTEUR", 0x05: "SET_MOTEUR", 0x06: "GET_SERVO",
            0x07: "SET_SERVO", 0x08: "GET_DISTANCE", 0x09: "GET_BATTERIE",
            0x0A: "GET_VITESSE", 0x0F: "RESET_ERREUR"
        }

        self.CMD_RECEPTION = {
            0x81: "PING_REP", 0x82: "DEBUG_REP", 0x83: "STATUS_REP",
            0x84: "MOTEUR_REP", 0x85: "MOTEUR_ACK", 0x86: "SERVO",
            0x87: "SERVO_ACK", 0x88: "DISTANCE", 0x89: "BATTERIE",
            0x8A: "VITESSE", 0xFF: "ERREUR"
        }

        self.CODE_ERROR = {
            0x01: "CRC_INVALIDE",
            0x02: "CMD_INCONNUE",
            0x03: "LEN_INVALIDE",
            0x04: "ETAT_INTERDIT",
            0x05: "HARDWARE_FAIL",
            0x06: "UNKNOW_ERROR"
        }

    def _listen(self):
        buffer = b""
        print("Thread sur ecoute . . .")
        while self.running:
            to_read = 8 - len(buffer)
            if to_read > 0:
                data = self.ser.read(to_read)
                buffer += data

            if len(buffer) == 8:
                cmd = buffer[0]
                payload = buffer[1:7]
                chk_rx = buffer[7]
                chk_calc = (cmd + sum(payload)) % 256
                
                if chk_calc == chk_rx:
                    self._process_packet(cmd, payload)
                
                buffer = b""

    def _process_packet(self, cmd, payload):
        if cmd == 0xFF:
            err_code = payload[0]
            err_name = self.CODE_ERROR.get(err_code, "ERREUR_INCONNUE")
            print(f"!!! {err_name} (0x{err_code:02X})")
        elif cmd in self.CMD_RECEPTION:
            name = self.CMD_RECEPTION[cmd]
            print(f"RX: {name} | {payload.hex()}")

    def start(self):
        if not self.running:
            self.running = True
            self.rx_thread = threading.Thread(target=self._listen, daemon=True)
            self.rx_thread.start()
            print("Thread lance en arriere plan . . .")

    def stop(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        self.ser.close()

    def send_command(self, cmd, data=b'\x00'*6):
        if len(data) != 6:
            return
        chk_tx = (cmd + sum(data)) % 256
        frame = bytes([cmd]) + data + bytes([chk_tx])
        try:
            self.ser.write(frame)
        except:
            pass
    
    def gestion_start_and_stop(self, etat_signal=None, cerveau=None, signal=None, com=None):
        msg = signal.read_signal()
        if msg == "GO":
            #print("Signal reçu : DÉPART")
            self.start()
            etat_signal = True
            cerveau.start_detection(etat_signal, cerveau, signal, com)
            
        elif msg == "STOP":
            #print("Signal reçu : ARRÊT")
            #self.stop()
            return False
        """
        if etat_signal:
            print("Démarrage")
        else:
            # On attend un peu pour ne pas saturer le processeur
            time.sleep(0.1)
            """