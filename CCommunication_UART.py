import serial
import threading
import time
import struct   #nico1
import serial.tools.list_ports
from CEnregistrement import CEnregistreurCovaciel  #nico

from CSignal_XBEE import CSignal
from CCerveauVoiture import CCerveau

class CCommunication:
    def __init__(self, port="/dev/carte", baud=9600, timeout=0.1, recorder=None):   #nico1
        self.PORT = port
        self.BAUD = baud
        self.timeout = timeout
        self.ser = serial.Serial(self.PORT, self.BAUD, timeout=self.timeout)
        self.rx_thread = None
        self.running = False
        self.recorder = recorder   #nico1 : instance PARTAGEE injectee (plus de doublon de session)

        # --- Dernieres valeurs UART parsees (remplies par le thread RX, lues par le Cerveau) --- #nico1
        self.lock_data = threading.Lock()   #nico1 : protege l'acces concurrent aux valeurs
        self.derniere_vitesse = 0.0          #nico1 : maintien de la derniere valeur connue
        self.derniere_batterie = 0.0         #nico1
        # Codes de REPONSE qui transportent un float IEEE754 (4 octets). #nico1
        # ATTENTION : a confirmer avec le firmware (code de reponse + offset/endianness). #nico1
        self.REPONSES_FLOAT = {              #nico1
            0x8A: "vitesse",                 #nico1 : reponse VITESSE (GET_VITESSE/IMU)
            0x89: "batterie",                #nico1 : reponse BATTERIE (GET_BATTERIE 0x09)
        }

        self.CMD_EMISSION = {
            0x01: "PING", 0x02: "GET_DEBUG", 0x03: "GET_STATUS",
            0x04: "GET_MOTEUR", 0x05: "SET_MOTEUR", 0x06: "GET_IMU",
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
            err_code = payload[0] if len(payload) >= 1 else 0xFF   #nico1 : garde-fou
            err_name = self.CODE_ERROR.get(err_code, "ERREUR_INCONNUE")
            print(f"!!! {err_name} (0x{err_code:02X})")
            return

        # --- Parsing des valeurs flottantes (vitesse, batterie) --- #nico1
        # Le firmware C++ (FreeRTOS) envoie ces valeurs en float natif (IEEE754, 4 octets).
        # On les decode avec struct ('<f' = little-endian) et on les stocke de facon thread-safe.
        if cmd in self.REPONSES_FLOAT and len(payload) >= 4:   #nico1
            try:                                               #nico1
                valeur = struct.unpack('<f', payload[0:4])[0]  #nico1
            except struct.error:                               #nico1
                return                                         #nico1
            # On ignore les trames corrompues (NaN / inf) #nico1
            if valeur != valeur or valeur in (float('inf'), float('-inf')):  #nico1
                return                                         #nico1
            with self.lock_data:                               #nico1
                if self.REPONSES_FLOAT[cmd] == "vitesse":      #nico1
                    self.derniere_vitesse = valeur             #nico1
                elif self.REPONSES_FLOAT[cmd] == "batterie":   #nico1
                    self.derniere_batterie = valeur            #nico1
            return                                             #nico1

        if cmd in self.CMD_RECEPTION:
            name = self.CMD_RECEPTION[cmd]
            # print desactive : floodait stdout a chaque message UART -> stallait la boucle
            # de nav -> debordement du buffer LIDAR. Reactiver pour debug ponctuel seulement.
            # print(f"RX: {name} | {payload.hex()}")
            pass
        #self.recorder.sauvegarder_trame(cmd, payload) #nico

    def get_vitesse(self):       #nico1
        """ Derniere vitesse (float) recue de l'UART, thread-safe. """  #nico1
        with self.lock_data:     #nico1
            return self.derniere_vitesse   #nico1

    def get_batterie(self):      #nico1
        """ Derniere tension batterie (float) recue de l'UART, thread-safe. """  #nico1
        with self.lock_data:     #nico1
            return self.derniere_batterie  #nico1

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
            # On enregistre l'ordre exact que le Pi envoie � la voiture ! nico
            #if hasattr(self, 'recorder'): #nico
                #self.recorder.sauvegarder_trame(cmd, data) #nico

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