import serial

class CSignal:
    def __init__(self, port='/dev/ttyUSB1', baud=9600, timeout=0.1):
        self.PORT = port
        self.BAUD = baud
        self.timeout = timeout
        self.buffer = ""
        self.ser = serial.Serial(self.PORT, self.BAUD, timeout=self.timeout)

    def read_signal(self):
        # Lecture d'un caractère (ou plus selon le timeout)
        data = self.ser.read(self.ser.in_waiting or 1).decode(errors="ignore")
        if not data:
            return None

        self.buffer += data

        if ';' in self.buffer:
            # On sépare le premier message complet du reste
            parts = self.buffer.split(';', 1)
            raw_msg = parts[0].strip()
            self.buffer = parts[1] # On garde le reste dans le buffer

            if raw_msg.startswith('$'):
                return raw_msg[1:] # Retourne "STOP", "GO", etc.
        
        return None