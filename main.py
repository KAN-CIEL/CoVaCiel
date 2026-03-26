from CCerveauVoiture import CCerveau
from CCommunication_UART import CCommunication
from CSignal_XBEE import CSignal

import time

if __name__ == "__main__":
    Brain = CCerveau()
    com = CCommunication()
    signal = CSignal()

    com.start()
    

    etat_signal = False

    print("Système prêt. En attente du signal de départ ($GO;)...")

    while True:

        #msg = signal.read_signal()

        com.gestion_start_and_stop(etat_signal, Brain, signal, com)
        

        """
        if msg == "GO":
            print("Signal reçu : DÉPART")
            etat_signal = True
            Brain.start_detection()
        elif msg == "STOP":
            print("Signal reçu : ARRÊT")
            etat_signal = False
            Brain.stop_detection()

        if etat_signal:
            print("Démarrage")
        else:
            # On attend un peu pour ne pas saturer le processeur
            time.sleep(0.1)
    """

