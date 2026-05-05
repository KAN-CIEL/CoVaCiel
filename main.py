from CCerveauVoiture import CCerveau
from CCommunication_UART import CCommunication
from CSignal_XBEE import CSignal
from CEnregistrement import EnregistreurCovaciel 

import time
import logging

logging.getLogger('rplidar').setLevel(logging.ERROR)

if __name__ == "__main__":
    Brain = CCerveau()
    com = CCommunication(port="/dev/ttyACM0", baud=9600)
    signal = CSignal()
    recorder = EnregistreurCovaciel()

    original_listen = com._listen
    def listen_and_record():
        # Cette fonction s'execute dans le thread de lecture de tes camarades
        # Elle est appelee automatiquement d�s que l'UART re�oit un message
        original_listen() 
        if hasattr(com, 'derniere_commande'):
            recorder.sauvegarder_trame(com.derniere_commande, com.derniers_datas)
            


    print("Systeme de Telemetrie Connect et Automatique.")
    com.start()
    

    #etat_signal = False
    

    print("Système prêt. En attente du signal de départ ($GO;)...")
    print("--- Systeme de Telemetrie Connece ---")
    while True:

        msg = signal.read_signal()

        #com.gestion_start_and_stop(etat_signal, Brain, signal, com)
        
        if msg == "GO":
            print("Demarrage de la detection...")
            # On passe les objets necessaires pour que le cerveau check le STOP
            Brain.start_detection(signal, com)
            print("Retour en veille. En attente de $GO;")

        

        time.sleep(0.1) # Petite pause pour le CPU

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

