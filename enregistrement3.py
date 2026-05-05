import serial
import time
from CEnregistrement import EnregistreurCovaciel

print("--- DÉMARRAGE DU CERVEAU DE LA VOITURE ---")

# 1. Connexion à la base de données
recorder = EnregistreurCovaciel()

# 2. Connexion au câble UART (La carte de la voiture)
try:
    # '/dev/serial0' est le port matériel standard du Raspberry Pi
    # baudrate=9600 est la vitesse de base (à vérifier avec ton équipe)
    voiture = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
    print("[OK] Connecté à la voiture via le port UART.")
except Exception as e:
    print(f"[ERREUR] Impossible de se connecter à l'UART : {e}")
    exit()

# 3. Boucle d'écoute
try:
    print("\nEn attente des vraies données de la voiture... (Ctrl+C pour quitter)")
    while True:
        # Est-ce que la carte électronique a envoyé un message ?
        if voiture.in_waiting > 0:
            
            # On lit les octets reçus (ex: on lit 6 octets)
            trame = voiture.read(voiture.in_waiting) 
            
            # Si on a au moins le code de commande et une valeur...
            if len(trame) >= 2:
                code_cmd = trame[0]
                data_bytes = trame[1:]
                
                # --- L'ENVOI VERS TA BASE DE DONNÉES ---
                recorder.sauvegarder_trame(code_cmd, data_bytes)
                
                # Affichage dans le terminal pour prouver que ça marche
                valeur = data_bytes[0] if len(data_bytes) > 0 else 0
                print(f"[VRAIE TRAME REÇUE] Code : {hex(code_cmd)} | Valeur brute : {valeur}")
        
        # Petite pause pour ne pas surcharger le processeur
        time.sleep(0.05) 

except KeyboardInterrupt:
    print("\n[ARRÊT] Fin de la course.")
    recorder.fermer()
    voiture.close()
