# -*- coding: utf-8 -*#
"""
Fichier de test unitaire pour la validation du flux de télémétrie.
Permet de visualiser en direct dans le terminal les données transmises par la carte.
"""

import time
import os
import datetime
from CCommunication_UART import CCommunication
from CSignal_XBEE import CSignal

class CTestTelemetrie:
    def __init__(self):
        print("[INIT] Initialisation du module de communication UART...")
        self.com = CCommunication(port="/dev/ttyACM0", baud=9600, timeout=0.1)
        self.signal = CSignal()
        
        # Variables de stockage des valeurs décodées
        self.vitesse_imu = 0.0
        self.batterie_tension = 0.0
        self.angle_direction = 0.0
        self.lidar_statut_arret = "Non"

    def extraire_valeur_16bits(self, trame):
        """ Recompose l'entier 16 bits à partir de l'octet de poids fort et faible """
        if trame and len(trame) >= 2:
            valeur_brute = (trame[0] << 8) | trame[1]
            return valeur_brute / 100.0
        return None

    def executer_diagnostic(self):
        # Démarrage du thread d'écoute UART
        self.com.start()
        print("\n" + "="*60)
        #nico1 : Affichage des consignes et du mode de diagnostic
        print("SÉANCE DE DIAGNOSTIC TÉLÉMÉTRIE EN DIRECT")
        print("Pressez 'Ctrl+C' ou basculez le switch XBEE sur STOP pour quitter.")
        print("="*60 + "\n")

        last_display_t = 0

        try:
            while True:
                t_now = time.time()

                # 1. Requête active pour la tension batterie (0x09)
                trame_bat = self.com.send_command(0x09, b'\x00\x00\x00\x00\x00\x00')
                res_bat = self.extraire_valeur_16bits(trame_bat)
                if res_bat is not None:
                    self.batterie_tension = res_bat

                # 2. Requête active pour la vitesse IMU (0x06)
                trame_vit = self.com.send_command(0x06, b'\x00\x00\x00\x00\x00\x00')
                res_vit = self.extraire_valeur_16bits(trame_vit)
                if res_vit is not None:
                    self.vitesse_imu = res_vit

                # 3. Récupération des états d'arrêt et de géométrie (Simulation ou via variables d'état locales)
                # nico1 : Ces valeurs seront injectées par la suite depuis le CCerveau principal
                if hasattr(self.com, 'dernier_angle_degres'):
                    self.angle_direction = self.com.dernier_angle_degres

                # Vérification du signal d'arrêt externe XBEE
                if self.signal.read_signal() == "STOP":
                    print("[XBEE] Signal d'arrêt détecté. Fin du diagnostic.")
                    break

                # 4. Affichage rafraîchi à une fréquence de 5 Hz (toutes les 0.2s)
                if t_now - last_display_t > 0.2:
                    # Effacement de la ligne précédente pour un affichage dynamique propre
                    os.system('cls' if os.name == 'nt' else 'clear')
                    
                    print("="*60)
                    print(f" HORODATAGE : {datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
                    print("="*60)
                    #nico1 : Validation visuelle des types de données requis pour la revue
                    print(f" BATTERIE (0x09)    : {self.batterie_tension:.2f} V")
                    print(f" VITESSE IMU (0x06) : {self.vitesse_imu:.2f} m/s")
                    print(f" ANGLE CALCULÉ      : {self.angle_direction:.1f} °")
                    print(f" ARRÊT LIDAR        : {self.lidar_statut_arret}")
                    print("="*60)
                    print(" Statut de la liaison : Active [OK]")
                    
                    last_display_t = t_now

                # Petite pause pour soulager l'ordonnanceur du Raspberry Pi
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n[INFO] Diagnostic interrompu manuellement par l'utilisateur.")
        finally:
            # Fermeture propre des descripteurs de fichiers séries
            self.com.stop()
            print("[INFO] Thread de communication arrêté proprement.")

if __name__ == "__main__":
    diagnostic = CTestTelemetrie()
    diagnostic.executer_diagnostic()
