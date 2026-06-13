from rplidar import RPLidar, RPLidarException
from CGestion_Lidar import CGestion
import time
import threading

class CDetection:
    def __init__(self):

        self.lidar = None
        self.PORT = '/dev/lidar'   # symlink fixe (udev) -> CP2102 du RPLidar
        self.BAUDERATE = 256000
        self.timeout = 3

        # --- Thread lecteur LIDAR ---
        # Un thread dedie lit le LIDAR EN CONTINU et garde le dernier scan. Comme il ne
        # fait QUE lire, il draine le buffer serie en permanence -> plus de debordement
        # (donc plus de 'Check bit' / 'flags mismatch' dus au vidage du buffer). La boucle
        # de navigation consomme le dernier scan a son rythme, sans bloquer le LIDAR.
        self._latest_scan = None
        self._scan_seq = 0                 # incremente a chaque nouveau scan (detection de fraicheur)
        self._scan_lock = threading.Lock()
        self._reader_running = False
        self._reader_thread = None

    def start_lidar(self):
        try:
            self.lidar = RPLidar(self.PORT, self.BAUDERATE, self.timeout)
            # Remise a l'etat repos pour eviter "Descriptor length mismatch" :
            # un run precedent a pu laisser le LIDAR en mode scan -> on arrete le
            # scan, le moteur, et on vide le buffer serie residuel avant de relancer.
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.clear_input()
            time.sleep(0.5)
            print("LIDAR demarre")
            return True
        except Exception as e:
            print(f"Erreur de connexion au LIDAR: {e}")
            return False

    def start_reader(self, max_buf_meas=8000):
        """ Demarre le thread lecteur LIDAR (draine le buffer en continu). """
        if self._reader_running:
            return
        with self._scan_lock:
            self._latest_scan = None   # pas de scan perime d'une course precedente
        self._reader_running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, args=(max_buf_meas,), daemon=True)
        self._reader_thread.start()

    def _reader_loop(self, max_buf_meas):
        """ Boucle du thread lecteur : lit les scans en continu et stocke le dernier.
            Comme ce thread ne fait QUE lire, le buffer serie ne deborde plus. Sur paquet
            corrompu, on resynchronise le flux (moteur maintenu) sans toucher a la nav. """
        while self._reader_running:
            try:
                for scan in self.lidar.iter_scans(max_buf_meas=max_buf_meas):
                    if not self._reader_running:
                        break
                    with self._scan_lock:
                        self._latest_scan = scan
                        self._scan_seq += 1
            except RPLidarException as e:
                # Desync (buffer/corruption) : on resync le flux, moteur maintenu.
                print(f"LIDAR resync ({e})")
                try:
                    self.lidar.stop()
                    self.lidar.clear_input()
                    time.sleep(0.05)
                except Exception:
                    pass
            except Exception as e:
                # Erreur plus grave (port ferme...) : on sort proprement.
                if self._reader_running:
                    print(f"LIDAR lecteur arrete : {e}")
                break

    def get_latest_scan(self):
        """ Renvoie (numero_de_sequence, dernier_scan). Le numero change a chaque nouveau
            scan -> la nav sait si le scan est NEUF ou deja traite (thread-safe). """
        with self._scan_lock:
            return self._scan_seq, self._latest_scan

    def stop_reader(self):
        """ Arrete le thread lecteur (avant de couper le LIDAR). """
        self._reader_running = False
        t = self._reader_thread
        if t is not None:
            t.join(timeout=1.0)
        self._reader_thread = None

    def stop_lidar(self):
        # On v?rifie si l'objet lidar existe
        if self.lidar:
            try:
                # On arr?te les scans et le moteur
                self.lidar.stop()
                self.lidar.stop_motor()
                # On ferme la connexion s?rie proprement
                self.lidar.disconnect()
                print("LIDAR arr?t? proprement.")
            except Exception as e:
                print(f"Note: Erreur lors de la fermeture : {e}")
            finally:
                self.lidar = None # On remet ? None pour ?viter de boucler
        else:
            print("LIDAR d?j? arr?t? ou non d?marr?.")
    
    def gerer(self, scan):
        gestion = CGestion()
        gestion.scan = scan

        gestion.trier_par_angle()
        gestion.filtrer_distance()
        gestion.filtrer_qualite()
        gestion.filtrer_angle()

        obstacle_proche = gestion.get_obstacle_proche()
        return obstacle_proche