#!/bin/bash

cleanup() {
    echo ""
    echo "[SYSTEME] ArrÃªt demandÃ©. Fermeture de tous les services..."
    # On tue tous les processus d'un coup
    killall -9 mediamtx rpicam-vid ffmpeg python3 > /dev/null 2>&1
    echo "[SYSTEME] Tout est Ã©teint. Bonne journÃ©e !"
    exit 0
}
trap cleanup SIGINT

echo "=========================================="
echo " LANCEMENT DU SYSTEME COMPLET COVACIEL "
echo "=========================================="

killall -9 mediamtx rpicam-vid ffmpeg python3 > /dev/null 2>&1

echo "[1/4] DÃ©marrage du serveur MediaMTX..."
./mediamtx > /dev/null 2>&1 &
sleep 2

echo "[2/4] Allumage de la CamÃ©ra (10 FPS)..."
rpicam-vid -t 0 -n --inline --width 640 --height 480 --framerate 10 --codec h264 --profile baseline --bitrate 500000 -o - | ffmpeg -f h264 -i - -c copy -f rtsp rtsp://127.0.0.1:8554/cam > /dev/null 2>&1 &
sleep 3

# On lance l'IA en arriÃ¨re-plan (elle va crÃ©er le vision_data.json en silence)
echo "[3/4] Lancement de l'Intelligence Artificielle (CamÃ©ra)..."
python3 detection_couleur.py > /dev/null 2>&1 &
sleep 2

# Et enfin, on lance ton script principal au premier plan pour voir les logs SQL !
echo "[4/4] Lancement de l'Enregistrement Global (I2C + IA -> MariaDB)..."
python3 CEnregistrement.py
