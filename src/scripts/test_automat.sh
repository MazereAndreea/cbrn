#!/bin/bash

if [ "$#" -eq 0 ]; then
    echo "❌ Eroare: Nu ai specificat niciun model!"
    exit 1
fi

MODELE=("$@")

echo "🚀 Pornim pipeline-ul LIVE pentru ${#MODELE[@]} modele..."

for MODEL in "${MODELE[@]}"
do
    echo "========================================================"
    echo "⏳ Se încarcă modelul: $MODEL"
    ros2 param set /universal_pose_detector model_config "$MODEL"
    
    # Lăsăm 3 secunde ca YOLOX/HRNet să se descarce în placa video
    sleep 3
    
    echo "📊 Se colectează rapid 5cadre live..."
    # Scriptul Python pornește, ascultă 5 cadre (~0.3 secunde), scrie și se închide singur
    ./colectare_live.py
    
done

echo "========================================================"
echo "🎉 Pipeline finalizat instant! Verifică fișierul: comparatie_modele.csv"