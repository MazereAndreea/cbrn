#!/bin/bash

# Lista de modele pe care vrei să le testezi
MODELE=(
    "yoloxpose_s_8xb32-300e_coco-640"
    "rtmo_s_8xb32-600e_coco-640"
    # Adaugă restul modelelor aici...
)

NUME_ROBOT="numele_robotului_tau" # <-- SCHIMBĂ AICI cu numele exact al robotului din Gazebo

echo "🚀 Pornim pipeline-ul de benchmarking pentru distanță..."

for MODEL in "${MODELE[@]}"
do
    echo "========================================================"
    echo "🔄 Resetare poziție robot la 100m distanță..."
    
    # Comandă Gazebo Classic pentru teleportare la X=-100, Y=0 (presupunând omul la 0,0)
    gz service -s /world/cbrn_world/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "name: 'cbrn_robot',
          position: {x: -5.0, y: 0.0, z: 0.35},
          orientation: {x: 0, y: 0, z: 0, w: 1}"
    
    # NOTĂ: Dacă folosești noul Gazebo (Ignition/Harmonic), înlocuiește linia de mai sus cu linia de mai jos:
    # gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --resptype gz.msgs.Boolean --timeout 2000 --req "name: '$NUME_ROBOT', position: {x: -100.0, y: 0.0, z: 0.0}" > /dev/null

    sleep 1 # Lăsăm fizica din Gazebo să se stabilizeze după teleportare
    
    echo "⏳ Încărcare model AI: $MODEL"
    ros2 param set /universal_pose_detector model_config "$MODEL"
    
    sleep 3 # Timp pentru alocarea memoriei GPU (VRAM)
    
    echo "🚗 Robotul pornește la drum. Se așteaptă prima detecție..."
    # Pornim nodul de control; scriptul se blochează aici până când robotul detectează omul și se oprește
    ros2 run cbrn_perception colector_distanta
    
    echo "✔️ Test finalizat pentru $MODEL."
done

echo "========================================================"
echo "🎉 Pipeline complet! Rezultatele se află în: comparatie_modele.csv"