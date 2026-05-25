# 🤖 Proiect de Licență: Robot Autonom CBRN

Acest repository conține workspace-ul ROS 2 pentru proiectul de licență ce vizează dezvoltarea unui robot autonom pentru intervenții în medii CBRN (Chimice, Biologice, Radiologice, Nucleare).

Scopul principal al robotului este să navigheze autonom, să identifice pacienți și să realizeze un triaj medical de bază prin **analiza posturii (pose estimation)** și alte detalii medicale.

## 🎯 Obiectivul Acestui Workspace

Acest workspace (`cbrn_ws`) conține pachetul `sim_env`, al cărui scop este crearea unui **mediu de simulare 3D** în Gazebo. Acest simulator ne permite să testăm și să validăm algoritmii de percepție (modelele de *pose estimation*) înainte de a-i implementa pe un robot fizic.

Mediul de simulare include:
* Un robot simplu, echipat cu o cameră.
* O "lume" (`.world`) care simulează un spațiu închis (o cameră cu pereți).
* Un model 3D al unui pacient, descărcat de pe [human-gazebo](https://github.com/robotology/human-gazebo) și rotit pentru a simula o persoană culcată.

```text
cbrn/
├── src/
│   ├── sim_env/                  # Pachetul principal de simulare (Gazebo)
│   │   ├── launch/               # Scripturi de lansare a lumii și a robotului
│   │   ├── models/               # Fișierele 3D ale mediului (ex. modelul human-gazebo)
│   │   ├── urdf/                 # Descrierea cinematică și senzorii robotului (Xacro)
│   │   └── worlds/               # Scenariile de test (ex. cbrn_world.world)
│   │
│   ├── cbrn_interfaces/          # Pachet pentru mesaje customizate
│   │   └── msg/
│   │       └── PerceptionMetrics.msg  # Structura datelor analitice
│   │
│   └── cbrn_perception/          # Pachetul de Inteligență Artificială (MMPose)
│       ├── mmpose_detector.py    # Nodul principal de detecție 2D
│       ├── colector_live.py      # Nod pentru navigație și logging (Benchmarking)
│       └── pipeline_benchmarking.sh # Automatizarea testelor pe 170+ modele
│
├── build/                        # Fisiere binare generate (ignorat in git)
├── install/                      # Fisiere executabile și scripturi setup (ignorat in git)
└── log/                          # Jurnalele de execuție ROS 2
```
## 🛠️ Mediul de Dezvoltare și Dependințe

Acest proiect a fost configurat și testat folosind următorul mediu. Toate problemele de configurare (dependințe lipsă, conflicte de mediu) au fost rezolvate.

* **Sistem de Operare:** Ubuntu 24.04 (rulat prin WSL)
* **Distribuție ROS 2:** ROS 2 Jazzy (folosind instalarea binară din `/opt/ros/jazzy/`)
* **Simulator:** Ignition Gazebo (pachetul `ros-jazzy-ros-gz-sim`)

### Dependințe Cheie

Pentru a compila și rula acest pachet, asigurați-vă că aveți instalate pachetele de bază ROS 2 și pachetele specifice Gazebo:

```bash
# Instalează pachetele de bază
sudo apt install ros-jazzy-desktop

# Instalează pachetele specifice pentru simularea cu Gz (Gazebo)
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge

# Instalează Xacro (pentru procesarea fișierelor robotului)
sudo apt install ros-jazzy-xacro
```
## 🚀 Instalare și Rulare
### Compilarea Workspace-ului, activarea mediului de lucru și rulare
Comanda (`colcon build`) va citi fișierul (`sim_env/CMakeLists.txt`) și va copia automat fișierele (`launch`), (`urdf`), (`worlds`) și (`models`) în folderul (`install`).

```bash
# Mergi la rădăcina workspace-ului
cd ~/cbrn_ws/src/scripts

# Folosește scriptul compile_act_run.bash
./compile_act_run
```

Interfata Gazebo Sim 8 + Rviz2 pentru vizualizarea lumii (`cbrn_world`) in Gazebo și perspectiva camerei in (`Rviz2`):

<img width="1920" height="875" alt="gazebo_rviz" src="https://github.com/user-attachments/assets/465f3837-a714-42bc-a3f4-38670ed6a12c" />

# 1. Navighează în folderul proiectului
cd ~/cbrn/cbrn

# 2. Activează mediul virtual
source ~/venvs/python_3.12/bin/activate

# 3. ȘTERGE compilarea veche (acest pas este obligatoriu pentru a reseta căile)
rm -rf build install log

# 4. Recompilează pachetul având mediul virtual activat
colcon build

# 5. Încarcă mediul ROS 2 principal și noul build
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 6. Rulează nodul
ros2 run cbrn_perception mmpose_detector

# TERMINAL 2:
source ~/cbrn/cbrn/install/setup.bash
./test_automat.sh [NUME MODEL]

# Universal Pose Detector - Perception Metrics

Acest pachet conține un nod ROS 2 pentru estimarea posturii umane, integrând modele din ecosistemul OpenMMLab (MMPose). Nodul procesează imagini live, detectează punctele cheie ale corpului uman (keypoints) și publică metrice detaliate de performanță pentru evaluarea algoritmilor de inteligență artificială.

## Interfața de Date: `PerceptionMetrics.msg`

Nodul publică date analitice pe topicul `/pose_estimation/metrics` folosind o structură personalizată de mesaj. Mai jos este documentată metodologia de calcul pentru fiecare câmp al mesajului.

### Structura Mesajului
- `std_msgs/Header header`
- `string model_name`
- `float64 inference_time`
- `float64 confidence_score`
- `bool is_detected`
- `float64 distance_estimate`

### Detalii de Calcul

**1. header**
Sincronizează timestamp-ul metricilor cu cel al imaginii originale (secunde și nanosecunde). Permite corelarea exactă a performanței cu cadrul video procesat.

**2. model_name**
Stochează identificatorul modelului curent rulat de placa video (ex: `yoloxpose_s_8xb32-300e_coco-640`). Valoarea este injectată dinamic la fiecare cadru, fiind citită din parametrul ROS 2 `model_config`.

**3. inference_time**
Măsoară durata pură a procesării feed-forward prin rețeaua neurală.
- Este calculat folosind `time.perf_counter()` din Python.
- Cronometrul este pornit chiar înainte ca imaginea să fie trimisă către `MMPoseInferencer` și oprit imediat după primirea predicțiilor.
- Exclude timpii de randare grafică a scheletului (OpenCV) sau latențele rețelei. Rezultatul este exprimat în secunde.

**4. confidence_score**
Reprezintă gradul de siguranță al modelului asupra detecției curente (valoare între 0.0 și 1.0).
- MMPose generează un scor de confidență individual pentru fiecare punct cheie detectat (nas, umăr stâng, gleznă dreaptă etc.).
- Scorul final publicat în mesaj este media aritmetică (`numpy.mean`) a tuturor acestor scoruri individuale pentru persoana detectată.

**5. is_detected**
Un indicator boolean rapid. Returnează `True` dacă algoritmul a identificat cel puțin un schelet uman valid în cadru și `False` dacă imaginea nu conține persoane detectabile (sau dacă scorul general cade sub pragul intern al modelului).

**6. distance_estimate**
*Notă de implementare:* Câmp rezervat pentru viitoarele integrări cu senzori de adâncime (ex: camere RGB-D) sau pentru algoritmi de estimare monoculară bazată pe calibrare. Momentan, sistemul returnează valoarea implicită `0.0`.

Metoda PInhole:
Codul adună toate articulațiile detectate și vede care este cea mai de sus și cea mai de jos, formând un fel de "Bounding Box" flexibil.
D = f * H / h
f = Distanța focală a camerei în pixeli (extrasă live de pe topicul /camera/camera_info).
H = Înălțimea reală presupusă a persoanei (1.70 metri).h = Înălțimea persoanei pe ecran, în pixeli.

Nota: Robotul transmite distanta din momentul detectarii nasului persoanei (key[0], cel mai inalt punct)
