# 🤖 Proiect de Licență: Robot Autonom CBRN

Acest repository conține workspace-ul ROS 2 pentru proiectul de licență ce vizează dezvoltarea unui robot autonom pentru intervenții în medii CBRN (Chimice, Biologice, Radiologice, Nucleare).

Scopul principal al robotului este să navigheze autonom, să identifice pacienți și să realizeze un triaj medical de bază prin **analiza posturii (pose estimation)** și alte detalii medicale.

## 🎯 Obiectivul Acestui Workspace

Acest workspace (`cbrn_ws`) conține pachetul `sim_env`, al cărui scop este crearea unui **mediu de simulare 3D** în Gazebo. Acest simulator ne permite să testăm și să validăm algoritmii de percepție (modelele de *pose estimation*) înainte de a-i implementa pe un robot fizic.

Mediul de simulare include:
* Un robot simplu, echipat cu o cameră.
* O "lume" (`.world`) care simulează un spațiu închis (o cameră cu pereți).
* Un model 3D al unui pacient, descărcat de pe [human-gazebo](https://github.com/robotology/human-gazebo) și rotit pentru a simula o persoană culcată.

---

## 🛠️ Mediul de Dezvoltare și Dependințe

Acest proiect a fost configurat și testat folosind următorul mediu. Toate problemele de configurare (dependințe lipsă, conflicte de mediu) au fost rezolvate.

* **Sistem de Operare:** Ubuntu 24.04 (rulat prin WSL)
* **Distribuție ROS 2:** ROS 2 Jazzy (folosind instalarea binară din `/opt/ros/jazzy/`)
* **Simulator:** Ignition Gazebo (pachetul `ros-jazzy-ros-gz-sim`)
* **Python 3.12**

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

Interfata Gazebo Sim 8 + Modul AI:

<img width="1595" height="674" alt="interface" src="https://github.com/user-attachments/assets/85b29d41-b44e-41dd-af31-9615fa865b84" />

# Modele AI pentru estimarea pozitiei:
Pentru partea de testare voi folosi modelele: MoveNet, ResNet50, Yolo11_pose, MediaPipePose Landmarker
## Pentru rulare:
### Pas1: Lansarea Simulării (Terminal 1)
Acest pas curăță fișierele vechi, recompilează spațiul de lucru și lansează mediul Gazebo, modelul robotului și interfața de vizualizare RViz.
```
cd ~/cbrn_ws/src/scripts
./compile_act_run.bash
```
**Notă**: Așteptați ca Gazebo și RViz să se încarce complet și ca robotul să apară în mediu înainte de a trece la pasul următor.

### Pas2: Pornirea Controlerului (Terminal 2)
Acest nod acționează ca "creierul" de navigație. El primește coordonatele țintei de la nodul de percepție, comandă motoarele robotului și salvează datele experimentale în fișiere Excel (CSV).

Trebuie să specificați parametrul model_name pentru a eticheta corect fișierul de log generat.
```
ros2 run cbrn_perception robot_controller --ros-args -p model_name:=[nume_model]
```
### Pas3: Activarea Modelului AI (Terminal 3)
Acest pas pornește "ochii" robotului. Rulați scriptul specific modelului pe care doriți să îl evaluați în sesiunea curentă.
```
ros2 run cbrn_perception [nume_nod_detector]
```

## Media Pipe Pose Landmarker

CONFIDENCE_SCORE = 0.2

<img width="800" height="600" alt="MediaPipe_0 2" src="https://github.com/user-attachments/assets/e72459ac-4c6f-4782-b505-5ce7b8bf5682" />

## YOLO11 Pose

CONFIDENCE_SCORE = 0.15

<img width="800" height="600" alt="YOLO_0 15conf" src="https://github.com/user-attachments/assets/b1338fea-6131-4bff-b0c1-e9d61b1cd7ca" />

## MoveNet

CONFIDENCE_SCORE = 0.1

<img width="800" height="600" alt="movenet_conf0 1" src="https://github.com/user-attachments/assets/4921db3c-8689-416e-b46b-d5cb0b580f72" />
