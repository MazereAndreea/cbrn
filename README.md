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
### Compilarea Workspace-ului
Comanda (`colcon build`) va citi fișierul (`sim_env/CMakeLists.txt`) și va copia automat fișierele (`launch`), (`urdf`), (`worlds`) și (`models`) în folderul (`install`).

```bash
# Mergi la rădăcina workspace-ului
cd ~/cbrn_ws

# Compilează (pentru dezvoltare)
colcon build --symlink-install 
source install/setup.bash

```
## Terminal 1 (Gazebo simulation):

```bash
# Pornește gazebo headless -s (fara fereastra 3D), -v 4 (logging detaliat)
gz sim -s -v 4 empty.sdf
```

## Terminal 2 (Creare model humanSubject01):

```bash
# Surseaza ROS2
source ~/cbrn_ws/install/setup.bash

# Ruleaza pentru a crea modelul in lumea gazebo
ros2 run ros_gz_sim create \
  -file ~/cbrn_ws/src/human-gazebo/humanSubject01/humanSubject01_66dof_colored.urdf \
  -name humanSubject \
  -x 0 -y 0 -z 1

```
## Verifica ca modelul exista:

```bash
ros2 topic list

ros2 topic echo /world/empty/model/humanSubject/pose

```
## Deschide RViz2 pentru vizualizare 3D

```bash
rviz2
```
