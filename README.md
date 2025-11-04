# Instrukcja Konfiguracji i Uruchomienia Systemu ROS 2.

Github: https://github.com/akaczy/Praca-magisterska

Przydatny link (poradnik ROS2 dla początkujących):
https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy

## 1. Wymagania Wstępne

Do poprawnego działania systemu wymagane jest, aby zarówno na komputerze zewnętrznym (PC), jak i na Raspberry Pi zainstalowany był ten sam system operacyjny Ubuntu 22.04 i dystrybucja ROS2 Humble.
System Operacyjny: Ubuntu 22.04 (Jammy Jellyfish)
Pobieranie Ubuntu Desktop (dla PC):https://releases.ubuntu.com/jammy/
Pobieranie Ubuntu dla Raspberry Pi: https://ubuntu.com/download/raspberry-pi
ROS 2: ROS 2 Humble Hawksbill
Oficjalna instrukcja instalacji: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## 2. Instalacja Dodatkowych Narzędzi

### 2.1. RViz2 (Wizualizator ROS 2)

RViz2 jest narzędziem do wizualizacji 3D, pozwalającym na monitorowanie danych z sensorów, modeli robotów i transformacji.
Instalacja:
W przypadku instalacji pełnego pakietu ros-humble-desktop, RViz2 jest już zainstalowany. W przeciwnym razie, lub jeśli jest potrzebny osobno, można go zainstalować poleceniem:
```bash
sudo apt update
sudo apt install ros-humble-rviz2
````

Uruchomienie:
Należy otworzyć nowy terminal (z aktywnym środowiskiem ROS 2) i wykonać polecenie:

```bash
rviz2
```

### 2.2. Google Cartographer (SLAM)

[https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html)

Instalacja:
Należy zainstalować pakiet główny oraz jego integrację z ROS 2:

```bash
sudo apt update
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

### 2.3. SLAM Toolbox (SLAM)

[https://github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

Instalacja:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

## 3. Uruchomienie Systemu na Robocie (pakiet: robot-magisterka)

Procedura uruchomienia pakietu robot-magisterka (znajdującego się w ścieżce: robot/src/robot-magisterka/).

### 3.1. Komputer Zewnętrzny (PC)

Procedura na komputerze głównym obejmuje budowę i uruchomienie pakietu.

1.Budowanie pakietu:
Otwórz terminal i przejdź do głównego katalogu przestrzeni roboczej (robot/):

```bash
cd /sciezka/do/folderu/robot
```

2.Uruchom proces budowania za pomocą colcon:

```bash
colcon build
```

3. Aktywacja środowiska (source):
   Po zakończeniu budowania, w tym samym katalogu (robot/), należy aktywować środowisko, aby terminal mógł zlokalizować nowe pakiety:

```bash
source install/setup.bash
```

4. Uruchomienie programu głównego:
   W terminalu z aktywnym środowiskiem (po wykonaniu kroku 2), uruchom główny plik launch:

```bash
ros2 launch robot-magisterka launch_robot.launch.py
```

### 3.2. Raspberry Pi (Robot)

Na Raspberry Pi należy uruchomić cztery osobne procesy w oddzielnych terminalach.

A. Uruchomienie skryptu - odometria.

1. Otwórz terminal na Raspberry Pi.
2. Przejdź do katalogu zawierającego skrypt publish_tf4.py.
3. Uruchom skrypt za pomocą interpretera Python 3:

```bash
python3 publish_tf4.py
```

B. Uruchomienie Lidaru (RPLIDAR)
Należy zainstalować sterownik, a następnie uruchomić węzeł.

1. Instalacja sterownika (jednorazowo):
   Otwórz terminal na Raspberry Pi i zainstaluj pakiet rplidar_ros:

```bash
sudo apt update
sudo apt install ros-humble-rplidar-ros
```

2. Uruchomienie węzła Lidaru:
   W nowym terminalu (z aktywnym środowiskiem ROS 2), uruchom węzeł rplidar, wskazując odpowiedni port (/dev/ttyUSB0) i prędkość transmisji:

```bash
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000
```

C. Uruchomienie IMU

1. Otwórz terminal na Raspberry Pi.
2. Przejdź do katalogu zawierającego skrypt ultrasound.py.
3. Uruchom skrypt za pomocą interpretera Python 3:

```bash
python3 ultrasound.py
```

D. Uruchomienie czujników ultradźwiękowych
1.Budowanie pakietu:
Otwórz terminal i przejdź do głównego katalogu motor_control :

```bash
cd /sciezka/do/folderu/motor_control
```

2.Uruchom proces budowania za pomocą colcon:

```bash
colcon build
```

3. Aktywacja środowiska (source):
   Po zakończeniu budowania, w tym samym katalogu (motor_control/), należy aktywować środowisko, aby terminal mógł zlokalizować nowe pakiety:

```bash
source install/setup.bash
```

4. Uruchomienie programu głównego:
   W terminalu z aktywnym środowiskiem (po wykonaniu kroku 2), uruchom główny plik launch:

```bash
ros2 launch motor_control robot_imu_ekf_launch.py
```

## 4. Uruchomienie Symulacji (robot-symulacja)

Ta sekcja opisuje procedurę uruchomienia pakietów symulacyjnych (znajdujących się w ścieżce: robot-symulacja/src/).

1. Budowanie pakietów symulacyjnych:
   Otwórz terminal i przejdź do głównego katalogu przestrzeni roboczej symulacji (robot-symulacja/):

```bash
cd /sciezka/do/folderu/robot-symulacja
```

2. Uruchom proces budowania:

```bash
colcon build
```

3. środowiska (source):
   W tym samym katalogu (robot-symulacja/) aktywuj środowisko:

```bash
source install/setup.bash
```

4, Uruchomienie symulacji:
W terminalu z aktywnym środowiskiem symulacji (po wykonaniu kroku 2), uruchom plik launch:

```bash
ros2 launch robot_symulacja symulacja.py
```

## 5. Uruchomienie Narzędzi Dodatkowych (Teleoperacja i Lokalizacja) (dotyczy zarówno rzeczywistego robota jak i symulacji)

### 5.1. Sterowanie Ręczne (teleop_twist_keyboard)

Teleop_twist_keyboard  to prosty węzeł ROS 2, który przechwytuje naciśnięcia klawiszy w terminalu i publikuje je jako wiadomości geometry_msgs/msg/Twist (określające prędkość liniową i kątową) na standardowy temat /cmd_vel. Jest to podstawowe narzędzie do ręcznego testowania i poruszania robotem.
Instalacja:
Pakiet jest dostępny w repozytoriach ROS 2:

```bash
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
```

Uruchomienie:
Otwórz nowy terminal (z aktywnym środowiskiem ROS 2).
Uruchom węzeł:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Ważne: Aby sterowanie działało, okno terminala, w którym uruchomiono polecenie, musi być aktywne (wybrane). W terminalu wyświetli się instrukcja klawiszy (np. u/i/o do poruszania się).

### 5.2 Wgrywanie i zapisywanie mapy:

Po zbudowaniu mapy za pomocą jednego z pakietów SLAM (takich jak Cartographer lub SLAM Toolbox), mapa ta jest zazwyczaj publikowana na temacie (topic) /map. Aby móc z niej skorzystać w przyszłości do lokalizacji, należy ją zapisać na dysku. Do tego celu służy narzędzie map_saver_cli.
Instalacja:

```bash
sudu apt update
sudo apt install ros-humble-nav2-map-server
```

Uruchomienie (Zapisywanie mapy):

1. Upewnij się, że Twój węzeł SLAM działa i publikuje ukończoną mapę na temacie /map.
2. Otwórz nowy terminal i wykonaj polecenie, podając ścieżkę i nazwę pliku, pod jaką mapa ma zostać zapisana (bez rozszerzenia):

```bash
ros2 run nav2_map_server map_saver_cli -f /home/adam/maps/moja_mapa
```

Polecenie to automatycznie zasubskrybuje temat /map, pobierze dane i zapisze w podanej lokalizacji dwa pliki: nazwa_twojej_mapy.pgm (obraz mapy) oraz nazwa_twojej_mapy.yaml (plik konfiguracyjny mapy).
UWAGA: W celu otrzymania wizualizacji wgranej mapy w progrmaie rviz2, należy:

1. Uruchomić rviz2
2. Ustawić “topic” na /map
3. W zakładce topic map ustawić durability na: transient local

### 5.3. Lokalizacja (Nav2 Bringup)

Localization Bringup to części stosu nawigacyjnego Nav2 odpowiedzialnej wyłącznie za lokalizację. Oznacza to, że system będzie określał pozycję i orientację robota na istniejącej, wcześniej wygenerowanej mapie, bazując na danych z sensorów (np. skanach z Lidaru) i odometrii.
WAŻNE: W celu prawidłowego uruchomienia Nav2 Bringup należy pamiętać aby wyłączyć terminal z aktywnym map_server.
Instalacja (jednorazowo):
Należy zainstalować główny pakiet konfiguracyjny Nav2, który pociągnie za sobą niezbędne zależności.

```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup
```

Uruchomienie:
Do uruchomienia lokalizacji służy dedykowany plik launch z pakietu nav2_bringup. Wymaga on podania ścieżki do pliku mapy (.yaml)

```bash
ros2 launch nav2_bringup localization_launch.py map:=/sciezka/do/twojej/mapy.yaml
```

Po prawidłowym uruchomieniu, w programie rviz przy wykorzystaniu opcji “2D Pose Estiamte” należy znaznaczyć na mapie gdzie w rzeczywistości znajduje się robot.

Uwaga: Aby lokalizacja działała poprawnie, robot musi dostarczać dane odometrii (temat /odom) oraz skany lasera (temat /scan), a także odpowiednie transformacje TF (np. odom -> base_link oraz base_link -> laser_frame).
