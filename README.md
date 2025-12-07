# GUI_OV2SLAM
PowerViz to aplikacja GUI pozwalająca na żywo wizualizować mi.n parametry takie jak pozycja, prędkość, obraz, trajektora czy chmura punktów z $OV^2SLAM$
<img width="1786" height="977" alt="Screenshot from 2025-12-07 16-40-43" src="https://github.com/user-attachments/assets/1da78d87-56bf-4f75-b3cb-b1d9bd3a27a8" />
## Uruchomienie
Na początku pobieramy repozytorium
```bash
git clone https://github.com/PUT-POWERTRAIN/GUI_OV2SLAM.git
```
Następnie wchodzimy w folder repozytorium
```bash
cd GUI_OV2SLAM
```
Budujemy dockera
```bash
docker build -t imgui_ros2_gui:latest .
```
Następnie musimy dać dockerowi uprawnienia do wyświetlania okien na ekranie
```bash
xhost +local:docker
```
I uruchamiamy dockera tą komendą
```bash
docker run -it --rm \
  --name imgui_gui \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  imgui_ros2_gui:latest
```
Będąc w dockerze musimy zsourcować środowisko
```bash
source /ws/install/setup.bash
```
Ostatecznie gdy chcemy uruchomić aplikację włączamy plik launch
```bash
ros2 launch imgui_app visualizer.launch.py
```
