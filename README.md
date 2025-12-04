# GUI_OV2SLAM
PowerViz to aplikacja GUI pozwalająca na żywo wizualizować mi.n parametry takie jak pozycja, prędkość, obraz, trajektora czy chmura punktów z $OV^2SLAM$
<img width="1785" height="978" alt="Screenshot from 2025-12-04 20-28-15" src="https://github.com/user-attachments/assets/7b67b4fd-a93a-48b1-973f-b52cd9f9724b" />
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
