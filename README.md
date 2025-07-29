# Exploración basada en fronteras de ambientes 2D

## Instalación

### En primer lugar se clona el repositorio

```
https://github.com/saaaax/Exploraci-n-basada-en-Fronteras.git

```
### Ahora se copila el repositorio

```
colcon build

```
### Se sourcea la terminal

```
source install/setup.bash

```
### Se abre otra terminal en el mismo directorio y se repite lo anterior

### Se lanza el launch en una terminal

```
ros2 launch el7009_diff_drive_robot slam.launch.py

```
### En la otra terminal se lanza el nodo

```
ros2 run el7009_diff_drive_robot explorador.py

```
