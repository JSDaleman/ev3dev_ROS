# 🤖 ev3dev_ROS
Paquete de ROS para control y simulacion de robot ev3 con sistema ev3dev.

**Table of Contents**

[TOCM]

Este paquete consta de 5 paquetes los cuales son:

- 🎛️ **controls:** Este paquete tiene el nodo para hacer control del flujo de datos. Debem implementar controladores y elementos de control de los datos en este paquete.

- ✈️ **ev3_launch_pak:** Este paquete es el que controla los archivos .launch generales con los cuales se lanzan los nodos de cada uno de los paquete y archivos .launch de cada uno simultaneamente.

- 🎮 **gui:** Este paquete tiene la interfaz grafica de controles para el usuario tanto como aplicacion idependiente como plugin para RQT.

- 📨 **mqtt:** Este paquete tiene el nodo de protocolo MQTTel cual tiene tambien procesamiento de los mensajes que se reciben del broker MQTT y envio de mensajes.

- 🚗 **ugv_description:** Este tiene la descripcion del robot para su simulación y caracteristeicas de la escena de simulacion.

## ⚙️ Requerimientos

- Ubuntun 20.04: Se puede usar en una maquina virtual, una imagen de docker, como instalacion nativa o el WSL con Windows 11.
- ROS noetic: Se recomienda usar la intalacion completa.
- Gazebo 11.15.1
- RQT 0.5.3
- RVIZ 1.14.25
- Python 3.8.10

>💡 **Nota**: Con la versión completa de ROS vienen preinstalados Gazebo, RQT y RVIZ. En linux por defecto viene una version de python

## Intalacion de bibliotecas,dependencias y paquetes necesarios

Intalaremos el pip de python para esto se usa el siguente commando 

```sh
sudo apt update
sudo apt install python3-pip
pip --version
``` 

>💡 **Nota**: Si deseas saltarte la instalacion de bibliotecas una por una y asegurarte de tener las versiones con las cuales se creo este paquete puedes usar el comando ```pip3 install -r requirements.txt``` del paquete para instalar todas las bibliotecas.

Para el correcto funcionamiento de todo el paquete se necesitan algunas bibliotecas como:

### Pyqt
Esta bibliteca permite el uso de QT como framework desde python para la creacion de interfaces graficas con esta se creo la aplicacion y el plugin en RQT.

```sh
pip install PyQt5
```

### paho 
Esta biblioteca contine los elementos necesarios para crear la comunicación mqtt con el broker y generar todo el protocolo.

```sh
pip install paho-mqtt
```

### Deppendencias plugin RQT
Nos aseguramos que esten instaladas las dependencias para que funcione el plugin creado en RQT.

```sh
sudo apt-get install ros-noetic-rqt-gui
sudo apt-get install ros-noetic-rqt-gui-py
```

### Paquetes simulacion con gazebo
Nos aseguramos de tener todos los paquetes necesarios para la simulacion con ros.

```sh
sudo apt-get update
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-msgs
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-laser-geometry
sudo apt-get install ros-noetic-tf-conversions ros-noetic-tf2-geometry-msgs
sudo apt-get install ros-noetic-joint-state-controller ros-noetic-effort-controllers
sudo apt-get install ros-noetic-position-controllers ros-noetic-velocity-controllers
sudo apt-get install ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher
sudo apt-get install ros-noetic-joint-state-publisher-gui
```

