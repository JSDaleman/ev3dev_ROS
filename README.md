# 🤖 ev3dev_ROS
Paquete de ROS para control y simulacion de robot ev3 con sistema ev3dev.


<details>
    <summary>📖 Tabla de Contenidos</summary>

---
- [🤖 ev3dev\_ROS](#-ev3dev_ros)
  - [🗂️ Estructura de paquetes](#️-estructura-de-paquetes)
  - [⚙️ Requerimientos](#️-requerimientos)
  - [📚 Intalacion de bibliotecas,dependencias y paquetes necesarios](#-intalacion-de-bibliotecasdependencias-y-paquetes-necesarios)
    - [📊 Pyqt](#-pyqt)
    - [📡 paho](#-paho)
    - [🔗 Dependencias plugin RQT](#-dependencias-plugin-rqt)
    - [📦 Paquetes simulacion con gazebo](#-paquetes-simulacion-con-gazebo)
  - [📝 Especificaciones de cada paquete](#-especificaciones-de-cada-paquete)
    - [🎛️ controls](#️-controls)
      - [🕹️ controls\_simulation.launch](#️-controls_simulationlaunch)
      - [🎮 controls.launch](#-controlslaunch)
      - [🎚️ controller.py](#️-controllerpy)
      - [🎯 forward\_kinematic.py](#-forward_kinematicpy)
      - [🔄 inverse\_kinematic.py](#-inverse_kinematicpy)
    - [✈️ ev3\_launch\_pkg](#️-ev3_launch_pkg)
      - [🎮🤖🏎️ ev3\_teleop\_simulate.launch](#️-ev3_teleop_simulatelaunch)
      - [🎮🤖 ev3\_teleop.launch](#-ev3_teleoplaunch)
    - [🎮 gui](#-gui)
      - [🎨🧩🖥️🚀 gui\_rqt.launch](#️-gui_rqtlaunch)
      - [🎨🖥️🚀 gui.launch](#️-guilaunch)
      - [🎨🔌🖥️🚀 rqt\_plugin.launch](#️-rqt_pluginlaunch)
      - [🎨🖥️🌄 gui\_diff\_control.perspective](#️-gui_diff_controlperspective)
      - [🔧🎨🖥️🚀 rqt\_control\_gui\_pligin.xml](#️-rqt_control_gui_pliginxml)
      - [📱app.py](#apppy)
      - [🖥️🔌📱 rqt\_app.py](#️-rqt_apppy)
    - [📨 mqtt](#-mqtt)
      - [🗣️👂🏼delegate.py](#️delegatepy)
      - [🌐📡✉️  mqtt\_remote\_method\_calls.py](#️--mqtt_remote_method_callspy)
    - [🚗 ugv\_description](#-ugv_description)
      - [🏗️⚖️ simple\_diff\_robot\_gazebo.xacro](#️️-simple_diff_robot_gazeboxacro)
      - [🤖🩻⚙️ simple\_diff\_robot\_urdf.xacro](#️-simple_diff_robot_urdfxacro)
    - [📌🎯 Ejemplos de funcionamiento](#-ejemplos-de-funcionamiento)
      - [📡🎮🤖 Teleoperación del robot](#-teleoperación-del-robot)
      - [🎮📊🤖 Teleoperación y simulación](#-teleoperación-y-simulación)
      - [🧩🏙️🖥️ Aplicación y plugin de rqt](#️️-aplicación-y-plugin-de-rqt)
    - [🚨🐞💥 Fallas conocidas](#-fallas-conocidas)
</details>

## 🗂️ Estructura de paquetes
Este paquete consta de 5 paquetes los cuales son:

- 🎛️ **controls:** Este paquete tiene el nodo para hacer control del flujo de datos. Debem implementar controladores y elementos de control de los datos en este paquete.

- ✈️ **ev3_launch_pkg:** Este paquete es el que controla los archivos .launch generales con los cuales se lanzan los nodos de cada uno de los paquete y archivos .launch de cada uno simultaneamente.

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

> [!NOTE]
> Con la versión completa de ROS vienen preinstalados Gazebo, RQT y RVIZ. En linux por defecto viene una version de python

## 📚 Intalacion de bibliotecas,dependencias y paquetes necesarios

Intalaremos el pip de python para esto se usa el siguente commando 

```sh
sudo apt update
sudo apt install python3-pip
pip --version
``` 

> [!TIP]
> Si deseas saltarte la instalacion de bibliotecas una por una y asegurarte de tener las versiones con las cuales se creo este paquete puedes usar el comando ```pip3 install -r requirements.txt``` del paquete para instalar todas las bibliotecas.

Para el correcto funcionamiento de todo el paquete se necesitan algunas bibliotecas como:

### 📊 Pyqt
Esta bibliteca permite el uso de QT como framework desde python para la creacion de interfaces graficas con esta se creo la aplicacion y el plugin en RQT.

```sh
pip install PyQt5
```

### 📡 paho 
Esta biblioteca contine los elementos necesarios para crear la comunicación mqtt con el broker y generar todo el protocolo.

```sh
pip install paho-mqtt
```

### 🔗 Dependencias plugin RQT
Nos aseguramos que esten instaladas las dependencias para que funcione el plugin creado en RQT.

```sh
sudo apt-get install ros-noetic-rqt-gui
sudo apt-get install ros-noetic-rqt-gui-py
```

### 📦 Paquetes simulacion con gazebo
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

## 📝 Especificaciones de cada paquete

En cada uno de los archivos para su manejo dinamico con diferentes robots se tiene que el parametro robot_id para dar el número de identidicacion de cada robot con lo cual se crea su nombre para los topicos usados y los topicos mqtt siendo asi el nombre del robot LegoEv3XX donde XX es el número de identificación.

### 🎛️ controls

<details>
    <summary>🚀 launch</summary>

#### 🕹️ controls_simulation.launch

En este se lanza el nodo de forward_kinematic el cual le da a la simulación de gazebo los parametros de velocidades del robot y el nodo de controller para recibir los mensajes de velocidades de ruedas del robot de la interfaz y mandarlas al nodo mqtt.

#### 🎮 controls.launch

Este lanza todos los nodo de control.
</details>

<details>
    <summary>📄 scripts</summary>

#### 🎚️ controller.py
En este nodo se puede hacer control de los valores y parametros ingresados a la interfaz y procesasarlos antes de mandarlos al robot

<table border="1" align="center">
  <tr>
    <th>Subscribe</th>
    <th>Publica</th>
  </tr>
  <tr>
    <td>LegoEv3XX/drive_control</td>
    <td>LegoEv3XX/wheels_vel</td>
  </tr>
  <tr>
    <td>LegoEv3XX/drive_control/vel_control</td>
    <td>LegoEv3XX/command</td>
  </tr>
  <tr>
    <td>LegoEv3XX/command_control</td>
    <td></td>
  </tr>
</table>

#### 🎯 forward_kinematic.py

Usando la cinematica directa del robot permite encontrar el valor de sus valocidades lineal y angular apartir de la velocidad de las ruedas. /odom_kinematic se puede ver la pose del robot y velocidades mientras que en /cmd_vel_kinematic solo las velocidades.

<table border="1" align="center">
  <tr>
    <th>Subscribe</th>
    <th>Publica</th>
  </tr>
  <tr>
    <td>LegoEv3XX/wheels_vel</td>
    <td>LegoEv3XX/odom_kinematic</td>
  </tr>
  <tr>
    <td></td>
    <td>LegoEv3XX/cmd_vel_kinematic</td>
  </tr>

</table>

#### 🔄 inverse_kinematic.py

Usando la cinematica inversa del robot permite encontrar el valor de la velocidad de cada rueda apartir de la velocidad lineal y angular que tiene. 

<table border="1" align="center">
  <tr>
    <th>Subscribe</th>
    <th>Publica</th>
  </tr>
  <tr>
    <td>LegoEv3XX/cmd_vel</td>
    <td>LegoEv3XX/wheels_vel_kinematic</td>
  </tr>
  <tr>
    <td>LegoEv3XX/odom</td>
    <td></td>
  </tr>

</table>

</details>


### ✈️ ev3_launch_pkg

<details>
    <summary>🚀 launch</summary>
Este paquete solo tiene archivos launch para lanzar de forma simultanea todos los nodos para el funcionamiento del paquete.

#### 🎮🤖🏎️ ev3_teleop_simulate.launch
Lanza todos los nodos incluyendo los de simulacion del robot adicionalmente aca se tiene la asignacion el aparametro robot_id.

#### 🎮🤖 ev3_teleop.launch
Lanza los nodos necesarios para teleoperar el robot y asigna el parametro de robot_id.

</details>

### 🎮 gui

<details>
    <summary>🚀 launch</summary>

#### 🎨🧩🖥️🚀 gui_rqt.launch
Lanza el nodo del plugin de rqt en una ventana solo en donde solo se ve la interfaz grafica creada.

#### 🎨🖥️🚀 gui.launch
Lanza el nodo como una aplicacion de PyQT independiende.

#### 🎨🔌🖥️🚀 rqt_plugin.launch
Lanza el nodo de rqt y carga la perspectiva con el plugin de rqt creado.

</details>

<details>
    <summary>🛠️ resource</summary>

#### 🎨🖥️🌄 gui_diff_control.perspective
Es el archivo donde se tiene la perspectiva del plugin para no tener que cargarlo cuando se inicia rqt.

#### 🔧🎨🖥️🚀 rqt_control_gui_pligin.xml
Es el archivo que carga el plugin a rqt para poder usarlo en el junto a detalles como el nombre de presentacion de este en la interfaz y el icono de este.

</details>

<details>
    <summary>📄 scripts</summary>

#### 📱app.py
Crea la ventna principal para el funcionamiento de la interfaz como aplicacion e importa todos los elementos de la GUI.

#### 🖥️🔌📱 rqt_app.py
Carga el plugin para funcionar en la ventana de rqt


</details>

<details>
    <summary>💾 src</summary>

En esta carpeta se tiene todos los elementos de la gui para su fucnionamiento.

- 🎮🔧 **controls:** Estan los archivos para el control de elementos como botones y teclas. Además de el conversor de unidades para pasar de rpm a rad/s. En estos estan los topicos para el control del robot tanto en ```keys.py``` y ```buttons.py```.


<table border="1" align="center">
  <tr>
    <th>Subscribe</th>
    <th>Publica</th>
  </tr>
  <tr>
    <td>LegoEv3XX/gyro_sensor</td>
    <td>LegoEv3XX/drive_control</td>
  </tr>
  <tr>
    <td></td>
    <td>LegoEv3XX/vel_control</td>
  </tr>
  <tr>
    <td></td>
    <td>    LegoEv3XX/command_control</td>
  </tr>
</table>

- 🪄🖌️🎨 **styles:** Esta el archivo que genera los estilos para los elementos de la intefaz como: frame, botones, entradas de texto, etiquetas de texto y layouts.

- 🎆🖼️🏙️ **views:** Estan los archivos que controlan los elementos que se veran como los frames y menus junto a la estructura de estos.

- 🖼️🖥️🔧 **gui.py:** Tiene las clases de creacion de ventana de visualizacion para la aplicacion independiente y widget para el plugin usado en rqt.

- 🔧🧩🛠️ **rqt_plugin.py** Tiene la clase para la definición grafica de los elementos graficos y logicos del plugin.

</details>

### 📨 mqtt

<details>
    <summary>🚀 launch</summary>
Tiene el archivo lanzador para el nodo de mqtt.
</details>

<details>
    <summary>📄 scripts</summary>
Tiene el archivo para crear la conexión con el broker mqtt, la asignacion del delegado de mensajes entrantes que se usa junto a la tranforamacion de los mensajes recibidos por la interfaz a formato jason para su publicacion en el broker. Transformacion de unidades usadas en ros (rad/s) a las usadas por el robot (rpm) en los mensajes mandados.

<table border="1" align="center">
  <tr>
    <th>Subscribe</th>
    <th>Publica</th>
  </tr>
  <tr>
    <td>LegoEv3XX/wheels_vel</td>
    <td>LegoEv3XX/mqtt_message</td>
  </tr>
  <tr>
    <td>LegoEv3XX/command</td>
    <td>LegoEv3XX/gyro_sensor</td>
  </tr>
</table>

</details>


<details>
    <summary>💾 src</summary>

#### 🗣️👂🏼delegate.py
Tiene la clase que se le delegan los mensajes entrantes las funciones de esta clase son los comandos recibidos de los mensajes mqtt y la lista de parametros que son los parametros de cada función.

#### 🌐📡✉️  mqtt_remote_method_calls.py
Tiene los elementos del protocolo mqtt, la creacion de los topicos de suscripcion y publicación en el broker mqtt, comportamientos de conexión, publicacion y suscipción.

</details>

### 🚗 ugv_description
<!--details>
    <summary>🛠️⚙️🧰 config</summary>
</details-->

<details>
    <summary>🚀 launch</summary>
En el archivo de lanzamiento se especifica el lanzamiento de gazebo, del modelo del robot a usar junto a su posición con respecto al marco global, la definicion del mundo usado, lanzamiento de nodo de estado del robot y de riviz con la configuración previamente guardada de este.
</details>

<details>
    <summary>🧵🕸️ meshes</summary>
En esta carpeta se tienen los archivos de las mallas usadas para los modelos visuales del chasis del robot y las ruedas. En la carpeta collada y wavefront estan los archivos en su formato correspondiente (.dae y .obj)

</details>

<details>
    <summary>🔍🧰 rviz</summary>
Se tiene el archivo con la configuración de rviz para no tener que configurarlo manualmente con TF y robot model y el frame de odom.
</details>

<!--details>
    <summary>📄 scripts</summary>
</details-->

<details>
    <summary>🤖🩻🦴 urdf</summary>

#### 🏗️⚖️ simple_diff_robot_gazebo.xacro
En este archivo se define el nombre del robot para gazebo, se dan las propiedades fisicas para los eslabones del robot y se improta el plugin de gazebo para robots diferenciales definiendo fuente de odometria, topicos de odometria y comandos, tranformaciones de las ruedas, ruedas, diametros de ruedas, trocha del robot, torque y aceleración de las ruedas.

#### 🤖🩻⚙️ simple_diff_robot_urdf.xacro
En este archivo se define la configuración del robot, definición visual, de colición e inercia de cada parte del robot, relación entre eslabones y tipos de uniones.
</details>

<details>
    <summary>🌏🖼️🏙️ worlds</summary>
Se tiene el archivo de configuración del mundo del robot como condiciones de iluminación de la escena, modelo del mundo, motor de fisica y propiedades de este. 
</details>

### 📌🎯 Ejemplos de funcionamiento



#### 📡🎮🤖 Teleoperación del robot

<a href="https://www.youtube.com/watch?v=hCdG2yltG18">
  <img src="https://img.youtube.com/vi/hCdG2yltG18/0.jpg" alt="" width="600px">
</a>

#### 🎮📊🤖 Teleoperación y simulación

<a href="https://www.youtube.com/watch?v=J4IgCLKRch4">
  <img src="https://img.youtube.com/vi/J4IgCLKRch4/0.jpg" alt="" width="600px">
</a>

#### 🧩🏙️🖥️ Aplicación y plugin de rqt

<a href="https://www.youtube.com/watch?v=NIEbXVXS-eo">
  <img src="https://img.youtube.com/vi/NIEbXVXS-eo/0.jpg" alt="" width="600px">
</a>


### 🚨🐞💥 Fallas conocidas

<details>
    <summary>🧰🛠️ Problema visualizacion RVIZ</summary>
Si usas un ubuntu nativo o por maquina virtual rviz no deberia presentar problemas de visualización como si  lo hace el wls en windows. Esto se debe a que no se esta usando OpenGL para renderizar o tenga un conflicto con los drivers del computador. Para solucionar esto sigue los siguientes pasos:

1. Verifica tener OpenGL habilitado en caso de no aparecer nada instala lo y reinicia la simulación.
  ```sh
  glxinfo | grep "OpenGL"
  ```

  ```sh
  sudo apt update
  sudo apt install mesa-utils
  glxinfo | grep "OpenGL" #renderer string muestra la funte para renderizar debe aparecer tu tarjeta grafica
  ```
2.  Si aun presentas el problema puede ser una incompatibilidad de drivers dentro del wls para esto prueba forzando el renderizado por cpu.
  ```
  LIBGL_ALWAYS_SOFTWARE=1 rviz
  ```
3. Con eso se abrira una nueva escena en rviz usando el boton "add" agregando "TF" y ""RobotModel" y en fixed frame "odom".
4. Si con el paso anterior se resolvio el problema recomiendo que guardes la configuracion en el ~/.bashrc o ~/.zshrc
  ```sh
  cd ~
  nano ~/.bashrc #  ~/.zshrc en caso de que uses Zsh
  ```
  ```sh
  #Al final del archivo guarda la configuracion
  export LIBGL_ALWAYS_SOFTWARE=1
  ```
  ```sh
  #Aplica los cambios
  source ~/.bashrc  # O ~/.zshrc si usas Zsh
  ```


</details>