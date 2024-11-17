
# TFG: Integración de Robot Manipulador en un Sistema Distribuido con ROS 

Este repositorio contiene el código y la configuración necesarios para trabajar con los brazos robóticos Staubli TX-90 y TX2-90 utilizando ROS y el paquete `MoveIt`. Aquí aprenderás a configurar un workspace de ROS, a clonar este repositorio y a ejecutar un ejemplo.

## Requisitos Previos

- **ROS Melodic** instalado en tu sistema.
- Familiaridad básica con ROS.

## 1. Creación del Workspace de ROS

1. Abre una terminal y crea una carpeta para el workspace de ROS (usualmente llamada `catkin_ws` o similar):

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   ```

2. Inicializa el workspace usando `catkin_make`:

   ```bash
   catkin_make
   ```

   Esto creará las carpetas necesarias (`build`, `devel`) dentro del workspace.

3. Configura el entorno para ROS:

   ```bash
   source devel/setup.bash
   ```

   Añade esta línea al archivo `~/.bashrc` para que se ejecute automáticamente cada vez que abras una terminal:

   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## 2. Clonar el Repositorio en el Workspace

Asegúrate de que estás en la carpeta `src` de tu workspace y luego clona este repositorio:

```bash
cd ~/catkin_ws/src
git clone https://github.com/juanantms/TFG.git
```

Esto descargará el contenido de este repositorio en la carpeta `src`.

## 3. Estructura de Carpetas del Repositorio

Este repositorio contiene las siguientes carpetas principales dentro de `src`:

- **staubli_experimental**: Contiene configuraciones experimentales y archivos de soporte para los brazos Staubli TX-90 y TX2-90 en ROS, incluyendo descripciones en URDF y Xacro, configuraciones de simulación en Gazebo y paquetes para la planificación de movimientos con MoveIt!.
- **staubli_val3_driver**: Incluye el driver `staubli_val3_driver` para comunicarse con el controlador CS9 de Staubli, lo cual permite enviar comandos y recibir el estado del brazo robótico.

## 4. Compilar el Workspace

Después de clonar el repositorio, regresa a la carpeta principal de tu workspace y ejecuta `catkin_make` para compilar todo el contenido:

```bash
cd ~/catkin_ws
catkin_make
```

Si todo está correctamente configurado, la compilación debería finalizar sin errores.

## 5. Ejecutar un Ejemplo

Para ejecutar un ejemplo de movimiento con `MoveIt` y el brazo Staubli TX-90, sigue estos pasos:

1. Asegúrate de tener el entorno de ROS cargado:

   ```bash
   source devel/setup.bash
   ```

2. Lanza el archivo de configuración de `MoveIt` para el brazo Staubli TX-90:

   ```bash
   roslaunch staubli_tx90_moveit_config demo.launch
   ```

   Esto abrirá una ventana de `RViz` donde podrás visualizar el robot y planificar movimientos.

3. **Ejecutar un Script de Ejemplo**:

   Puedes ejecutar un script de ejemplo para mover el brazo a través de waypoints. Asegúrate de que el nodo `MoveIt` está en ejecución y abre una nueva terminal. Luego, ejecuta el siguiente comando:

   ```bash
   rosrun staubli_tx90_moveit_config move_robot_real.py
   ```

   Este script mueve el robot a través de varios puntos predefinidos, usando el controlador de `MoveIt`.
