# Joy Base Control
Este paquete de ROS contiene el nodo `teleop_twist_node` encargado de la teleoperación de un controlador de velocidad cartesiano mediante un mando. Se ha diseñado para la teleoperación de una plataforma móvil omnidireccional controlada por dos placas Roboclaw.

## Funcionamiento

Este nodo se subscribe al topic `/Joy` que contiene un mensaje del tipo `sensor_msgs/Joy` con la información sobre el estado del mando. Procesa la información y publica en el topic `/cmd_vel` un mensaje del tipo `Geometry_msg/Twist`.

Para iniciar el movimiento, el operador debe pulsar el botón B del mando y accionar el joystick JL, para comandar velocidad lineal en el plano XY, o el joystick JR, para comandar velocidad angular sobre el eje Z de la plataforma.

## Paquetes necesarios
Se debe tener instalado el paquete `joy` encargado de actuar como driver para el mando. Se encuentra disponible en: http://wiki.ros.org/joy

## Instalación
En la terminal de linux:

```bash
git clone https://github.com/rodri-castro/joy_base_control.git
catkin_make
```

## Uso
```bash
roslaunch joy_base_control teleop_twist.launch
```
Este comando se encarga de iniciar el nodo del driver del mando y el nodo teleop_twist_node que se encarga de publicar el comando de velocidad en el topic. El nodo del controlador se debe lanzar de manera independiente.

## Modificación de los botones
Para modificar la relación entre los ejes/botones y su funcionalidad se debe modificar el archivo joy.config.yaml contenido dentro de la carpeta `/config`. Se recomienda el programa Jstest-gtk para identificar los botones y los ejes.

## Proyecto RAFI
Este paquete se ha desarrollado para enviar comandos de velocidad al controlador Roboclaw de los motores. No contiene el controlador de la base. Solo contiene el nodo encargado de traducir la información del mando a un mensaje `Twist`. El controlador de la base debe ser lanzado por separado. Para lanzar todos los nodos relacionados con la teleoperación y el control de la base, véase el paquete `rafi_launch_files`.


