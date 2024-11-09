#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

def move_robot_to_position(x, y, z, roll, pitch, yaw):
    # Inicializar MoveIt! Commander y el nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_to_position', anonymous=True)

    # Crear objetos para manejar el robot y el grupo de movimientos del brazo
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")

    # Establecer el tiempo máximo para planificar una trayectoria
    group.set_planning_time(60)
    group.set_max_velocity_scaling_factor(0.5)  # Escala de velocidad para hardware real
    group.set_max_acceleration_scaling_factor(0.5)  # Escala de aceleración

    # Obtener la posición y orientación actuales del robot como punto de partida
    start_pose = group.get_current_pose().pose

    # Definir la posición y orientación de destino según las coordenadas y ángulos dados
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    # Convertir los ángulos de Euler a una orientación en cuaterniones
    q = quaternion_from_euler(roll, pitch, yaw)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    # Crear una lista de waypoints (solo con el objetivo final en este caso)
    waypoints = [start_pose, target_pose]
    rospy.loginfo("Moviéndose a la posición: x=%.3f, y=%.3f, z=%.3f" % (x, y, z))

    # Planificar la trayectoria cartesiana
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        0.02,
        0.0,
        avoid_collisions=True)

    # Comprobar si la planificación fue completa (fraction == 1.0)
    if fraction == 1.0:
        rospy.loginfo("Trayectoria calculada correctamente. Ejecutando...")

        # Aplicar parametrización temporal para asegurar tiempos crecientes entre puntos de trayectoria
        for i, point in enumerate(plan.joint_trajectory.points):
            point.time_from_start = rospy.Duration(0.1 * i)  # Ajusta el tiempo para hardware real

        # Ejecutar la trayectoria planificada en el robot real
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("La trayectoria no se pudo calcular completamente: solo un %.2f%% se pudo planificar." % (fraction * 100))

    # Finalizar movimiento y cerrar node
    group.stop()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    if len(sys.argv) != 7:
        rospy.logerr("Uso: move_robot_to_position.py <x> <y> <z> <roll> <pitch> <yaw>")
    else:
        try:
            # Obtener las coordenadas y ángulos de los argumentos
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
            roll = float(sys.argv[4])
            pitch = float(sys.argv[5])
            yaw = float(sys.argv[6])

            # Llamar a la función con los parámetros proporcionados
            move_robot_to_position(x, y, z, roll, pitch, yaw)
        except rospy.ROSInterruptException:
            pass
