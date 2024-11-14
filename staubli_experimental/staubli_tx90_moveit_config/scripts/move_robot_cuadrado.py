#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

def move_robot():
    # Inicializar MoveIt! Commander y el nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot', anonymous=True)

    # Crear objetos para manejar el robot y el grupo de movimientos del brazo
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")

    # Establecer el tiempo máximo que se permitirá para planificar una trayectoria
    group.set_planning_time(60)

    group.set_max_velocity_scaling_factor(0.1)  # Escala de velocidad para hardware real
    group.set_max_acceleration_scaling_factor(0.1)

    start_pose = group.get_current_pose().pose
   
    waypoints = []

    # Agregar el punto inicial (A) a la lista de waypoints
    waypoints.append(start_pose)
    rospy.loginfo("Posición A: x=%.3f, y=%.3f, z=%.3f" % (start_pose.position.x, start_pose.position.y, start_pose.position.z))

    # Definir los puntos del cuadrado con distancias de 0.1 en x y y
    # Posición B: (x+0.1, y, z)
    target_pose_1 = geometry_msgs.msg.Pose()
    target_pose_1.position.x = start_pose.position.x + 0.1
    target_pose_1.position.y = start_pose.position.y
    target_pose_1.position.z = start_pose.position.z  # Mantener la altura z = z
    target_pose_1.orientation = start_pose.orientation
    waypoints.append(target_pose_1)
    rospy.loginfo("Posición B: x=%.3f, y=%.3f, z=%.3f" % (target_pose_1.position.x, target_pose_1.position.y, target_pose_1.position.z))

    # Posición C: (x+0.1, y+0.1, z)
    target_pose_2 = geometry_msgs.msg.Pose()
    target_pose_2.position.x = target_pose_1.position.x
    target_pose_2.position.y = target_pose_1.position.y + 0.1
    target_pose_2.position.z = start_pose.position.z
    target_pose_2.orientation = start_pose.orientation
    waypoints.append(target_pose_2)
    rospy.loginfo("Posición C: x=%.3f, y=%.3f, z=%.3f" % (target_pose_2.position.x, target_pose_2.position.y, target_pose_2.position.z))

    # Posición D: (x, y+0.1, z)
    target_pose_3 = geometry_msgs.msg.Pose()
    target_pose_3.position.x = start_pose.position.x
    target_pose_3.position.y = target_pose_2.position.y
    target_pose_3.position.z = start_pose.position.z
    target_pose_3.orientation = start_pose.orientation
    waypoints.append(target_pose_3)
    rospy.loginfo("Posición D: x=%.3f, y=%.3f, z=%.3f" % (target_pose_3.position.x, target_pose_3.position.y, target_pose_3.position.z))

    # Volver a la Posición A: (x, y, z)
    waypoints.append(start_pose)
    rospy.loginfo("Posición de regreso (A): x=%.3f, y=%.3f, z=%.3f" % (start_pose.position.x, start_pose.position.y, start_pose.position.z))

    # Planificar la trayectoria cartesiana para todos los waypoints definidos
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # Lista de waypoints a seguir
        0.02,        # Resolución de la trayectoria en metros
        0.0,
        avoid_collisions=True)  # Definir si debe evitar colisiones

    # Comprobar si la planificación fue completa (fraction == 1.0)
    if fraction == 1.0:
        rospy.loginfo("Trayectoria calculada correctamente. Ejecutando...")

        # Aplicar parametrización temporal para asegurar tiempos crecientes entre puntos de trayectoria
        for i, point in enumerate(plan.joint_trajectory.points):
            point.time_from_start = rospy.Duration(0.1 * i)  # Asegura tiempos crecientes

        # Ejecutar la trayectoria planificada
        group.execute(plan, wait=True)
    else:
        # Si la planificación no fue completa, mostrar advertencia
        rospy.logwarn("La trayectoria no se pudo calcular completamente: solo un %.2f%% se pudo planificar." % (fraction * 100))

    # Finalizar movimiento y cerrar node
    group.stop()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
