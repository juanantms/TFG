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
    group.set_max_velocity_scaling_factor(0.5)  # Escala de velocidad para hardware real
    group.set_max_acceleration_scaling_factor(0.5)  # Escala de aceleración

    # Obtener la posición y orientación actuales del robot como punto de partida
    start_pose = group.get_current_pose().pose

    # Definir una lista de waypoints (puntos de paso) para la trayectoria
    waypoints = []

    # Agregar el punto inicial (A) a la lista de waypoints
    waypoints.append(start_pose)
    rospy.loginfo("Posición A: x=%.3f, y=%.3f, z=%.3f" % (start_pose.position.x, start_pose.position.y, start_pose.position.z))

    # Definir el primer objetivo (B) con desplazamiento desde la posición inicial
    target_pose_1 = geometry_msgs.msg.Pose()
    target_pose_1.position.x = start_pose.position.x + 0.1
    target_pose_1.position.y = start_pose.position.y - 0.2
    target_pose_1.position.z = start_pose.position.z - 0.1

    # Establecer la orientación de B: 90 grados de rotación en Yaw (giro alrededor del eje Z)
    q1 = quaternion_from_euler(0, 0, 1.57)
    target_pose_1.orientation.x = q1[0]
    target_pose_1.orientation.y = q1[1]
    target_pose_1.orientation.z = q1[2]
    target_pose_1.orientation.w = q1[3]

    waypoints.append(target_pose_1)
    rospy.loginfo("Posición B: x=%.3f, y=%.3f, z=%.3f" % (target_pose_1.position.x, target_pose_1.position.y, target_pose_1.position.z))

    # Definir el segundo objetivo (C)
    target_pose_2 = geometry_msgs.msg.Pose()
    target_pose_2.position.x = target_pose_1.position.x + 0.15
    target_pose_2.position.y = target_pose_1.position.y - 0.1
    target_pose_2.position.z = target_pose_1.position.z - 0.05

    # Orientación para C
    q2 = quaternion_from_euler(0.4, 1.57, -0.3)
    target_pose_2.orientation.x = q2[0]
    target_pose_2.orientation.y = q2[1]
    target_pose_2.orientation.z = q2[2]
    target_pose_2.orientation.w = q2[3]

    waypoints.append(target_pose_2)
    rospy.loginfo("Posición C: x=%.3f, y=%.3f, z=%.3f" % (target_pose_2.position.x, target_pose_2.position.y, target_pose_2.position.z))

    # Definir el tercer objetivo (D)
    target_pose_3 = geometry_msgs.msg.Pose()
    target_pose_3.position.x = target_pose_2.position.x + 0.5
    target_pose_3.position.y = target_pose_2.position.y + 0.3
    target_pose_3.position.z = target_pose_2.position.z - 0.75

    # Orientación para D
    q3 = quaternion_from_euler(-0.785, -1.5, 0)
    target_pose_3.orientation.x = q3[0]
    target_pose_3.orientation.y = q3[1]
    target_pose_3.orientation.z = q3[2]
    target_pose_3.orientation.w = q3[3]

    waypoints.append(target_pose_3)
    rospy.loginfo("Posición D: x=%.3f, y=%.3f, z=%.3f" % (target_pose_3.position.x, target_pose_3.position.y, target_pose_3.position.z))

    # Planificar la trayectoria cartesiana para todos los waypoints definidos
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
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
