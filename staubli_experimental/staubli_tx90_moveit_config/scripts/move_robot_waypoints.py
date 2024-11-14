#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
import numpy as np  

def calculate_trajectory_length(trajectory):
    length = 0.0
    previous_point = None
    for point in trajectory.joint_trajectory.points:
        current_position = point.positions
        if previous_point is not None:
            length += np.linalg.norm(np.array(current_position) - np.array(previous_point))
        previous_point = current_position
    return length

def move_robot():
    # Inicializar MoveIt! Commander y el nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot', anonymous=True)

    # Crear objetos para manejar el robot y el grupo de movimientos del brazo
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")

    # Establecer el tiempo máximo que se permitirá para planificar una trayectoria
    group.set_planning_time(60)

    # Obtener la posición y orientación actuales del robot como punto de partida
    start_pose = group.get_current_pose().pose

    # Definir una lista de waypoints (puntos de paso) para la trayectoria
    waypoints = [start_pose]  # Agregar el punto inicial (A)

    # Definir el primer objetivo (B)
    target_pose_1 = geometry_msgs.msg.Pose()
    target_pose_1.position.x = start_pose.position.x + 0.1
    target_pose_1.position.y = start_pose.position.y - 0.2
    target_pose_1.position.z = start_pose.position.z - 0.1
    q1 = quaternion_from_euler(0, 0, 1.57)
    target_pose_1.orientation.x = q1[0]
    target_pose_1.orientation.y = q1[1]
    target_pose_1.orientation.z = q1[2]
    target_pose_1.orientation.w = q1[3]
    waypoints.append(target_pose_1)

    # Definir el segundo objetivo (C)
    target_pose_2 = geometry_msgs.msg.Pose()
    target_pose_2.position.x = target_pose_1.position.x + 0.15
    target_pose_2.position.y = target_pose_1.position.y - 0.1
    target_pose_2.position.z = target_pose_1.position.z - 0.05
    q2 = quaternion_from_euler(0.4, 1.57, -0.3)
    target_pose_2.orientation.x = q2[0]
    target_pose_2.orientation.y = q2[1]
    target_pose_2.orientation.z = q2[2]
    target_pose_2.orientation.w = q2[3]
    waypoints.append(target_pose_2)

    # Definir el tercer objetivo (D)
    target_pose_3 = geometry_msgs.msg.Pose()
    target_pose_3.position.x = target_pose_2.position.x + 0.5
    target_pose_3.position.y = target_pose_2.position.y + 0.3
    target_pose_3.position.z = target_pose_2.position.z - 0.75
    q3 = quaternion_from_euler(0.785, -0.5, 0)
    target_pose_3.orientation.x = q3[0]
    target_pose_3.orientation.y = q3[1]
    target_pose_3.orientation.z = q3[2]
    target_pose_3.orientation.w = q3[3]
    waypoints.append(target_pose_3)

    # Volver al punto de inicio como último waypoint
    waypoints.append(start_pose)

    # Medir el tiempo de planificación
    planning_start_time = time.time()
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # Lista de waypoints a seguir
        0.02,       # Resolución de la trayectoria en metros
        0.0,
        avoid_collisions=True
    )
    planning_time = time.time() - planning_start_time

    # Calcular la longitud total de la trayectoria
    trajectory_length = calculate_trajectory_length(plan)

    # Comprobar si la planificación fue completa
    if fraction == 1.0:
        rospy.loginfo("Trayectoria calculada correctamente en %.2f segundos." % planning_time)
        rospy.loginfo("Número de puntos en la trayectoria: %d" % len(plan.joint_trajectory.points))
        rospy.loginfo("Longitud total de la trayectoria: %.3f" % trajectory_length)

        # Aplicar parametrización temporal para asegurar tiempos crecientes entre puntos de trayectoria
        for i, point in enumerate(plan.joint_trajectory.points):
            point.time_from_start = rospy.Duration(0.05 * i)

        # Medir el tiempo de ejecución de la trayectoria
        execution_start_time = time.time()
        group.execute(plan, wait=True)
        execution_time = time.time() - execution_start_time

        rospy.loginfo("Tiempo de ejecución de la trayectoria: %.2f segundos" % execution_time)
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
