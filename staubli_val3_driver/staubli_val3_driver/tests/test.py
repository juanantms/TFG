#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Inicializa el nodo
rospy.init_node('send_joint_position')

# Crea el publisher
pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=10)

# Espera que el publisher se conecte
rospy.sleep(1)

# Define la trayectoria de las articulaciones
joint_trajectory = JointTrajectory()
joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Inicializa las posiciones en 0.1 para cada articulación
current_positions = [0.1, -0.1, 0.1, 0.0, 0.0, 0.0]

# Define un bucle que aumente las posiciones cada 2 segundos
rate = rospy.Rate(0.5)  # 0.5 Hz = 1 publicación cada 2 segundos

while not rospy.is_shutdown():
    # Crea un nuevo punto de trayectoria
    point = JointTrajectoryPoint()
    point.positions = current_positions[:]  # Copia las posiciones actuales
    point.time_from_start = rospy.Duration(2.0)  # Duración para alcanzar la posición

    # Agrega el punto a la trayectoria
    joint_trajectory.points = [point]  # Sobrescribe puntos anteriores con el nuevo

    # Publica la trayectoria
    pub.publish(joint_trajectory)

    # Incrementa cada posición en 0.1
    current_positions = [pos + 0.1 for pos in current_positions]

    # Espera 2 segundos antes de la siguiente actualización
    rate.sleep()
