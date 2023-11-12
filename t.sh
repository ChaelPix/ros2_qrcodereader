#!/bin/bash

# Liste tous les nœuds ROS 2
nodes=$(ros2 node list)

# Boucle sur chaque nœud
for node in $nodes
do
    # Obtient le nom du processus pour le nœud
    process_name=$(ros2 node info $node | grep "Pid:" | awk '{print $2}')

    # Vérifie si un nom de processus a été trouvé
    if [ ! -z "$process_name" ]; then
        # Tue le processus
        kill -9 $process_name
        echo "Killed node $node"
    else
        echo "Could not find process for node $node"
    fi
done
