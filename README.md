# Simulation de Poppy Ergo Jr avec Gazebo

Charge le modèle dans Gazebo ainsi que les contrôleurs (trajectory ou position).
Le modèle inclue la caméra qui est simulée par un plugin Gazebo.
Enfin un service basique pour ouvrir/fermer la pince est disponible.

Nécessite l'installation de python3-lxml: `sudo apt install python3-lxml`

Dépend du package [Poppy Ergo Jr Description](https://github.com/poppy-project/poppy_ergo_jr_description)

`roslaunch poppy_ergo_jr_gazebo start_gazebo.launch gripper:=true lamp:=false` (si `traj:=false` est passé, les contrôleurs en position seront chargés)
`roslaunch poppy_ergo_jr_gazebo spawn_cubes.launch`
`rosrun poppy_ergo_jr_gazebo gripper_gz_service.py`