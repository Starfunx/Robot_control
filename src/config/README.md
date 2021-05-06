# Fichier de configuration

On retrouve dans ce module les fichiers de configuration necessaire au programme.

## Sommaire

  * [Configuration du robot](#configuration-du-robot)
  * [Configuration de la table](#configuration-de-la-table)

## Configuration du robot

La configuration du robot se fait dans le fichier `robot_config.yaml`. On y retrouve :
* `point_robot` avec les différentes coordonnées de la forme du robot
* `capteurs` avec les différents capteurs, on retrouve pour chaques capteurs 
    * `type` : le type du capteur
    * `x` : sa position en x dans le repère du robot
    * `y` : sa position en y dans le repère du robot
    * `theta` : l'angle d'inclinaison du capteur suivant l'axe `y_robot`.

## Configuration de la table

La configuration de la table se fait dans le fichier `table_config.yaml`. On retrouve dedans les différents points de la table, avec :
* `x` : la coordonnées en x du point dans le repère de la table.
* `y` : la coordonnées en y du point dans le repère de la table.