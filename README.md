# Robot_control

Ce paquet à pour objectif de contrôler le robot en vitesse linéaire et angulaire en évitant les obstacles.


## Installation

Pour l'installer, entrer les commandes suivantes :

```bash
# Téléchargement du repo
 git clone --recursive https://github.com/S6ril/Robot_control.git

# Petit problème avec une bibliothèque qui apparait 3x,
# On supprime une occurence dans sensor_board
cd .\Robot_control\external\sensor_board\
git submodule deinit -- .\external\serial_driver\

# On supprime une occurence dans TLF_API
cd .\Robot_control\external\TLF_API\
git submodule deinit -- .\external\serial_driver\
```


