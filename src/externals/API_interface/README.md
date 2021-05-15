# API-Gcode-Human
API pour pouvoir controler les robots en Gcode avec un code plus facile pour un Humain. Il est fait pour respecter le GCODE donner pour le robot PLP 2020.

## Sommaire

  * [Utilisation](#utilisation)
  * [Installation](#installation)
    + [Méthode commande github](#m-thode-commande-github)
    + [Méthode manuelle](#m-thode-manuelle)
    + [Résultat final](#r-sultat-final)
  * [Auteurs](#auteurs)

## Utilisation

Pour l'utiliser, il faut dans un premier temps connecter la STM32 du robot à l'ordinateur (ou Raspberry). Puis il faut connaitre le port sur lequel la carte est connecté. Pour cela sous Windows, il suffit d'ouvrir l'idle Arduino et de regarder le port dans `outils\ports`. Il faut aussi connaitre le baudrate de la carte (`112500` dans notre cas).

Pour utiliser cette api, on l'inclue dans notre projet, comme décrit ci-dessous.


```python 
# Importation de la classe depuis le dossier external 
# (voir Installation plus bas)
from external.API-Gcode-Human import Robot

# On se place dans le cas où le port de communication est le port "COM8"
robot = Robot("COM8", 112500)

# Exemple de récupération de la position du robot
while True:
        A = robot.get_pose()
        P = [A.x, A.y, A.theta]
        print(P)
```

## Installation

### Méthode commande github

Une manière simple est d'utiliser un sous module github. Pour cela placer vous dans le dossier de votre projet, puis effectuer la commande :

```bash
git submodule add https://github.com/S6ril/API-interface-python.git .\external\API-interface-python
```

Cela créer directement le dossier `external` avec le module inclut dedans. Le résultat final est décrit plus bas.


### Méthode manuelle

On peut faire cette étape à la main :
 * On télécharge ce module grâce à github, ou en ligne de commande
   ```bash
   git clone --recursive https://github.com/S6ril/API-interface-python.git
   ```

  * Puis on le copie/colle dans notre le dossier de notre projet. 
 Je conseille de l'inclure dans un dossier `external`, comme décrit ci dessous.

### Résultat final

Le dossier de votre projet doit ressembler à cela :

```
├── external                # Fichier avec toutes les bibliothèques importées
|   ├── API-Gcode-Human     # Fichier source de l'api
├── src                     # Fichier source de votre code
├── LICENSE
└── README.md
```

## Auteurs

 <table style="width:100%" >
  <tr>
    <th>
    <a href="https://github.com/S6ril/">
      <img width=30% src="https://avatars.githubusercontent.com/u/58038125?v=4" />
    </a>
    </th>
    <th>
    <a href="https://github.com/Starfunx">
      <img width=30% src="https://avatars.githubusercontent.com/u/7883804?v=4" />
    </a>
    </th>
  </tr>
  <tr>
    <th>S6ril</td>
    <th>Starfunx</td>
  </tr>
</table> 


<p align="center">
<img width="10%" src="https://avatars.githubusercontent.com/u/39584742?v=4" alt="Logo Valrob">
</p>