# Sensor Board

Ce paquet permet d'interfacer une carte regroupant plusieurs capteurs et de renvoyer les distances mesurées. On utilise ici des capteurs dans une direction (exemple : capteurs ultrasons, capteurs laser, sharps).

## Sommaire

  * [Installation](#installation)
  * [Utilisation](#utilisation)
  * [Erreurs possibles](#erreurs-possibles)
    + [Mauvaise connection](#mauvaise-connection)
    + [Paquets manquants](#paquets-manquants)
  * [Auteurs](#auteurs)


## Installation

❗❗ Ce paquet utilise mon paquet `BetterSerial` pour communiquer plus facilement. Il faut donc le télécharger (`git clone`) avec l'option **recursive** pour obtenir la totalité des paquets necessaires.

```git 
git clone --recursive https://github.com/S6ril/sensor_board.git
```

## Utilisation

Cette classe permet d'interfacer facilement une carte de detection d'obstacle (carte avec plusieurs capteurs) sous Python.
Pour cela il suffit d'utiliser la bibliotèque dans le fichier `src`.

Pour l'utiliser, il faut inialiser la carte avec son port série et son bauderate.
Puis pour récupérer les données de la carte, on execute la fonction `carte.get_distance("X")`, avec `X` le numéro du capteur, ou `A` pour afficher tous les capteurs. 
Par exemple avec une boucle `while`, cela donne le code suivant :

```python
carte = CarteDetecteurObstacle("COM8", 9600)

while True:
    A = carte.get_distance("A")
    print(A)
```

Pour quitter la boucle, il suffit de faire `CTRL+C`. Cela quitte le terminal.

Dans le fichier `src arduino`, vous trouverez un code arduino fonctionnant avec des capteurs ultrasons. Ce code est compatible avec ce script Python.

## Erreurs possibles

Voici une liste des possibles erreurs lorsque vous utilisez ce paquet.

### Mauvaise connection
```bash
serial.serialutil.SerialException: could not open port 'COM8': FileNotFoundError(2, 'Le fichier spécifié est introuvable.', None, 2) 
```

Vérifier que le port USB sur lequel vous avez connecter la carte. Pour le trouver facilement, il suffit de passer par l'idle Arduino et de regarder dans le menu : `outils\Port` le port USB de la carte. 

### Paquets manquants

```bash
from external.serial_driver import BetterSerial ModuleNotFoundError: No module named 'external'
```

Ici il manque le sous-paquet pour la communication avec le port série. Il manque surement l'option `--recursive` lorsque vous avez fait la commande `git clone`. 


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