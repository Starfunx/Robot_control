# Serial driver

Cette classe permet d'ajouter quelques méthodes à la classe `serial.Serial` du module `pyserial`. Elle hérite de cette classe.

## Utilisation

Il est facile d'ajouter cette classe avec un git submodule. Pour cela, dans votre dossier de travail, executer la commande :

```bash
git submodule add https://github.com/S6ril/serial_driver.git external/serial_driver
```

Cela permet de créer un nouveau dossier `external` contenant toutes vos bibliothèques à importer, puis un dossier `serial_driver` avec cette classe.

Votre dossier ressemble alors :
```
.
├── external                # Fichier avec toutes les bibliothèques importées
|   ├── serial_driver       # Fichier source de la classe serial
├── src                     # Fichier source de votre code
├── LICENSE
└── README.md
```


Une manière simple de l'utiliser est de faire hériter une nouvelle classe de cette dernière. Par exemple :

```python
# Importation de la classe depuis le dossier external
from external.serial_driver import BetterSerial

class my_class(BetterSerial):
    def __init__(self, portserial, bauderate):
        # Héritance de la classe BetterSerial, et par extension de la classe serial.
        BetterSerial.__init__(self, portserial, bauderate) 

```

Ensuite lors de l'utilisation de votre classe, vous avez accès à toutes les méthodes de `pyserial` et les nouvelles méthodes décrites ci-dessous.

Un exemple concret d'utilisation est le script `CarteDetecteurObstacle`.

## Méthodes ajoutées

### Clean sérial

On ajoute la méthode `cleanSerial()` afin de nettoyer facilement les données du port série. Cela exectute alors des `flush` sur les ports série en entrée et en sortie de l'ordinateur.

### Décode sérial

On ajoute la méthode `decode_serial()` qui permet de directement lire une ligne du port série et de retourner le message en un tableau de `str`. Cela est plus facile à utiliser ensuite.

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