# Code Arduino

## Présentation

Dans cette partie de code, on retrouve un script permettant de récupérer les données de plusieurs capteurs ultrasons, et d'envoyer les distances via le port série. La communication se fait avec du GCODE.

## Sommaire

  * [Présentation](#pr-sentation)
  * [Sommaire](#sommaire)
  * [Mise en route rapide](#mise-en-route-rapide)
  * [Configuration](#configuration)
  * [Utilisation](#utilisation)
  * [Auteurs](#auteurs)

## Mise en route rapide

Ce code est déjà configuré pour deux capteurs ultrasons à connecter sur une arduino. 

Voici le schéma de cablage : 
<p align="center">
<img width="75%" src=".\cablage\Exemple_cablage_ultrason.png" alt="Logo Valrob">
</p>

Une fois le câble et le code `ultrason_arduino.ino` téleversé sur la carte, passer directement à la section [Utilisation](#utilisation).

## Configuration

Queslques étapes sont nécessaires :
1. Il faut définir dans le fichier `pin_configuration.h` les différents pins utilisés pour les capteurs. On propose la notation suivante :
    ```cpp
    // Sensor 1 definition
    #define ECHO_PIN1 13
    #define TRIG_PIN1 12
    ```

2. Il faut choisir une distance maximale de mesure. Comme le capteur ultrason est un capteur qui mesure le temps que prend l'onde à voyager sur l'objet, on définit un temps maximal `Timeout`. Si le capteur n'a pas eut de retours d'information avant ce temps, on renvoie la distance maximale définie. Pour définir ce temps, il faut entrer la valeur `Timeout` comme cela :

    ```cpp
    // TimeOut = Max.Distance(mm) * 5,88
    int TimeOut = 1000 * 5.8; // 1 m de distance max
    ```

3. Il faut ajouter ces capteurs dans le fichier principal `ultrason_arduino.ino`. Pour cela, il suffit d'entrer le nombre de capteurs que l'on souhaite interfacer, puis de les ajouter dans le tableau de capteur (exemple avec 2 capteurs) : 
    ```cpp
    // Changer le nombre de capteur et les ajouter dans la liste suivante
    #define NUMBER_SENSOR 2
    Ultrasonic sensor[NUMBER_SENSOR] = {Ultrasonic(TRIG_PIN1, ECHO_PIN1, TimeOut), Ultrasonic(TRIG_PIN2, ECHO_PIN2, TimeOut)};
    ```
    On retrouve l'objet `Ultrasonic(TRIG_PIN, ECHO_PIN, TimeOut)` qui représente le capteur ultrason.

Dans la suite, on utilise seulement la méthode `Distance()` des capteurs ultrasons pour connaitre la distance mesurée.
L'algorithme est cadancé à la vitesse de *40 Hz*, les données des différents capteurs sont récupérées l'une après l'autre et stoquées dans le tableau 
```cpp 
double distance[NUMBER_SENSOR] = {0.0 };
```
Lorsque l'utilisateur demande les distances, on lit seulement ce tableau qui contient les dernières valeurs mesurées.


## Utilisation

J'utilise la norme suivante pour communiquer avec la carte. Il suffit d'entrer ces commandes dans le port série pour obtenir la requète demandée.

* `SA` pour demander toutes les distances de tous les capteurs. Il est alors retoruné les distances dans l'ordre des capteurs, séparées par un `;`. 
Exemple de retour pour deux capteurs :
    ```
    SA
    >>> 1003.99; 128.96
    ```

* `Sxx` avec `xx` le numéro du capteur pour obtenir la distance d'un seul capteur. Par exemple `S0` demande la distance mesurée du capteur numéro 0.
    ```
    S0
    >>> 102.48
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



