# Robot_control

Ce paquet à pour objectif de contrôler le robot en vitesse linéaire et angulaire en évitant les obstacles.


## Nouvelle Installation

On présente ici l'installation dès le départ sur une Raspberry Pi modèle 3+. 

### Installation de l'OS Raspberry

Dans un premier temps il faut un OS sur la carte Raspberry.
Aller sur le site officiel, et télécharger le logiciel [Raspberry Pi Imager](https://www.raspberrypi.org/software/). 

On paramètre le wifi et la connection ssh grâce à ce logiciel. Pour cela sur la page d'accueil faîtes le raccourcis :
`ctrl + shift + x`. Entrer alors :
* `hostname` : laisser le à `raspberrypi`.
* Activer le ssh, le mot de passe est `raspberry`.
* Ajouter le réseau wifi, avec la zone de wifi `FR`. (optionnel si vous avez un câble ethernet)
* Changer les paramètres locaux `Europe/Paris` et le clavier en `fr`.

Une fois ces paramètres choisi, sauvegarder et choisissez l'os par défaut 32 bits (avec ou sans version Desktop, à vous de voir 😉). Par la suite, on utilise la version Destop Lite. 

Une fois la carte ssh flashée, on peut inserer la carte dans la Raspberry Pi.


### Première connexion à la Raspberry

Dans ce tutoriel, nous sommes sous Windows. Cela reste équivalent pour d'autres OS comme Linux ou Mac.

Nous allons nous connecter en ssh à la Raspberry Pi fraichement installée. Assurez vous que la carte sd est dans la Raspberry, qu'elle soit allumée, et connecter à internet (en wifi, il n'y a aucun voyant 🙁). 

❗ Votre ordinateur et la carte doivent être sur le même réseau local ❗

* Ouvrer un terminal Powersheel
* Taper la commande `ssh pi@raspberrypi.local`. Si la commande ne marche pas, il faudra trouver l'adresse locale de la Raspberry avec votre router. La commande sera : `ssh pi@192.168.x.xxx` avec `x` l'adresse que vous avez trouvé. 

Vous devrier avoir maintenant accès à la Raspberry en ssh.

### Installation du logiciel à partir du Github

Pour l'installer, entrer les commandes suivantes :

```bash
# Téléchargement du repo
git clone https://github.com/S6ril/Robot_control.git --branch v0.2
```

Si vous avez du mal avec les lignes de commandes, il reste toujours possible de télécharger la realease directement sur Github dans une archive zip. Vous pouvez ensuite la copier coller sur la Raspberry Pi. 
