if __name__ == "__main__":
    # Pour executer le code en local et avoir les bon import
    import sys
    import os
    sys.path.append(os.getcwd() + "\\src")

from utils.yaml_utils import yaml_data_import


def get_table_point(nom_fichier):
    """Fonction pour obtenir les points de la table d'après le fichier configuration yaml

    Args:
        nom_fichier (string): nom du fichier de configuation yaml

    Returns:
        liste float: liste des coordonnées de la table
    """

    point_table = yaml_data_import(nom_fichier)
    points = point_table.get('point_table')

    listeX = []
    listeY = []
    for point in points:
            listeX.append(points.get(point).get('x'))
            listeY.append(points.get(point).get('y'))
    
    # On ajoute les derniers points de la table pour fermer le rectangle
    listeX.append(listeX[0])
    listeY.append(listeY[0])

    return [listeX, listeY]


if __name__ == "__main__":
    nom_fichier = './src/table_config.yaml'
    table = get_table_point(nom_fichier)

    print(table)
