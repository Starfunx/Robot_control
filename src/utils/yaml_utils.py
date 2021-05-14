"""
Programme pour lire les données du json
"""

import yaml


def yaml_data_import(nom_fichier):
    """Fonction pour importer et lire les fichier yaml

    Args:
        nom_fichier (string): nom du fichier à importer

    Returns:
        dict: dictionnaire des donnée chargés
    """
    with open(nom_fichier) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        return data


if __name__ == "__main__":
    # Pour executer le code en local et avoir les bon import
    nom_fichier = './src/config/robot_config.yaml'
    dictionnaire = yaml_data_import(nom_fichier)
    print(dictionnaire)
    

    
