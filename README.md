# Détecteur de Fumée Intelligent

Ce projet est un détecteur de fumée intelligent basé sur l'ESP32, intégrant plusieurs capteurs pour surveiller les conditions environnementales et déclencher des alertes en cas de danger. Une interface web intégrée permet de visualiser les données des capteurs en temps réel et de contrôler la LED RGB. Le projet utilise également le protocole MQTT pour transmettre les données vers Node-RED pour un traitement avancé.

## 🛠️ **Caractéristiques Principales**
- **ESP32** : Microcontrôleur utilisé pour gérer les capteurs, l'interface web et la communication MQTT.
- **DHT22** : Capteur de température et d'humidité, offrant des mesures précises.
- **MQ2** : Capteur de gaz capable de détecter des gaz combustibles comme le CO, le méthane, et les vapeurs de propane.
- **Photorésistance** : Capteur de lumière pour ajuster la luminosité des signaux lumineux.
- **LED RGB** : Changement de couleur en fonction de la température.
- **Buzzer** : Alerte sonore en cas de détection de niveaux de gaz dangereux.
- **Page Web** : Interface utilisateur hébergée sur l'ESP32 pour visualiser les données et contrôler la LED.
- **MQTT & Node-RED** : Communication avec un broker MQTT pour transmettre les données et afficher les alertes dans Node-RED.

## 🚀 **Fonctionnalités**
- **Surveillance en Temps Réel** : Affichage des données des capteurs sur une page web.
- **Système d'Alerte** : LED RGB pour des alertes visuelles et buzzer pour des alertes sonores.
- **Contrôle à Distance** : Contrôle de la LED RGB depuis l'interface web.
- **Intégration MQTT** : Envoi des données des capteurs vers Node-RED pour un suivi centralisé.

## 📦 **Matériel Nécessaire**
- ESP32
- DHT22 (capteur de température et d'humidité)
- MQ2 (capteur de gaz)
- Photorésistance (capteur de lumière)
- LED RGB
- Buzzer

## ⚙️ **Installation et Déploiement**
1. **Cloner ce dépôt** :
   ```bash
   git clone https://github.com/votre-utilisateur/votre-repo.git
   ```
2. **Configurer l'ESP32** :
   - Connectez les capteurs à l'ESP32 selon le schéma de câblage (à venir).
   - Téléversez le code sur l'ESP32 en utilisant vscode avec platformio

3. **Accéder à l'Interface Web** :
   - Une fois le code téléversé, connectez-vous à l'ESP32 via un navigateur pour visualiser les données des capteurs et contrôler la LED RGB.

4. **Configurer MQTT et Node-RED** :
   - Configurez un broker MQTT et connectez Node-RED pour recevoir et traiter les données des capteurs.
