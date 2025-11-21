#include <ArduinoSTL.h>
#include <string>
#include "datamanager.h"
#include "MeMegaPi.h" // Bibliothèque Makeblock pour la carte MegaPi
#include "Wire.h"      // Bibliothèque pour la communication I2C

  DataManager dataManager;
  // Capteur de couleur
  #define NUM 1 // Nombre de capteurs de couleur utilisés
  MeColorSensor colorsensor0(PORT_6); // Capteur de couleur connecté au port 6
  MeGyro gyro(PORT_7); // Gyroscope connecté au port 7
  uint8_t colorresult[NUM] = {0}; // Tableau pour stocker les résultats des couleurs détectées
  MeLineFollower lineSensor(PORT_8);        // Line sensor connected to PORT6

  // Moteurs
  MeMegaPiDCMotor motorGauche(PORT1B); /* moteur de gauche */
  MeMegaPiDCMotor motorDroite(PORT2B); /* moteur de droite */
  MeMegaPiDCMotor motorPorte(PORT3B); /* moteur de la porte */
  MeMegaPiDCMotor motorPince(PORT4B); /* moteur de la pince */

  //Paramètres moteurs
  // Constantes
  const int vitesseNormale = 100; // Vitesse standard pour avancer droit
  const int vitesseCorrection = 110; // Vitesse pour corriger la direction
  const int vitesseNormaleMA = 100; // Vitesse standard pour avancer droit en marche arrière
  const int vitesseCorrectionMA = 110; // Vitesse pour corriger la direction en marche arrière
  const int vitesseRotation = 50; // Vitesse pour tourner sur place
  int MotPorOUVRE = -255 ;
  int MotPorFERME = 255 ;
  int MotPinOUVRE = -255 ;
  int MotPinFERME = 255 ;

  //Delay
  int delayOUVREporte = 800 ;
  int delayFERMEporte = 800 ;
  int delayOUVREpince = 800 ;
  int delayFERMEpince = 800 ;
  float dernierCapCible = 0; // Dernier cap ciblé (commence à 0°)
  int nombredeligne = 0;

void setup() { //initialisation 
  colorsensor0.SensorInit(); //initialisation du capteur de couleur
  gyro.begin(); // Initialisation du gyroscope
  delay(1000); // Stabilisation
  Serial.println("Capteur de couleur et gyroscope initialisé !");
  Serial.println("Initialisation de la porte et de la pince");
  delay(1000);
  Serial.println("Fermeture de la pince et de la porte");
  motorPince.run(MotPinFERME);
  motorPorte.run(MotPorFERME);
  delay(delayFERMEporte);
  motorPince.stop();
  motorPorte.stop();
  delay(500);
  Serial.println("Ouverture de la pince et de la porte");
  motorPorte.run(MotPorOUVRE);
  delay(delayOUVREpince);
  motorPorte.stop();
  Serial.println("Début du ramassage");
  delay(1000);
	Serial.begin(115200);
	Serial3.begin(115200);
  loop();
}

float calculerErreurAngle(float angleActuel, float capCible) {
    float erreur = capCible - angleActuel;
    
    // Gestion de la discontinuité autour de ±180°
    if (erreur > 180) erreur -= 360;
    else if (erreur < -180) erreur += 360;
    
    return erreur;
}

void tourneAGauche() {
    float capCible = dernierCapCible + 90;
    if (capCible > 180) capCible -= 360; // Ajuster pour rester dans [-180°, 180°]
    
    while (true) {
        gyro.update();
        float angleZ = gyro.getAngleZ();
        
        // Gestion de la discontinuité ±179°
        float erreur = calculerErreurAngle(angleZ, capCible);
        
        // Vérifier si on a atteint le cap
        if (abs(erreur) <= 1) break;
        
        // Faire avancer uniquement le moteur droit
        motorGauche.stop();
        motorDroite.run(vitesseRotation);
    }
    motorGauche.stop();
    motorDroite.stop();
    dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}

void tourneADroite() {
    float capCible = dernierCapCible - 90;
    if (capCible < -180) capCible += 360; // Ajuster pour rester dans [-180°, 180°]
    
    while (true) {
        gyro.update();
        float angleZ = gyro.getAngleZ();
        
        // Gestion de la discontinuité ±179°
        float erreur = calculerErreurAngle(angleZ, capCible);
        
        // Vérifier si on a atteint le cap
        if (abs(erreur) <= 1) break;
        
        // Faire avancer uniquement le moteur gauche
        motorGauche.run(-vitesseRotation);
        motorDroite.stop();
    }
    motorGauche.stop();
    motorDroite.stop();
    dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}

void avancerAvecCorrection(float capCible, int nombredelignecible) {
    bool dernierEtatCapteur = false; // Initialisation de l'état précédent du capteur
    
    Serial.println("\n=== Début avancerAvecCorrection ===");
    Serial.print("Cap cible: ");
    Serial.println(capCible);
    Serial.print("Nombre de lignes à détecter: ");
    Serial.println(nombredelignecible);
    Serial.println("Démarrage de la boucle de contrôle");
    
    while (nombredeligne < nombredelignecible) {
        // Mise à jour et lecture du gyroscope
        gyro.update();
        float angleZ = gyro.getAngleZ();
        float erreur = calculerErreurAngle(angleZ, capCible);
        
        Serial.print("Angle actuel: ");
        Serial.print(angleZ);
        Serial.print(" | Erreur: ");
        Serial.println(erreur);
        
        // Calcul et application de la correction
        if (erreur > 0) {
            Serial.println("Correction vers la droite");
            motorDroite.run(vitesseCorrection);
            motorGauche.run(-vitesseNormale);
        } else if (erreur < 0) {
            Serial.println("Correction vers la gauche");
            motorGauche.run(-vitesseCorrection);
            motorDroite.run(vitesseNormale);
        } else {
            Serial.println("Trajectoire droite");
            motorGauche.run(-vitesseNormale);
            motorDroite.run(vitesseNormale);
        }
        
        // Lecture et interprétation du capteur de ligne
        int sensorValue = lineSensor.readSensors();
        Serial.print("Valeur capteur de ligne: ");
        Serial.println(sensorValue);
        
        bool ligneDetectee = (sensorValue == S1_IN_S2_IN || sensorValue == S1_OUT_S2_IN);
        
        if (ligneDetectee && !dernierEtatCapteur) {
            nombredeligne++;
            Serial.print("!! Ligne détectée !! Compteur: ");
            Serial.println(nombredeligne);
        }
        
        dernierEtatCapteur = ligneDetectee;
        
        Serial.print("Progression: ");
        Serial.print(nombredeligne);
        Serial.print("/");
        Serial.println(nombredelignecible);
        
        delay(100);
    }
    
    // Arrêt des moteurs à la fin
    Serial.println("Arrêt des moteurs...");
    motorGauche.run(vitesseNormale);
    motorDroite.run(-vitesseNormale);
    delay(50);
    motorGauche.stop();
    motorDroite.stop();
    
    // Réinitialisation du compteur
    nombredeligne = 0;
    
    Serial.println("=== Fin avancerAvecCorrection ===\n");
}

void avancerEnMarcheArriere(float capCible, int nombredelignecible) {
  while (nombredeligne < nombredelignecible) {
    gyro.update();
    float angleZ = gyro.getAngleZ(); // Lire l'angle Z

    // Gestion de la discontinuité ±179°
    float erreur = calculerErreurAngle(angleZ, capCible);

    // Calcul de la correction pour la marche arrière
    if (erreur < 0) {
      motorDroite.run(-vitesseCorrectionMA); // Le moteur droit corrige trop à gauche
      motorGauche.run(vitesseNormaleMA);     // Le moteur gauche avance moins vite
    } else if (erreur > 0) {
      motorGauche.run(vitesseCorrectionMA); // Le moteur gauche corrige trop à droite
      motorDroite.run(-vitesseNormaleMA);   // Le moteur droit avance moins vite
    } else {
      motorGauche.run(vitesseNormaleMA);
      motorDroite.run(-vitesseNormaleMA);
    }

    // Vérification de la ligne noire
    int sensorValue = lineSensor.readSensors();
    if (sensorValue == S1_IN_S2_IN || sensorValue == S1_OUT_S2_IN)  { // Si une ligne noire est détectée
      nombredeligne++;  // Incrémente le compteur de lignes détectées
      Serial.println("Nombre de lignes détectées : " + String(nombredeligne));
    }
    delay(200); // Petit délai pour ne pas saturer le processeur
  }
  motorGauche.stop();
  motorDroite.stop();
  nombredeligne = 0;
}

void executingPathTraveling(std::vector<Movement> movements) { //calcul des trajectoires à prendre
    Serial.println("\n=== Début executingPathTraveling ===");
    Serial.print("Nombre de mouvements: ");
    Serial.println(movements.size());
    
    for(const Movement& mov : movements) {
        // Affichage de la destination
        char letter = 'A' + ((mov.point - 1) / 5);
        int number = 1 + ((mov.point - 1) % 5);
        Serial.print("Vers ");
        Serial.print(letter);
        Serial.print(number);
        Serial.print(" (point ");
        Serial.print(mov.point);
        Serial.println(")");
      
        // Validation du point cible
        if(mov.point < 1 || mov.point > 25) {
            Serial.println("ERREUR: Point invalide");
            continue;
        }

        if(mov.columns_first) {
            // Si rotation initiale nécessaire
            if(mov.needs_first_rotation) {
                if(mov.dy > 0) {
                    Serial.println("Rotation gauche pour monter");
                    tourneAGauche();
                } else if(mov.dy < 0) {
                    Serial.println("Rotation droite pour descendre");
                    tourneADroite();
                }
            }

            // Déplacement en colonnes
            if(mov.dy != 0) {
                Serial.print(abs(mov.dy));
                Serial.print(" cases en colonnes (");
                Serial.print(mov.dy > 0 ? "haut" : "bas");
                Serial.println(")");
                avancerAvecCorrection(dernierCapCible, abs(mov.dy));
            }

            // Rotation et déplacement en lignes si nécessaire
            if(mov.dx != 0) {
                if(mov.dx > 0) {
                    Serial.println("Rotation droite pour aller à droite");
                    tourneADroite();
                } else {
                    Serial.println("Rotation gauche pour aller à gauche");
                    tourneAGauche();
                }
                Serial.print(abs(mov.dx));
                Serial.print(" cases en lignes");
                Serial.println(mov.dx > 0 ? " (droite)" : " (gauche)");
                avancerAvecCorrection(dernierCapCible, abs(mov.dx));
            }
        } else {
            // Si rotation initiale nécessaire
            if(mov.needs_first_rotation) {
                if(mov.dx > 0) {
                    Serial.println("Rotation droite pour aller à droite");
                    tourneADroite();
                } else if(mov.dx < 0) {
                    Serial.println("Rotation gauche pour aller à gauche");
                    tourneAGauche();
                }
            }

            // Déplacement en lignes
            if(mov.dx != 0) {
                Serial.print(abs(mov.dx));
                Serial.print(" cases en lignes");
                Serial.println(mov.dx > 0 ? " (droite)" : " (gauche)");
                avancerAvecCorrection(dernierCapCible, abs(mov.dx));
            }

            // Rotation et déplacement en colonnes si nécessaire
            if(mov.dy != 0) {
                if(mov.dy > 0) {
                    Serial.println("Rotation gauche pour monter");
                    tourneAGauche();
                } else {
                    Serial.println("Rotation droite pour descendre");
                    tourneADroite();
                }
                Serial.print(abs(mov.dy));
                Serial.print(" cases en colonnes (");
                Serial.print(mov.dy > 0 ? "haut" : "bas");
                Serial.println(")");
                avancerAvecCorrection(dernierCapCible, abs(mov.dy));
            }
        }
    }
    Serial.println("=== Fin executingPathTraveling ===");
}

void scenarioAutoTest(){
  Serial.print("RENTRE DANS 9ZaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaZZZZZZZZZZZZZZ");
  // Avancer en ligne droite au cap 0° (vers A3)
  avancerAvecCorrection(dernierCapCible, 2);

  // Tourner à droite
  tourneADroite();

  // Avancer en ligne droite au cap 0° (vers C3)
  avancerAvecCorrection(dernierCapCible, 2);

  // Tourner à gauche
  tourneAGauche();

  // Avancer en ligne droite au cap 0° (vers C5)
  avancerAvecCorrection(dernierCapCible, 2);

  // Tourner à droite
  tourneADroite();

  // Avancer en ligne droite au nouveau cap (vers E5)
  avancerAvecCorrection(dernierCapCible, 2);

  //Reculer jusqu'en C5
  avancerEnMarcheArriere(dernierCapCible, 2);

  motorGauche.stop();
  motorDroite.stop();
  delay(1000);
}

void listener(){ //arrêt d'urgence
  if (Serial3.available()) {
    std::string datagram = String(Serial3.readString()).c_str();
    if(datagram.compare("9Z") == 0) {
      Serial.print("RENTRE DANS 9ZZZZZZZZZZZZZZZZZZZ");
      motorGauche.stop();
      motorDroite.stop();
    }
  }
}

void openPince(){
  motorPince.run(MotPinOUVRE);
  delay(delayOUVREpince);
  motorPince.stop();
}

void openDoor(){
  motorPorte.run(MotPorOUVRE);
  delay(delayOUVREporte);
  motorPorte.stop();
}

void loop() { //communication avec le bluetoth
  // Checking if data arrived on the bluetooth module (Serial3)
  if (Serial3.available()) {
    // Conversion from the Arduino String type to the std::string type, way easier for data processing
    std::string datagram = String(Serial3.readString()).c_str();

    // @TODO Fonction qui traite le message reçu du téléphone qui pour trouver un chemin SI il commence par 9S (futurement, 9D passera par le même type de process)
    std::string scenario = dataManager.processMessage(datagram);

    // @TODO Une fois le chemin calculé, on peut lancer notre scénario standard, executingPathTraveling va récupérer les mouvements à réaliser stockés dans dataManager.getMovements() pour les traiters
    if (scenario == "9S")
    {
      openPince();
      executingPathTraveling(dataManager.getMovements());
      
      /* ~~~~~~ @TODO Ici c'est le code qui lance le tri, il n'est pas complètement fonctionnel, donc le décommenter risque de créer par mal de soucis ~~~~~~ */
      // Fermer la porte et la pince
      /*motorPince.run(MotPinFERME);
      motorPorte.run(MotPorFERME);
      delay(delayFERMEporte);
      motorPince.stop();
      motorPorte.stop();

      int current_position = dataManager.getFinalPosition();
      int current_direction = dataManager.getFinalDirection();
      std::vector<COLORTYPES> color_list = dataManager.getColorList();

      for(int i = 0; i < 3; i++) {
        COLORTYPES detectedColor = colorsensor0.ColorIdentify();

        // Planifier et exécuter le mouvement vers la case E correspondante
        Movement new_movement = dataManager.moveToNewColor(detectedColor, color_list, current_position, current_direction);
        std::vector<Movement> singleMove = {new_movement};
        executingPathTraveling(singleMove);
        
        openDoor();
        
        // Reculer d'une case (retour en colonne D)
        avancerEnMarcheArriere(dernierCapCible, 1);

        // ouvre la pince et ferme la porte motorPorte.run(MotPorFERME); delay(delayFERMEporte); motorPorte.stop();
        motorPince.run(MotPinOUVRE);
        motorPorte.run(MotPorFERME);
        delay(delayFERMEporte);
        motorPince.stop();
        motorPorte.stop();

        // Reculer d'une case (retour en colonne C)
        avancerEnMarcheArriere(dernierCapCible, 1);
        
        current_position = new_movement.point - 10;  // Position en colonne C
        current_direction = 2;  // Orienté vers l'Est
      }*/
    }

    if(0 == datagram.compare("9D"))
    {
      Serial.print("TEST100");
      // @TODO ici le code pour la detection à ajouter
    }
    
    if(0 == datagram.compare("9A"))
    {
      scenarioAutoTest(); // @TODO c'est cette fonction qui lance l'autotest, il est appelé si l'application envoie "9A"
    }

    Serial.print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
  }
}
