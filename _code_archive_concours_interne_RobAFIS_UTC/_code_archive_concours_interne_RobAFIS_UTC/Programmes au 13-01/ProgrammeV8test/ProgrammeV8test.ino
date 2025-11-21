#include "MeMegaPi.h" // Bibliothèque Makeblock pour la carte MegaPi
#include "Wire.h"      // Bibliothèque pour la communication I2C

// Capteur de couleur
#define NUM 1 // Nombre de capteurs de couleur utilisés
MeColorSensor colorsensor0(PORT_6); // Capteur de couleur connecté au port 6
MeGyro gyro(PORT_7); // Gyroscope connecté au port 7
uint8_t colorresult[NUM] = {0}; // Tableau pour stocker les résultats des couleurs détectées
MeLineFollower lineSensor(PORT_8);        // Capteur de ligne connecté au port 8

// Moteurs
MeMegaPiDCMotor motorGauche(PORT1B); /* moteur de gauche */
MeMegaPiDCMotor motorDroite(PORT2B); /* moteur de droite */
MeMegaPiDCMotor motorPorte(PORT3B); /* moteur de la porte */
MeMegaPiDCMotor motorPince(PORT4B); /* moteur de la pince */

// Paramètres moteurs
// Constantes
const int vitesseNormale = 100; // Vitesse standard pour avancer droit
const int vitesseCorrection = 110; // Vitesse pour corriger la direction
const int vitesseNormaleMA = 100; // Vitesse standard pour avancer droit en marche arrière
const int vitesseCorrectionMA = 110; // Vitesse pour corriger la direction en marche arrière
const int vitesseRotation = 50; // Vitesse pour tourner sur place
float dernierCapCible = 0; // Dernier cap ciblé (commence à 0°)
int MotPorOUVRE = -255 ;
int MotPorFERME = 255 ;
int MotPinOUVRE = -255 ;
int MotPinFERME = 255 ;

// Delays
int delay_4cases = 3500 ;
int delay_5cases = delay_4cases * 5 / 4;
int delay_3cases = delay_4cases * 3 / 4;
int delay_2cases = delay_4cases * 2 / 4;
int delay_1case = delay_4cases * 1 / 4;
int changementcolonne = 350 ;
int delayOUVREporte = 800 ;
int delayFERMEporte = 800 ;
int delayOUVREpince = 800 ;
int delayFERMEpince = 800 ;

// Initialisation compteur de cube
int compteurCubes = 0;
int nombredeligne = 0;
bool lineFound = false;   // Flag to track if the line has been found after white detection
int sensorValue = lineSensor.readSensors();

//Variables actualisation gyro
unsigned long dernierTempsGyro = 0;
const unsigned long intervalleGyro = 50; // 100 ms par exemple

void setup() {
  Serial.begin(115200);
  delay(1000);
  // Initialisation de la communication série et du capteur de couleur
  colorsensor0.SensorInit(); // initialisation du capteur de couleur
  gyro.begin(); // Initialisation du gyroscope
  stabiliserGyroscope();
  Serial.println("Capteur de couleur et gyroscope initialisé !");
  Serial.println("Initialisation de la porte et de la pince");
  delay(1000);
  Serial.println("Fermeture de la pince et de la porte");
  motorPince.run(MotPinFERME);
  motorPorte.run(MotPorFERME);
  delay(800);
  motorPince.stop();
  motorPorte.stop();
  delay(500);
  Serial.println("Ouverture de la pince et de la porte");
  motorPorte.run(MotPorOUVRE);
  motorPince.run(MotPinOUVRE);
  delay(delayOUVREpince);
  motorPorte.stop();
  motorPince.stop();
  Serial.println("Début du ramassage");
  delay(1000);

  // Avancer en ligne droite au cap 0° (vers A5)
  avancerAvecCorrection(0, 4);

  // Tourner à droite
  tourneADroite();

  // Avancer en ligne droite au cap 0° (passe de A5 à B5)
  avancerAvecCorrection(-90, 1);

  // Tourner à droite
  tourneADroite();

  // Avancer en ligne droite au nouveau cap (vers B1)
  avancerAvecCorrection(180, 4);

  // Tourner à gauche
  tourneAGauche();

  // Avancer en ligne droite au cap 0° (passe de B1 à C1)
  avancerAvecCorrection(-90, 1);

  // Tourner à gauche
  tourneAGauche();

  // Avancer en ligne droite au cap 0° (vers C5)
  avancerAvecCorrection(0, 4);

  // Tourner à droite
  tourneADroite();

  // Avancer en ligne droite au cap 0° (passe de C5 à D5)
  avancerAvecCorrection(-90, 1);

  // Tourner à droite
  tourneADroite();

  // Avancer en ligne droite au nouveau cap (vers D0)
  avancerAvecCorrection(180, 5);

  // Fermeture de la pince et de la porte
  Serial.println("Fermeture de la pince et de la porte");
  motorPince.run(MotPinFERME);
  motorPorte.run(MotPorFERME);
  delay(delayFERMEpince);
  motorPince.stop();
  motorPorte.stop();
  delay(2000);
  Serial.println("Fin du ramassage");
  nombredeligne = 0;

  avancerEnMarcheArriere(180, 4);
  tourneAGauche();
  avancerEnMarcheArriere(-90, 2);
}

void loop() {
  Serial.println("Nombre total de lignes détectées : " + String(nombredeligne));
  delay(100); // Ajuste ce délai selon tes besoins

  colorresult[0] = colorsensor0.ColorIdentify();

  // Affiche la couleur détectée dans le moniteur série
  Serial.print("Couleur détectée : ");
  switch (colorresult[0]) {

    case BLACK:
    case WHITE:
      Serial.println("Cube NOIR avec pièce (Déchet chimique)");
      TrierEnE2();
      break;

    case BLUE:
      Serial.println("Cube BLEU (Déchet verre)");
      TrierEnE4();
      break;

    case YELLOW:
      Serial.println("Cube JAUNE (Déchet compostable)");
      TrierEnE3();
      break;

    case GREEN:
      Serial.println("Cube VERT (Déchet recyclable)");
      TrierEnE5();
      break;

    case RED:
      Serial.println("Cube ROUGE (Déchet traditionnel)");
      TrierEnE1();
      break;
  }
  delay(500);  // Délai de 0,5 seconde avant de refaire une nouvelle analyse
  if (compteurCubes >= 3) {
    motorGauche.stop();
    motorDroite.stop();
    motorPorte.stop();
    motorPince.stop();

    while (true) {
    }
  }
}

// Fonction pour avancer avec correction d'angle
void avancerAvecCorrection(float capCible, int nombredelignecible) {
    bool dernierEtatCapteur = false;
    while (nombredeligne < nombredelignecible) {
        // Vérifie si l'intervalle de temps est écoulé
        if (millis() - dernierTempsGyro >= intervalleGyro) {
            dernierTempsGyro = millis(); // Met à jour le dernier temps de mise à jour
            
            // Mise à jour et lecture du gyroscope
            gyro.update();
            float angleZ = gyro.getAngleZ();
            float erreur = calculerErreurAngle(angleZ, capCible);
            
            // Calcul et application de la correction
            if (erreur > 0) {
                motorDroite.run(vitesseCorrection);
                motorGauche.run(-vitesseNormale);
            } else if (erreur < 0) {
                motorGauche.run(-vitesseCorrection);
                motorDroite.run(vitesseNormale);
            } else {
                motorGauche.run(-vitesseNormale);
                motorDroite.run(vitesseNormale);
            }
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

        delay(50);
    }

    motorGauche.run(vitesseNormale);
    motorDroite.run(-vitesseNormale);
    delay(50);
    motorGauche.stop();
    motorDroite.stop();
    nombredeligne = 0;
}

// Fonction pour avancer en marche arrière avec correction d'angle
void avancerEnMarcheArriere(float capCible, int nombredelignecible) {
    bool dernierEtatCapteur = false; // Initialisation de l'état précédent du capteur
    while (nombredeligne < nombredelignecible) {
        // Mise à jour et lecture du gyroscope
        gyro.update();
        float angleZ = gyro.getAngleZ();
        float erreur = calculerErreurAngle(angleZ, capCible);
        
        // Calcul et application de la correction
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
        
        delay(50);
    }
    
    // Arrêt des moteurs à la fin
    motorGauche.run(-vitesseNormale);
    motorDroite.run(vitesseNormale);
    delay(50);
    motorGauche.stop();
    motorDroite.stop();
    
    // Réinitialisation du compteur
    nombredeligne = 0;
    
    Serial.println("=== Fin avancerAvecCorrection ===\n");
}

// Fonction pour calculer l'erreur d'angle
float calculerErreurAngle(float angleActuel, float angleCible) {
  float erreur = angleCible - angleActuel;
  if (erreur > 180) erreur -= 360; // Normalisation entre -180 et 180 degrés
  if (erreur < -180) erreur += 360;
  return erreur;
}

// Fonction pour tourner à gauche
void tourneAGauche() {
  float capCible = dernierCapCible + 90;
  if (capCible > 180) capCible -= 360; // Ajuster pour rester dans [-180°, 180°]
 
  while (true) {
    gyro.update();
    float angleZ = gyro.getAngleZ();
 
    // Gestion de la discontinuité ±179°
    float erreur = calculerErreurAngle(angleZ, capCible);
 
    // Vérifier si on a atteint le cap
    if (abs(erreur) <= 0.8) break;
 
    // Faire avancer uniquement le moteur droit
    motorGauche.stop();
    motorDroite.run(vitesseRotation);
  }
  motorGauche.stop();
  motorDroite.run(-vitesseRotation);
  delay(50);
  motorDroite.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}

// Fonction pour tourner à droite
void tourneADroite() {
  float capCible = dernierCapCible - 88;
  if (capCible < -180) capCible += 360; // Ajuster pour rester dans [-180°, 180°]
 
  while (true) {
    gyro.update();
    float angleZ = gyro.getAngleZ();
 
    // Gestion de la discontinuité ±179°
    float erreur = calculerErreurAngle(angleZ, capCible);
 
    // Vérifier si on a atteint le cap
    if (abs(erreur) <= 0.8) break;
 
    // Faire avancer uniquement le moteur gauche
    motorGauche.run(-vitesseRotation);
    motorDroite.stop();
  }
  motorDroite.stop();
  motorGauche.run(vitesseRotation);
  delay(50);
  motorGauche.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}

// Fonction pour tourner à gauche sur son axe de rotation
void tourneAGaucheAxe180() {
  float capCible = dernierCapCible + 180;
  if (capCible > 180) capCible -= 360; // Ajuster pour rester dans [-180°, 180°]
 
  while (true) {
    gyro.update();
    float angleZ = gyro.getAngleZ();
 
    // Gestion de la discontinuité ±179°
    float erreur = calculerErreurAngle(angleZ, capCible);
 
    // Vérifier si on a atteint le cap
    if (abs(erreur) <= 1) break;
 
    // Faire avancer uniquement le moteur droit
    motorGauche.run(vitesseRotation);
    motorDroite.run(vitesseRotation);
  }
  motorGauche.run(-vitesseRotation);
  motorDroite.run(-vitesseRotation);
  delay(10);
  motorDroite.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}

void tourneAGaucheAxe90() {
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
    motorGauche.run(vitesseRotation);
    motorDroite.run(vitesseRotation);
  }
  motorGauche.run(-vitesseRotation);
  motorDroite.run(-vitesseRotation);
  delay(10);
  motorDroite.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}

// Fonction pour tourner à droite sur son axe de rotation
void tourneADroiteAxe() {
  float capCible = dernierCapCible - 90;
  if (capCible < -180) capCible += 360; // Ajuster pour rester dans [-180°, 180°]
 
  while (true) {
    gyro.update();
    float angleZ = gyro.getAngleZ();
 
    // Gestion de la discontinuité ±179°
    float erreur = calculerErreurAngle(angleZ, capCible);
 
    // Vérifier si on a atteint le cap
    if (abs(erreur) <= 1) break;
 
    // Faire tourner les moteur en deux sens opposés
    motorGauche.run(-vitesseRotation+16);
    motorDroite.run(-vitesseRotation+16);
  }
  motorGauche.stop();
  motorDroite.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}

void TrierEnE1() {
      //Tri dans la case E1
      Serial.println("Tri dans la case E1");
      //tourne à droite 90°
      tourneADroiteAxe();
      //Avance en C1
      avancerAvecCorrection(dernierCapCible, 2);
      //Pivote à gauche 90°
      tourneAGaucheAxe90();
      //Avance en E1
      avancerAvecCorrection(dernierCapCible, 2);
      //ouverture porte
      motorPorte.run(MotPorOUVRE);
      delay(delayOUVREporte);
      motorPorte.stop();
      delay(500);
      //recule en D1
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //fermeture porte et ouverture pince
      motorPorte.run(MotPorFERME);
      motorPince.run(MotPinOUVRE);
      delay(delayFERMEporte);
      motorPorte.stop();
      motorPince.stop();
      //recule en C1
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //Fermeture pince
      motorPince.run(MotPinFERME);
      delay(delayFERMEpince);
      motorPince.stop();
      delay(200);
      //Pivote à gauche 90°
      tourneAGaucheAxe90();
      //Avance en C3
      avancerAvecCorrection(dernierCapCible, 2);
      //pivote à droite 90°
      tourneADroiteAxe();
      delay(500);
      compteurCubes++;
}

void TrierEnE2() {
      //Tri dans la case E2
      Serial.println("Tri dans la case E2");
      //tourne à droite 90°
      tourneADroite();
      //Avance en C2
      avancerAvecCorrection(180, 1);
      //Pivote à gauche 90°
      tourneAGauche();
      //Avance en E2
      avancerAvecCorrection(-90, 2);
      //ouverture porte
      motorPorte.run(MotPorOUVRE);
      delay(delayOUVREporte);
      motorPorte.stop();
      delay(500);
      //recule en D2
      avancerEnMarcheArriere(-90, 2);
      //fermeture porte et ouverture pince
      motorPorte.run(MotPorFERME);
      motorPince.run(MotPinOUVRE);
      delay(delayFERMEporte);
      motorPorte.stop();
      motorPince.stop();
      //recule en C2
      avancerEnMarcheArriere(-90, 1);
      //Fermeture pince
      motorPince.run(MotPinFERME);
      delay(delayFERMEpince);
      motorPince.stop();
      delay(200);
      //Pivote à gauche 90°
      tourneAGauche();
      //Avance en C3
      avancerAvecCorrection(0, 1);
      //pivote à droite 90°
      tourneADroite();
      delay(500);
      compteurCubes++;
}

void TrierEnE3() {
      //Tri dans la case E3
      Serial.println("Tri dans la case E3");
      //Avance en E3
      avancerAvecCorrection(dernierCapCible, 2);
      //ouverture porte
      motorPorte.run(MotPorOUVRE);
      delay(delayOUVREporte);
      motorPorte.stop();
      delay(500);
      //recule en D3
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //fermeture porte et ouverture pince
      motorPorte.run(MotPorFERME);
      motorPince.run(MotPinOUVRE);
      delay(delayFERMEporte);
      motorPorte.stop();
      motorPince.stop();
      //recule en C3
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //Fermeture pince
      motorPince.run(MotPinFERME);
      delay(delayFERMEpince);
      motorPince.stop();
      delay(500);
      compteurCubes++;
}

void TrierEnE4() {
      //Tri dans la case E4
      Serial.println("Tri dans la case E4");
      //tourne à gauche 90°
      tourneAGaucheAxe90();
      //Avance en C4
      avancerAvecCorrection(dernierCapCible, 1);
      //Pivote à droite 90°
      tourneADroiteAxe();
      //Avance en E4
      avancerAvecCorrection(dernierCapCible, 2);
      //ouverture porte
      motorPorte.run(MotPorOUVRE);
      delay(delayOUVREporte);
      motorPorte.stop();
      delay(500);
      //recule en D4
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //fermeture porte et ouverture pince
      motorPorte.run(MotPorFERME);
      motorPince.run(MotPinOUVRE);
      delay(delayFERMEporte);
      motorPorte.stop();
      motorPince.stop();
      //recule en C4
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //Fermeture pince
      motorPince.run(MotPinFERME);
      delay(delayFERMEpince);
      motorPince.stop();
      delay(200);
      //Pivote à droite 90°
      tourneADroiteAxe();
      //Avance en C3
      avancerAvecCorrection(dernierCapCible, 1);
      //pivote à gauche 90°
      tourneAGaucheAxe90();
      delay(500);
      compteurCubes++;
}

void TrierEnE5() {
      //Tri dans la case E5
      Serial.println("Tri dans la case E5");
      //tourne à gauche 90°
      tourneAGaucheAxe90();
      //Avance en C5
      avancerAvecCorrection(dernierCapCible, 2);
      //Pivote à droite 90°
      tourneADroiteAxe();
      //Avance en E5
      avancerAvecCorrection(dernierCapCible, 2);
      //ouverture porte
      motorPorte.run(MotPorOUVRE);
      delay(delayOUVREporte);
      motorPorte.stop();
      delay(500);
      //recule en D5
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //fermeture porte et ouverture pince
      motorPorte.run(MotPorFERME);
      motorPince.run(MotPinOUVRE);
      delay(delayFERMEporte);
      motorPorte.stop();
      motorPince.stop();
      //recule en C5
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //Fermeture pince
      motorPince.run(MotPinFERME);
      delay(delayFERMEpince);
      motorPince.stop();
      delay(200);
      //Pivote à droite 90°
      tourneADroiteAxe();
      //Avance en C3
      avancerAvecCorrection(dernierCapCible, 2);
      //pivote à gauche 90°
      tourneAGaucheAxe90();
      delay(500);
      compteurCubes++;
}

void stabiliserGyroscope() {
    gyro.update();
    float angleZ = gyro.getAngleZ();
    float seuilStabilisation = 0.5; // Seuil pour considérer le gyroscope comme stable
    while (abs(angleZ) > seuilStabilisation) {
        gyro.update();
        angleZ = gyro.getAngleZ();
        delay(10); // Attente pour stabiliser
    }
    Serial.println("Gyroscope stabilisé.");
}