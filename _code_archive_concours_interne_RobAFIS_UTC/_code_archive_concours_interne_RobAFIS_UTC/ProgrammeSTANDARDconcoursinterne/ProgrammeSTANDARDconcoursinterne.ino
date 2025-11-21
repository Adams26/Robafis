#include "MeMegaPi.h" // Bibliothèque Makeblock pour la carte MegaPi
#include "Wire.h"      // Bibliothèque pour la communication I2C
 
// Capteur de couleur
#define NUM 1 // Nombre de capteurs de couleur utilisés
MeColorSensor colorsensor0(PORT_6); // Capteur de couleur connecté au port 6
MeGyro gyro(PORT_7); // Gyroscope connecté au port 7
uint8_t colorresult[NUM] = {0}; // Tableau pour stocker les résultats des couleurs détectées

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
float dernierCapCible = 0; // Dernier cap ciblé (commence à 0°)
int MotPorOUVRE = -255 ;
int MotPorFERME = 255 ;
int MotPinOUVRE = -255 ;
int MotPinFERME = 255 ;

//Delay
int delay_4cases = 3000 ;
int delay_5cases = delay_4cases * 5 / 4;
int delay_3cases = delay_4cases * 3 / 4;
int delay_2cases = delay_4cases * 2 / 4;
int delay_1case = delay_4cases * 1 / 4;
int changementcolonne = 300 ;
int delayOUVREporte = 800 ;
int delayFERMEporte = 800 ;
int delayOUVREpince = 800 ;
int delayFERMEpince = 800 ;

//Initialisation compteur de cube
int compteurCubes = 0;

void setup() {
Serial.begin(115200);
//Message de démarrage
 
// Initialisation de la communication série et du capteur de couleur
colorsensor0.SensorInit(); //initialisation du capteur de couleur
gyro.begin(); // Initialisation du gyroscope
stabiliserGyroscope();
delay(1000); // Stabilisation
Serial.println("Capteur de couleur et gyroscope initialisé !");
  Serial.println("Initialisation de la porte et de la pince");
  delay(2000);
  Serial.println("Fermeture de la pince et de la porte");
  motorPince.run(MotPinFERME);
  motorPorte.run(MotPorFERME);
  delay(delayFERMEporte);
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
  avancerAvecCorrection(dernierCapCible, delay_4cases);
 
  // Tourner à droite
  tourneADroite();
 
 // Avancer en ligne droite au cap 0° (passe de A5 à B5)
  avancerAvecCorrection(dernierCapCible, changementcolonne);
 
  // Tourner à droite
  tourneADroite();
 
  // Avancer en ligne droite au nouveau cap (vers B1)
  avancerAvecCorrection(dernierCapCible, delay_4cases);
 
  // Tourner à gauche
  tourneAGauche();
 
 // Avancer en ligne droite au cap 0° (passe de B1 à C1)
  avancerAvecCorrection(dernierCapCible, changementcolonne);
 
  // Tourner à gauche
  tourneAGauche();
 
  // Avancer en ligne droite au cap 0° (vers C5)
  avancerAvecCorrection(dernierCapCible, delay_4cases);
 
  // Tourner à droite
  tourneADroite();
 
 // Avancer en ligne droite au cap 0° (passe de C5 à D5)
  avancerAvecCorrection(dernierCapCible, changementcolonne);
 
  // Tourner à droite
  tourneADroite();
 
  // Avancer en ligne droite au nouveau cap (vers D0)
  avancerAvecCorrection(dernierCapCible, delay_5cases+300);
 
  // Fermeture de la pince et de la porte
  Serial.println("Fermeture de la pince et de la porte");
  motorPince.run(MotPinFERME);
  motorPorte.run(MotPorFERME);
  delay(delayFERMEpince);
  motorPince.stop();
  motorPorte.stop();
  Serial.println("Fin du ramassage");
 
  //recule vers C3
  avancerEnMarcheArriere(dernierCapCible, delay_3cases+200);
  tourneAGaucheAxe();
  avancerEnMarcheArriere(dernierCapCible, delay_1case);
  motorGauche.stop();
  motorDroite.stop();
  delay(1000);
}

void loop() {
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
void avancerAvecCorrection(float capCible, int duree) {
  unsigned long startTime = millis();
  while (millis() - startTime < duree) {
    gyro.update();
    float angleZ = gyro.getAngleZ(); // Lire l'angle Z
 
    // Gestion de la discontinuité ±179°
    float erreur = calculerErreurAngle(angleZ, capCible);
 
    // Calcul de la correction
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
 
    // Debug
    Serial.print("Avancer - Angle Z: ");
    Serial.println(angleZ);
    delay(200);
  }
  motorGauche.stop();
  motorDroite.stop();
}

// Fonction pour avancer en marche arrière avec correction d'angle
void avancerEnMarcheArriere(float capCible, int duree) {
  unsigned long startTime = millis();
  while (millis() - startTime < duree) {
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
 
    // Debug
    Serial.print("Marche arrière - Angle Z: ");
    Serial.println(angleZ);
    delay(50);
  }
  motorGauche.stop();
  motorDroite.stop();
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
    if (abs(erreur) <= 1) break;
 
    // Faire avancer uniquement le moteur droit
    motorGauche.stop();
    motorDroite.run(vitesseRotation);
 
    // Debug
    Serial.print("Tourner à gauche vers cap ");
    Serial.print(capCible);
    Serial.print(" | Angle Z: ");
    Serial.println(angleZ);
    delay(50);
  }
  motorGauche.stop();
  motorDroite.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}
 
// Fonction pour tourner à droite
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
 
    // Debug
    Serial.print("Tourner à droite vers cap ");
    Serial.print(capCible);
    Serial.print(" | Angle Z: ");
    Serial.println(angleZ);
    delay(50);
  }
  motorGauche.stop();
  motorDroite.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}
 
// Fonction pour tourner à gauche sur son axe de rotation
void tourneAGaucheAxe() {
  float capCible = dernierCapCible + 90;
  if (capCible > 180) capCible -= 360; // Ajuster pour rester dans [-180°, 180°]
 
  while (true) {
    gyro.update();
    float angleZ = gyro.getAngleZ();
 
    // Gestion de la discontinuité ±179°
    float erreur = calculerErreurAngle(angleZ, capCible);
 
    // Vérifier si on a atteint le cap
    if (abs(erreur) <= 1) break;
 
    // Faire avancer les moteurs en sens oposés
    motorGauche.run(vitesseRotation-12);
    motorDroite.run(vitesseRotation-12);
 
    // Debug
    Serial.print("Tourner à gauche vers cap ");
    Serial.print(capCible);
    Serial.print(" | Angle Z: ");
    Serial.println(angleZ);
    delay(50);
  }
  motorGauche.stop();
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
    if (abs(erreur) <= 0.5) break;
 
    // Faire tourner les moteur en deux sens opposés
    motorGauche.run(-vitesseRotation+12);
    motorDroite.run(-vitesseRotation+12);
  
    // Debug
    Serial.print("Tourner à gauche vers cap ");
    Serial.print(capCible);
    Serial.print(" | Angle Z: ");
    Serial.println(angleZ);
    delay(50);
  }
  motorGauche.stop();
  motorDroite.stop();
  dernierCapCible = capCible; // Mettre à jour le dernier cap ciblé
}
 
// Fonction pour calculer l'erreur d'angle avec gestion de la discontinuité ±179°
float calculerErreurAngle(float angleActuel, float capCible) {
  float erreur = capCible - angleActuel;
 
  // Gestion de la discontinuité autour de ±180°
  if (erreur > 180) erreur -= 360;
  else if (erreur < -180) erreur += 360;
 
  return erreur;
}
 
void TrierEnE1() {
      //Tri dans la case E1
      Serial.println("Tri dans la case E1");
      //tourne à droite 90°
      tourneADroiteAxe();
      //Avance en C1
      avancerAvecCorrection(dernierCapCible, delay_2cases+200);
      //Pivote à gauche 90°
      tourneAGaucheAxe();
      //Avance en E1
      avancerAvecCorrection(dernierCapCible, delay_2cases);
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
      tourneAGaucheAxe();
      //Avance en C3
      avancerAvecCorrection(dernierCapCible, delay_2cases);
      //pivote à droite 90°
      tourneADroiteAxe();
      delay(500);
      compteurCubes++;
}
 
void TrierEnE2() {
      //Tri dans la case E2
      Serial.println("Tri dans la case E2");
      //tourne à droite 90°
      tourneADroiteAxe();
      //Avance en C2
      avancerAvecCorrection(dernierCapCible, delay_1case+200);
      //Pivote à gauche 90°
      tourneAGaucheAxe();
      //Avance en E2
      avancerAvecCorrection(dernierCapCible, delay_2cases);
      //ouverture porte
      motorPorte.run(MotPorOUVRE);
      delay(delayOUVREporte);
      motorPorte.stop();
      delay(500);
      //recule en D2
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //fermeture porte et ouverture pince
      motorPorte.run(MotPorFERME);
      motorPince.run(MotPinOUVRE);
      delay(delayFERMEporte);
      motorPorte.stop();
      motorPince.stop();
      //recule en C2
      avancerEnMarcheArriere(dernierCapCible, delay_1case);
      //Fermeture pince
      motorPince.run(MotPinFERME);
      delay(delayFERMEpince);
      motorPince.stop();
      delay(200);
      //Pivote à gauche 90°
      tourneAGaucheAxe();
      //Avance en C3
      avancerAvecCorrection(dernierCapCible, delay_1case);
      //pivote à droite 90°
      tourneADroiteAxe();
      delay(500);
      compteurCubes++;
}
 
void TrierEnE3() {
      //Tri dans la case E3
      Serial.println("Tri dans la case E3");
      //Avance en E3
      avancerAvecCorrection(dernierCapCible, delay_2cases);
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
      tourneAGaucheAxe();
      //Avance en C4
      avancerAvecCorrection(dernierCapCible, delay_1case+200);
      //Pivote à droite 90°
      tourneADroiteAxe();
      //Avance en E4
      avancerAvecCorrection(dernierCapCible, delay_2cases);
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
      avancerAvecCorrection(dernierCapCible, delay_1case);
      //pivote à gauche 90°
      tourneAGaucheAxe();
      delay(500);
      compteurCubes++;
}
 
void TrierEnE5() {
      //Tri dans la case E5
      Serial.println("Tri dans la case E5");
      //tourne à gauche 90°
      tourneAGaucheAxe();
      //Avance en C5
      avancerAvecCorrection(dernierCapCible, delay_2cases+200);
      //Pivote à droite 90°
      tourneADroiteAxe();
      //Avance en E5
      avancerAvecCorrection(dernierCapCible, delay_2cases);
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
      avancerAvecCorrection(dernierCapCible, delay_2cases);
      //pivote à gauche 90°
      tourneAGaucheAxe();
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