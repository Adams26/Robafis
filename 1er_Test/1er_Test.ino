#include <MeMegaPi.h> // Bibliothèque pour la carte MegaPi
#include "Wire.h"      // Bibliothèque pour la communication I2C

// Moteurs
//MeMegaPiDCMotor moteur(PORT1B);
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
#define BT Serial1

const int VITESSE = 250;

void setup() {


  Serial.begin(115200);   // PC (Moniteur Série)
  BT.begin(9600);         // BLE (LightBlue) - la plupart des HM-10/clones: 9600

  Serial.println("Commandes: A=avant, R=arriere, S=stop");
  BT.println("Commands: A=forward, R=reverse, S=stop");

}

void loop() {

  tournerbras();
  pince();



}

void tournerbras() {

    // Lecture clavier PC
  if (Serial.available()) {
    char key = Serial.read();
    Serial.println(key);
    if (key== 'A') {
     motorGauche.run(250);
     motorPince.stop();
    }
   
  }
   
}

void pince() {

    // Lecture clavier PC
  if (Serial.available()) {
    char key = Serial.read();
    if (key=='A') {
     motorPince.run(MotPinOUVRE);
     delay(delayOUVREpince);
     motorPince.stop();

    }
    if (key== 'B') {
      motorPince.run(MotPinFERME);
      delay(delayFERMEpince);
      motorPince.stop();

    }
   
  }
   
}
