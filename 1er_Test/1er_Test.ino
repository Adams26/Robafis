#include <MeMegaPi.h> // Bibliothèque pour la carte MegaPi

MeMegaPiDCMotor moteur(PORT1B);
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

}

void tournerbras() {

    // Lecture clavier PC
  if (Serial.available()) {
    char key = Serial.read();
    if (key== "A") {
    moteur.run(250);
   }
   
  }
   
}
