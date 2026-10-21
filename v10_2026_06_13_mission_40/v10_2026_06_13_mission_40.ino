// Importation de la bibliothèque externe Pololu pour faire fonctionner la barrette infrarouge.
#include <QTRSensors.h>

// ==========================================
// 1. DÉCLARATIONS DES BROCHES MOTEURS
// ==========================================
// Moteur Gauche
const int pinVitesseGauche = 3;
const int pinMoteurGaucheA = 7;
const int pinMoteurGaucheB = 6;

// Moteur Droit
const int pinMoteurDroitA = 5;
const int pinMoteurDroitB = 4;
const int pinVitesseDroit = 2;

// ==========================================
// 2. CONFIGURATION DU CAPTEUR QTR
// ==========================================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ==========================================
// 3. LE SÉQUENCEUR D'ACTIONS (LA PARTITION)
// ==========================================
const int totalEtapes = 5;
int indexEtapeActuelle = 0;

// --- PROGRAMMATION DES ÉTAPES ---
int ciblesMarqueurs[totalEtapes] = {1, 1, 2, 3, 10};
char cotesCapteurs[totalEtapes] = {'D','D','D','D','D'};

char typesActions[totalEtapes] = {'T','A','T','T','V'};

char sensRotations[totalEtapes] = {'D','-','D','G','-'};
int temposAvantAction[totalEtapes] = {500,100,100,150,0};
int tempsPauses[totalEtapes] = {0,0,0,0,0};

int dureesAveugle[totalEtapes] = {0, 1500, 0, 0, 0};
// =========================================================================

// --- LES VITESSES PAR TRONÇON ---
int vitessesEtapes[totalEtapes + 1] = {45, 45, 45, 45, 45, 45};

// --- ARRÊT TOTAL ---
int cibleArretFinale = 10;
int tempoAvantArret = 100;

// --- VARIABLES DE MÉMOIRE ---
int compteurGauche = 0;
int compteurDroite = 0;
bool surLigneGauche = false;
bool surLigneDroite = false;
bool robotArrete = false;

// === L'INTELLIGENCE DE PISTE ===
bool pisteInverse = false;

// === SÉCURITÉ ANTI-SORTIE DE PISTE ===
unsigned long chronoDerniereLigne = 0;
const int tempsToleranceBlanc = 200;

// ==========================================
// 4. RÉGLAGES DU CERVEAU (CONTRÔLE PID)
// ==========================================
int consigne = 3500;
int vitesseBase = 0;

// --- RÉGLAGES INTERMÉDIAIRES ---
float Kp = 0.025;
float Kd = 0.15;
int lastError = 0;

// ==========================================
// 5. SOUS-PROGRAMMES (LES ACTIONS DU ROBOT)
// ==========================================

unsigned int lirePisteEtDetecterMode() {
  unsigned int positionLue;

  if (pisteInverse == false) {
    positionLue = qtr.readLineBlack(sensorValues);

    if ((sensorValues[2] < 300 || sensorValues[3] < 300 || sensorValues[4] < 300 || sensorValues[5] < 300)
     && (sensorValues[0] > 600 && sensorValues[7] > 600)) {
      pisteInverse = true;
      Serial.println("!!! REFLEXE : PASSAGE AUTO EN PISTE INVERSEE !!!");
    }
  }
  else {
    positionLue = qtr.readLineWhite(sensorValues);

    if ((sensorValues[2] > 600 || sensorValues[3] > 600 || sensorValues[4] > 600 || sensorValues[5] > 600)
     && (sensorValues[0] < 300 && sensorValues[7] < 300)) {
      pisteInverse = false;
      Serial.println("!!! REFLEXE : RETOUR AUTO EN PISTE NORMALE !!!");
    }
  }
  return positionLue;
}

void actualiserVitesseEtPID(int nouvelleVitesse) {
  if (vitesseBase != nouvelleVitesse) {
    vitesseBase = nouvelleVitesse;

    float ratio = (float)vitesseBase / 45.0;

    Kp = 0.025 * ratio;
    Kd = 0.15 * ratio;

    Serial.print(">>> NOUVELLE ZONE - Vitesse : "); Serial.println(vitesseBase);
  }
}

// === NOUVELLE FONCTION : AVANCER À L'AVEUGLE ===
void avancerAveugle(int duree) {
  Serial.print("!!! ACTION [A] : SAUT A L'AVEUGLE DE ");
  Serial.print(duree);
  Serial.println(" ms !!!");

  // On force les moteurs tout droit à la vitesse actuelle du tronçon
  digitalWrite(pinMoteurGaucheA, HIGH); digitalWrite(pinMoteurGaucheB, LOW);
  analogWrite(pinVitesseGauche, vitesseBase);

  digitalWrite(pinMoteurDroitA, HIGH); digitalWrite(pinMoteurDroitB, LOW);
  analogWrite(pinVitesseDroit, vitesseBase);

  // On fige complètement le programme. Le robot ignore la ligne et la sécurité anti-crash.
  delay(duree);

  Serial.println("!!! FIN DU SAUT : REOUVERTURE DES YEUX !!!");
}
// ===============================================

void faireDemiTour(char direction) {
  int vitesseRotation = 45;

  if (direction == 'D') {
    digitalWrite(pinMoteurGaucheA, HIGH); digitalWrite(pinMoteurGaucheB, LOW);
    analogWrite(pinVitesseGauche, vitesseRotation);
    digitalWrite(pinMoteurDroitA, LOW); digitalWrite(pinMoteurDroitB, HIGH);
    analogWrite(pinVitesseDroit, vitesseRotation);
  } else if (direction == 'G') {
    digitalWrite(pinMoteurGaucheA, LOW); digitalWrite(pinMoteurGaucheB, HIGH);
    analogWrite(pinVitesseGauche, vitesseRotation);
    digitalWrite(pinMoteurDroitA, HIGH); digitalWrite(pinMoteurDroitB, LOW);
    analogWrite(pinVitesseDroit, vitesseRotation);
  }

  delay(200);

  while (true) {
    qtr.readLineBlack(sensorValues);
    if (sensorValues[3] > 600 || sensorValues[4] > 600) break;
  }

  analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
  delay(200);
}

void suivreLignePID(unsigned int positionActuelle) {
  int erreur = positionActuelle - consigne;
  int correction = (Kp * erreur) + (Kd * (erreur - lastError));
  lastError = erreur;

  int limiteCorrection = 100;
  if (correction > limiteCorrection) { correction = limiteCorrection; }
  else if (correction < -limiteCorrection) { correction = -limiteCorrection; }

  int vitesseGauche = vitesseBase + correction;
  int vitesseDroit = vitesseBase - correction;

  if (vitesseGauche < 0) vitesseGauche = 0;
  if (vitesseGauche > 255) vitesseGauche = 255;
  if (vitesseDroit < 0) vitesseDroit = 0;
  if (vitesseDroit > 255) vitesseDroit = 255;

  digitalWrite(pinMoteurGaucheA, HIGH); digitalWrite(pinMoteurGaucheB, LOW);
  analogWrite(pinVitesseGauche, vitesseGauche);
  digitalWrite(pinMoteurDroitA, HIGH); digitalWrite(pinMoteurDroitB, LOW);
  analogWrite(pinVitesseDroit, vitesseDroit);
}

// ==========================================
// 6. INITIALISATION
// ==========================================
void setup() {
  pinMode(pinMoteurGaucheA, OUTPUT); pinMode(pinMoteurGaucheB, OUTPUT); pinMode(pinVitesseGauche, OUTPUT);
  pinMode(pinMoteurDroitA, OUTPUT); pinMode(pinMoteurDroitB, OUTPUT); pinMode(pinVitesseDroit, OUTPUT);

  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){25, 27, 29, 31, 33, 35, 37, 41}, SensorCount);
  qtr.setEmitterPin(255);

  Serial.println("CALIBRATION (Passe le robot sur le BLANC ET sur le NOIR !)...");
  delay(1000);

  for (uint16_t i = 0; i < 150; i++) {
    qtr.calibrate();
    delay(20);
  }

  actualiserVitesseEtPID(vitessesEtapes[0]);
  Serial.println("GO !");
  delay(2000);
}

// ==========================================
// 7. BOUCLE PRINCIPALE
// ==========================================
void loop() {

  if (robotArrete == true) {
    analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
    return;
  }

  // --- LECTURE INTELLIGENTE ---
  unsigned int positionDuRobot = lirePisteEtDetecterMode();

  // --- SÉCURITÉ ANTI-SORTIE DE PISTE ---
  bool toutBlanc = true;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 300) { toutBlanc = false; break; }
  }

  if (toutBlanc == false) {
    chronoDerniereLigne = millis();
  }
  else {
    if (millis() - chronoDerniereLigne > tempsToleranceBlanc) {
      analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
      return;
    }
  }

  // --- COMPTAGE DES MARQUEURS ---
  if (sensorValues[0] > 750 && surLigneGauche == false) {
    compteurGauche++;
    surLigneGauche = true;
  }
  else if (sensorValues[0] < 300) {
    surLigneGauche = false;
  }

  if (sensorValues[7] > 750 && surLigneDroite == false) {
    compteurDroite++;
    surLigneDroite = true;
  }
  else if (sensorValues[7] < 300) {
    surLigneDroite = false;
  }

  // --- LE SÉQUENCEUR D'ACTIONS ---
  if (indexEtapeActuelle < totalEtapes) {
    int cibleMarqueurAttendu = ciblesMarqueurs[indexEtapeActuelle];
    char coteCapteurASurveiller = cotesCapteurs[indexEtapeActuelle];
    char typeActionDemandee = typesActions[indexEtapeActuelle];
    char sensDeLaRotation = sensRotations[indexEtapeActuelle];
    int tempoActuelle = temposAvantAction[indexEtapeActuelle];
    int pauseActuelle = tempsPauses[indexEtapeActuelle];

    // On récupère la durée demandée pour l'action Aveugle
    int dureePourAveugle = dureesAveugle[indexEtapeActuelle];

    int compteurEnCours = (coteCapteurASurveiller == 'D') ? compteurDroite : compteurGauche;

    if (compteurEnCours >= cibleMarqueurAttendu) {

      if (tempoActuelle > 0) {
        unsigned long tempsDepart = millis();
        while (millis() - tempsDepart < tempoActuelle) {
          unsigned int posDegagement = lirePisteEtDetecterMode();
          suivreLignePID(posDegagement);
        }
      }

      if (pauseActuelle > 0) {
        analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
        delay(pauseActuelle);
      }

      // === RÉPARTITION DES ACTIONS ===
      if (typeActionDemandee == 'T') {
        faireDemiTour(sensDeLaRotation);
      }
      else if (typeActionDemandee == 'V') {
        Serial.println(">>> TRONÇON VALIDE <<<");
      }
      else if (typeActionDemandee == 'S') {
        analogWrite(pinVitesseGauche, 0);
        analogWrite(pinVitesseDroit, 0);
        robotArrete = true;
        Serial.println("!!! ARRÊT DEMANDÉ PAR LE SEQUENCEUR !!!");
        return;
      }
      else if (typeActionDemandee == 'A') {
        // === DÉCLENCHEMENT DU NOUVEAU MODE ===
        avancerAveugle(dureePourAveugle);
      }

      indexEtapeActuelle++;
      compteurGauche = 0;
      compteurDroite = 0;
      actualiserVitesseEtPID(vitessesEtapes[indexEtapeActuelle]);
      return;
    }
  }

  // --- ARRÊT FINAL CLASSIQUE ---
  if (indexEtapeActuelle >= totalEtapes) {
    if (cibleArretFinale > 0 && (compteurDroite >= cibleArretFinale || compteurGauche >= cibleArretFinale)) {

      if (tempoAvantArret > 0) {
        unsigned long tempsDepart = millis();
        while (millis() - tempsDepart < tempoAvantArret) {
          unsigned int posArret = lirePisteEtDetecterMode();
          suivreLignePID(posArret);
        }
      }

      robotArrete = true;
      Serial.println("!!! BARRIÈRE ATTEINTE : ARRÊT TOTAL !!!");
      return;
    }
  }

  suivreLignePID(positionDuRobot);
}
