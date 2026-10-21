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
const int totalEtapes = 4; // 1 seule étape : on compte et on s'arrête
int indexEtapeActuelle = 0;

// --- PROGRAMMATION DES ÉTAPES ---
int ciblesMarqueurs[totalEtapes] = {2,3 ,3, 2}; // On s'arrête au 4ème trait
char cotesCapteurs[totalEtapes] = {'G','G','G','D'}; // On écoute uniquement à Gauche
char typesActions[totalEtapes] = {'T','T','T','S'}; // 'S' = STOP (Arrêt définitif)
char sensRotations[totalEtapes] = {'G','D','D','-'};
int temposAvantAction[totalEtapes] = {500,500,500,800}; // Petit dégagement de 100ms pour s'arrêter bien net SUR le trait
int tempsPauses[totalEtapes] = {0,0,0,0};

// --- LES VITESSES PAR TRONÇON ---
int vitessesEtapes[totalEtapes + 1] = {40, 40, 40, 40, 40};

// --- ARRÊT TOTAL (Désactivé, car géré par l'action 'S') ---
int cibleArretFinale = 0;
int tempoAvantArret = 0;

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

// --- RÉGLAGES INTERMÉDIAIRES (COMPROMIS) ---
float Kp = 0.035;
float Kd = 0.20;
int lastError = 0;

// ==========================================
// 5. SOUS-PROGRAMMES (LES ACTIONS DU ROBOT)
// ==========================================

// --- L'INTELLIGENCE ÉVÉNEMENTIELLE (DÉTECTION AUTO) ---
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
        // NOUVELLE ACTION : ARRET TOTAL !
        analogWrite(pinVitesseGauche, 0);
        analogWrite(pinVitesseDroit, 0);
        robotArrete = true;
        Serial.println("!!! 4 TRAITS GAUCHES ATTEINTS : ARRET DEMANDE PAR LE SEQUENCEUR !!!");
        return;
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
