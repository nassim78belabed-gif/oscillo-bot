#include <QTRSensors.h>

// ==========================================
// 1. DÉCLARATIONS DES BROCHES MOTEURS
// ==========================================
const int pinVitesseGauche = 3;
const int pinMoteurGaucheA = 7;
const int pinMoteurGaucheB = 6;

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
// C'est ici que tu programmes tout ton parcours.
// RÈGLE D'OR : Chaque colonne (de haut en bas) représente UNE étape complète.

// 1. COMBIEN D'ÉTAPES ? (Combien de fois le robot va-t-il réagir à un marqueur ?)
const int totalEtapes = 7;
int indexEtapeActuelle = 0; // Le robot commence à l'étape 0

// --- PROGRAMMATION DES ÉTAPES ---

// 2. COMBIEN DE TRAITS ? (Attention : le comptage repart à zéro après chaque étape)
int ciblesMarqueurs[totalEtapes] = {1, 2, 2, 3, 1, 3, 8};

// 3. QUEL CÔTÉ ÉCOUTER ? ('D' pour la Droite, 'G' pour la Gauche)
char cotesCapteurs[totalEtapes] = {'D', 'D', 'D', 'D', 'D', 'D', 'D'};

// 4. QUELLE ACTION FAIRE ?
// 'V' = Ligne droite (change juste la vitesse ou fait une pause)
// 'T' = Tourner (fait un demi-tour)
char typesActions[totalEtapes] = {'V', 'V', 'V', 'V', 'V', 'V', 'V'};

// 5. QUEL SENS POUR LE DEMI-TOUR ?
// 'G' = Pivote à gauche, 'D' = Pivote à droite.
// Si l'action au-dessus est 'V', met juste un tiret '-' pour faire propre.
char sensRotations[totalEtapes] = {'-', '-', '-', '-', '-', '-', '-'};

// 6. DÉGAGEMENT AVANT L'ACTION ? (En millisecondes)
// Temps laissé au robot pour avancer SUR le trait avant d'agir (0 = action immédiate).
int temposAvantAction[totalEtapes] = {0, 0, 0, 0, 0, 0, 0};

// 7. TEMPS DE PAUSE SUR LE TRAIT ? (En millisecondes)
// Ex: 2000 = Arrêt total de 2 secondes. 0 = Aucune pause, on enchaîne en roulant.
int tempsPauses[totalEtapes] = {2000, 2000, 2000, 2000, 2000, 2000, 2000};

// --- LES VITESSES PAR TRONÇON ---
// ATTENTION : Il y a toujours UNE vitesse de plus que le nombre d'étapes (7 étapes = 8 tronçons de piste).
// Ordre : {Vitesse de Départ, Après étape 1, Après étape 2, Après étape 3... Vitesse Finale}
int vitessesEtapes[totalEtapes + 1] = {45, 45, 45, 45, 45, 45, 45, 45};

// --- ARRÊT TOTAL (LIGNE D'ARRIVÉE) ---
// S'active APRÈS que les 7 étapes soient terminées.
int cibleArretFinale = 10; // S'arrête au 10ème trait rencontré à la fin du parcours
int tempoAvantArret = 0; // Dégagement pour passer la ligne d'arrivée avant de piler

// --- VARIABLES DE MÉMOIRE ---
int compteurGauche = 0;
int compteurDroite = 0;
bool surLigneGauche = false;
bool surLigneDroite = false;
bool robotArrete = false;

// ==========================================
// 4. RÉGLAGES DU CERVEAU (CONTRÔLE PID)
// ==========================================
int consigne = 3500;

int vitesseBase = 0; // Initialisé à 0 pour forcer la mise à jour au démarrage
float Kp = 0.02;
float Kd = 0.1;
int lastError = 0;

// ==========================================
// 5. SOUS-PROGRAMMES (LES ACTIONS DU ROBOT)
// ==========================================

void actualiserVitesseEtPID(int nouvelleVitesse) {
  if (vitesseBase != nouvelleVitesse) {
    vitesseBase = nouvelleVitesse;

    // Auto-ajustement des amortisseurs en fonction de la vitesse (Base 45)
    float ratio = (float)vitesseBase / 45.0;
    Kp = 0.02 * ratio;
    Kd = 0.10 * ratio;

    Serial.print(">>> NOUVELLE ZONE - Vitesse : "); Serial.print(vitesseBase);
    Serial.print(" | Kp : "); Serial.print(Kp, 4);
    Serial.print(" | Kd : "); Serial.println(Kd, 4);
  }
}

void faireDemiTour(char direction) {
  Serial.print("!!! MANOEUVRE EN COURS : PIVOT VERS ");
  Serial.println(direction);

  int vitesseRotation = 52;

  if (direction == 'D') {
    digitalWrite(pinMoteurGaucheA, HIGH);
    digitalWrite(pinMoteurGaucheB, LOW);
    analogWrite(pinVitesseGauche, vitesseRotation);
    digitalWrite(pinMoteurDroitA, LOW);
    digitalWrite(pinMoteurDroitB, HIGH);
    analogWrite(pinVitesseDroit, vitesseRotation);
  }
  else if (direction == 'G') {
    digitalWrite(pinMoteurGaucheA, LOW);
    digitalWrite(pinMoteurGaucheB, HIGH);
    analogWrite(pinVitesseGauche, vitesseRotation);
    digitalWrite(pinMoteurDroitA, HIGH);
    digitalWrite(pinMoteurDroitB, LOW);
    analogWrite(pinVitesseDroit, vitesseRotation);
  }

  delay(200);

  while (true) {
    qtr.readLineBlack(sensorValues);
    if (sensorValues[3] > 600 || sensorValues[4] > 600) {
      break;
    }
  }

  analogWrite(pinVitesseGauche, 0);
  analogWrite(pinVitesseDroit, 0);
  delay(200);
}

void suivreLignePID() {
  unsigned int positionActuelle = qtr.readLineBlack(sensorValues);

  int erreur = positionActuelle - consigne;
  int correction = (Kp * erreur) + (Kd * (erreur - lastError));
  lastError = erreur;

  int vitesseGauche = vitesseBase + correction;
  int vitesseDroit = vitesseBase - correction;

  if (vitesseGauche < 0) vitesseGauche = 0;
  if (vitesseGauche > 255) vitesseGauche = 255;
  if (vitesseDroit < 0) vitesseDroit = 0;
  if (vitesseDroit > 255) vitesseDroit = 255;

  digitalWrite(pinMoteurGaucheA, HIGH);
  digitalWrite(pinMoteurGaucheB, LOW);
  analogWrite(pinVitesseGauche, vitesseGauche);

  digitalWrite(pinMoteurDroitA, HIGH);
  digitalWrite(pinMoteurDroitB, LOW);
  analogWrite(pinVitesseDroit, vitesseDroit);
}

// ==========================================
// 6. INITIALISATION
// ==========================================
void setup() {
  pinMode(pinMoteurGaucheA, OUTPUT);
  pinMode(pinMoteurGaucheB, OUTPUT);
  pinMode(pinVitesseGauche, OUTPUT);
  pinMode(pinMoteurDroitA, OUTPUT);
  pinMode(pinMoteurDroitB, OUTPUT);
  pinMode(pinVitesseDroit, OUTPUT);

  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){25, 27, 29, 31, 33, 35, 37, 41}, SensorCount);
  qtr.setEmitterPin(255);

  Serial.println("CALIBRATION (5 SECONDES)...");
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
    analogWrite(pinVitesseGauche, 0);
    analogWrite(pinVitesseDroit, 0);
    return;
  }

  qtr.readLineBlack(sensorValues);

  // --- SÉCURITÉ ANTI-SORTIE DE PISTE ---
  bool toutBlanc = true;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 300) {
      toutBlanc = false;
      break;
    }
  }
  if (toutBlanc == true) {
    analogWrite(pinVitesseGauche, 0);
    analogWrite(pinVitesseDroit, 0);
    return;
  }

  // --- COMPTAGE GAUCHE ---
  if (sensorValues[0] > 750 && surLigneGauche == false) {
    compteurGauche++;
    surLigneGauche = true;
  }
  else if (sensorValues[0] < 300) {
    surLigneGauche = false;
  }

  // --- COMPTAGE DROIT ---
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

      // 1. Dégagement de la ligne
      if (tempoActuelle > 0) {
        unsigned long tempsDepart = millis();
        while (millis() - tempsDepart < tempoActuelle) {
          suivreLignePID();
        }
      }

      // 2. Exécution de la pause ciblée
      if (pauseActuelle > 0) {
        Serial.print("!!! PAUSE DE "); Serial.print(pauseActuelle); Serial.println(" ms !!!");
        analogWrite(pinVitesseGauche, 0);
        analogWrite(pinVitesseDroit, 0);
        delay(pauseActuelle);
      }

      // 3. Exécution de l'action selon la partition
      if (typeActionDemandee == 'T') {
        faireDemiTour(sensDeLaRotation);
      }
      else if (typeActionDemandee == 'V') {
        Serial.println("!!! FRANCHISSEMENT DE ZONE TERMINE !!!");
      }

      // 4. Validation de l'étape et préparation de la suivante
      indexEtapeActuelle++;
      compteurGauche = 0;
      compteurDroite = 0;

      actualiserVitesseEtPID(vitessesEtapes[indexEtapeActuelle]);

      return;
    }
  }

  // --- ARRÊT FINAL ---
  if (indexEtapeActuelle >= totalEtapes) {
    if (cibleArretFinale > 0 && (compteurDroite >= cibleArretFinale || compteurGauche >= cibleArretFinale)) {

      if (tempoAvantArret > 0) {
        unsigned long tempsDepart = millis();
        while (millis() - tempsDepart < tempoAvantArret) {
          suivreLignePID();
        }
      }

      robotArrete = true;
      Serial.println("!!! ENCHAÎNEMENT TERMINÉ : ARRET TOTAL DU ROBOT !!!");

      return;
    }
  }

  // --- LE SUIVI DE LIGNE EN CONTINU ---
  suivreLignePID();
}
