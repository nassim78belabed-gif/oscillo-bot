// Importation de la bibliothèque externe Pololu pour faire fonctionner la barrette infrarouge.
#include <QTRSensors.h>

// ==========================================
// 1. DÉCLARATIONS DES BROCHES MOTEURS
// ==========================================
// Moteur Gauche (Câblage vers le pont en H, type L298N ou TB6612FNG)
const int pinVitesseGauche = 3; // Broche PWM pour gérer la puissance (0 à 255)
const int pinMoteurGaucheA = 7; // Broche logique (0 ou 1) sens 1
const int pinMoteurGaucheB = 6; // Broche logique (0 ou 1) sens 2

// Moteur Droit
const int pinMoteurDroitA = 5; // Broche logique sens 1
const int pinMoteurDroitB = 4; // Broche logique sens 2
const int pinVitesseDroit = 2; // Broche PWM pour la puissance

// ==========================================
// 2. CONFIGURATION DU CAPTEUR QTR
// ==========================================
QTRSensors qtr; // On crée un "objet" nommé qtr qui représente notre barrette physique
const uint8_t SensorCount = 8; // On indique qu'il y a 8 capteurs infrarouges sur la barrette
uint16_t sensorValues[SensorCount]; // Tableau de 8 cases pour stocker les lectures (de 0 à 1000)

// ==========================================
// 3. LE SÉQUENCEUR D'ACTIONS (LA PARTITION)
// ==========================================
const int totalEtapes = 4; // Nombre total d'étapes programmées
int indexEtapeActuelle = 0; // Pointeur qui dit au robot où il en est (commence à 0)

// --- PROGRAMMATION DES ÉTAPES (Chaque colonne = 1 étape) ---
int ciblesMarqueurs[totalEtapes] = {5, 20, 4, 3}; // Traits à compter avant d'agir
char cotesCapteurs[totalEtapes] = {'D', 'D','D','G'}; // Côté à écouter ('G' ou 'D')
char typesActions[totalEtapes] = {'T', 'T','T','T'}; // Action ('T' = Demi-tour, 'V' = Ligne droite)
char sensRotations[totalEtapes] = {'G', 'D','D','G'}; // Sens du demi-tour ('G' ou 'D')
int temposAvantAction[totalEtapes] = {100, 0, 0, 0}; // Temps (ms) pour avancer sur le trait avant d'agir
int tempsPauses[totalEtapes] = {0, 0, 0, 0}; // Arrêt au stand sur le trait (0 = on trace)

// --- LES VITESSES PAR TRONÇON ---
int vitessesEtapes[totalEtapes + 1] = {50, 50, 50, 50, 50};

// --- ARRÊT TOTAL (LIGNE D'ARRIVÉE) ---
int cibleArretFinale = 10; // S'arrête au 10ème trait rencontré APRÈS toutes les étapes
int tempoAvantArret = 400; // Avance 400ms pour pousser la barrière finale

// --- VARIABLES DE MÉMOIRE (Compteurs internes) ---
int compteurGauche = 0;
int compteurDroite = 0;
bool surLigneGauche = false;
bool surLigneDroite = false;
bool robotArrete = false;

// === L'INTELLIGENCE DE PISTE ===
bool pisteInverse = false; // Par défaut, on démarre sur une piste normale (Ligne noire)

// === SÉCURITÉ ANTI-SORTIE DE PISTE ===
unsigned long chronoDerniereLigne = 0;
const int tempsToleranceBlanc = 200; // Tolérance de 200ms dans le vide avant de couper les moteurs

// ==========================================
// 4. RÉGLAGES DU CERVEAU (CONTRÔLE PID)
// ==========================================
int consigne = 3500; // Le centre théorique de la barrette (capteurs de 0 à 7000)
int vitesseBase = 0;
float Kp = 0.02; // Force du braquage (Proportionnel)
float Kd = 0.1; // Amortisseur de direction (Dérivé)
int lastError = 0;

// ==========================================
// 5. SOUS-PROGRAMMES (LES ACTIONS DU ROBOT)
// ==========================================

// --- L'INTELLIGENCE ÉVÉNEMENTIELLE (DÉTECTION AUTO) ---
// Cette fonction lit les capteurs et décide toute seule quel mode PID utiliser
unsigned int lirePisteEtDetecterMode() {
  unsigned int positionLue;

  if (pisteInverse == false) {
    // Mode Normal : On traque l'ombre (ligne noire)
    positionLue = qtr.readLineBlack(sensorValues);

    // ÉVÉNEMENT : Si le milieu voit du BLANC (<300) ET les bords du NOIR (>600)
    if ((sensorValues[3] < 300 || sensorValues[4] < 300) && (sensorValues[0] > 600 || sensorValues[7] > 600)) {
      pisteInverse = true; // On bascule le cerveau !
      Serial.println("!!! REFLEXE : PASSAGE AUTO EN PISTE INVERSEE !!!");
    }
  }
  else {
    // Mode Inversé : On traque la lumière (ligne blanche)
    positionLue = qtr.readLineWhite(sensorValues);

    // ÉVÉNEMENT : Si le milieu revoit du NOIR (>600) ET les bords du BLANC (<300)
    if ((sensorValues[3] > 600 || sensorValues[4] > 600) && (sensorValues[0] < 300 || sensorValues[7] < 300)) {
      pisteInverse = false; // On revient à la normale !
      Serial.println("!!! REFLEXE : RETOUR AUTO EN PISTE NORMALE !!!");
    }
  }
  return positionLue; // Renvoie la position calculée au système de direction
}

// --- ACTUALISATION DES RÉGLAGES ---
void actualiserVitesseEtPID(int nouvelleVitesse) {
  if (vitesseBase != nouvelleVitesse) {
    vitesseBase = nouvelleVitesse;

    // NOUVEAU RÉGLAGE ANTI-ACCOUPS :
    // On adoucit le braquage (Kp à 0.015) et on durcit l'amortisseur (Kd à 0.20)
    float ratio = (float)vitesseBase / 45.0;
    Kp = 0.015 * ratio;
    Kd = 0.20 * ratio;

    Serial.print(">>> NOUVELLE ZONE - Vitesse : "); Serial.println(vitesseBase);
  }
}

// --- ROTATION SUR PLACE ---
void faireDemiTour(char direction) {
  int vitesseRotation = 52; // Vitesse bridée pour ne pas déraper

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

  delay(200); // Laisse le robot sortir de sa ligne initiale

  while (true) {
    qtr.readLineBlack(sensorValues);
    if (sensorValues[3] > 600 || sensorValues[4] > 600) break; // Arrête de tourner dès qu'il recroise la ligne
  }

  analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
  delay(200);
}

// --- LE CERVEAU DE DIRECTION (PID AVEC BRIDAGE) ---
void suivreLignePID(unsigned int positionActuelle) {
  int erreur = positionActuelle - consigne;
  int correction = (Kp * erreur) + (Kd * (erreur - lastError));
  lastError = erreur;

  // === LA SÉCURITÉ ANTI-EMBALLEMENT ===
  // Interdit à une roue de donner un grand coup d'accélérateur lors d'une perte de ligne
  int limiteCorrection = 30; // La roue ne peut pas gagner ou perdre plus de 30 points d'un coup
  if (correction > limiteCorrection) { correction = limiteCorrection; }
  else if (correction < -limiteCorrection) { correction = -limiteCorrection; }
  // =====================================

  int vitesseGauche = vitesseBase + correction;
  int vitesseDroit = vitesseBase - correction;

  // Clamp de sécurité pour le pont en H (0 à 255 max)
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
      return; // Coupe les moteurs si on est dans le vide plus de 200ms
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

      // 1. Dégagement
      if (tempoActuelle > 0) {
        unsigned long tempsDepart = millis();
        while (millis() - tempsDepart < tempoActuelle) {
          unsigned int posDegagement = lirePisteEtDetecterMode();
          suivreLignePID(posDegagement);
        }
      }

      // 2. Pause
      if (pauseActuelle > 0) {
        analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
        delay(pauseActuelle);
      }

      // 3. Action
      if (typeActionDemandee == 'T') {
        faireDemiTour(sensDeLaRotation);
      }
      else if (typeActionDemandee == 'V') {
        Serial.println(">>> TRONÇON VALIDE <<<");
      }

      // 4. Validation
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

  // --- TRANSMISSION AU CERVEAU ---
  // On envoie la position calculée au moteur de suivi de ligne !
  suivreLignePID(positionDuRobot);
}
