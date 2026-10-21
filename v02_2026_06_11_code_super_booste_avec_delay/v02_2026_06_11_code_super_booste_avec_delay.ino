// Importation de la bibliothèque externe Pololu pour faire fonctionner la barrette infrarouge.
#include <QTRSensors.h>

// ==========================================
// 1. DÉCLARATIONS DES BROCHES MOTEURS
// ==========================================
// Moteur Gauche (Câblage vers le pont en H, type L298N ou TB6612FNG)
const int pinVitesseGauche = 3; // Broche PWM (avec le petit '~' sur l'Arduino) pour gérer la puissance (0 à 255)
const int pinMoteurGaucheA = 7; // Broche logique (0 ou 1) pour dicter le sens de rotation 1
const int pinMoteurGaucheB = 6; // Broche logique (0 ou 1) pour dicter le sens de rotation 2

// Moteur Droit
const int pinMoteurDroitA = 5; // Broche logique sens 1
const int pinMoteurDroitB = 4; // Broche logique sens 2
const int pinVitesseDroit = 2; // Broche PWM pour la puissance

// ==========================================
// 2. CONFIGURATION DU CAPTEUR QTR
// ==========================================
QTRSensors qtr; // On crée un "objet" nommé qtr qui représente notre barrette physique
const uint8_t SensorCount = 8; // On indique qu'il y a 8 capteurs infrarouges sur la barrette
uint16_t sensorValues[SensorCount]; // On crée un tableau vide de 8 cases pour stocker les futures lectures (de 0 à 1000)

// ==========================================
// 3. LE SÉQUENCEUR D'ACTIONS (LA PARTITION)
// ==========================================
const int totalEtapes = 2; // Le nombre d'actions que le robot doit faire avant de repasser en mode "suivi normal"
int indexEtapeActuelle = 0; // Le pointeur qui dit au robot où il en est (commence à 0)

// --- PROGRAMMATION DES ÉTAPES (Chaque colonne = 1 étape) ---
// Étape 0 : Croiser 1 trait à Gauche -> Ne rien faire ('V') -> Vitesse à 50
// Étape 1 : Croiser 1 trait à Droite -> Couper à travers champ ('C') -> Vitesse à 50
int ciblesMarqueurs[totalEtapes] = {1, 2, 4, 3}; // Combien de traits à compter avant de déclencher l'action
char cotesCapteurs[totalEtapes] = {'D', 'D','D','G'}; // Quel capteur extérieur écouter ('G'auche ou 'D'roite)
char typesActions[totalEtapes] = {'T', 'T','T','T'}; // Quelle action exécuter ('V'=Vitesse continue, 'C'=Couper l'herbe)
char sensRotations[totalEtapes] = {'G', 'D','D','G'}; // Sens du demi-tour (inutile ici car on ne fait pas l'action 'T')
int temposAvantAction[totalEtapes] = {0, 0, 0, 0, 0 }; // Temps (en ms) où le robot roule SUR le trait avant d'agir
int tempsPauses[totalEtapes] = {0, 0, 0, 0, 0}; // Temps d'arrêt total sur le trait (0 = on trace sans s'arrêter)

// --- LES VITESSES PAR TRONÇON ---
// Tableau contenant 3 vitesses car il y a 2 étapes (Départ, entre les 2 étapes, et après la dernière étape)
int vitessesEtapes[totalEtapes + 1] = {50, 50, 50};

// --- ARRÊT TOTAL (LIGNE D'ARRIVÉE) ---
// Ces paramètres ne s'activeront que lorsque l'indexEtapeActuelle aura atteint "totalEtapes" (donc la fin du parcours)
int cibleArretFinale = 1; // S'arrête dès qu'il voit 1 trait (la grande ligne bleue d'arrivée)
int tempoAvantArret = 400; // Avance 400ms à l'aveugle pour percuter/pousser la barrière en bois avant de freiner

// --- VARIABLES DE MÉMOIRE (Les compteurs internes du robot) ---
int compteurGauche = 0; // Nombre de traits vus à gauche
int compteurDroite = 0; // Nombre de traits vus à droite
bool surLigneGauche = false; // "Verrou" pour éviter de compter le même trait 1000 fois par seconde s'il est épais
bool surLigneDroite = false;
bool robotArrete = false; // Drapeau général : si true, le programme complet est stoppé

// === SÉCURITÉ ANTI-SORTIE DE PISTE ===
unsigned long chronoDerniereLigne = 0; // Chronomètre interne de l'Arduino (stocke des millisecondes)
const int tempsToleranceBlanc = 200; // Autorise le robot à être "hors-piste" pendant 200ms avant de tout couper

// ==========================================
// 4. RÉGLAGES DU CERVEAU (CONTRÔLE PID)
// ==========================================
int consigne = 3500; // Le centre théorique de la barrette (capteurs de 0 à 7000, le milieu est 3500)
int vitesseBase = 0; // Vitesse d'avancement brute. Sera écrasée par actualiserVitesseEtPID()
float Kp = 0.02; // Coefficient Proportionnel : Force du coup de volant pour revenir vers la ligne
float Kd = 0.1; // Coefficient Dérivé : Amortisseur pour éviter que le robot oscille et tremble
int lastError = 0; // Mémoire de l'erreur du tour précédent (nécessaire pour calculer le Kd)

// ==========================================
// 5. SOUS-PROGRAMMES (LES ACTIONS DU ROBOT)
// ==========================================

// Fonction qui ajuste le Kp et le Kd automatiquement selon la vitesse pour garder le robot stable
void actualiserVitesseEtPID(int nouvelleVitesse) {
  if (vitesseBase != nouvelleVitesse) { // Ne recalcule que s'il y a un vrai changement de vitesse
    vitesseBase = nouvelleVitesse;
    float ratio = (float)vitesseBase / 45.0; // Règle de trois : si on va plus vite que 45, on durcit les réglages
    Kp = 0.02 * ratio;
    Kd = 0.10 * ratio;
  }
}

// Fonction pour faire tourner le robot sur lui-même
void faireDemiTour(char direction) {
  int vitesseRotation = 52; // Vitesse fixée pour ne pas déraper

  if (direction == 'D') { // Commande le pont en H pour faire tourner les chenilles/roues en sens opposé
    digitalWrite(pinMoteurGaucheA, HIGH); digitalWrite(pinMoteurGaucheB, LOW); // Roue G avance
    analogWrite(pinVitesseGauche, vitesseRotation);
    digitalWrite(pinMoteurDroitA, LOW); digitalWrite(pinMoteurDroitB, HIGH); // Roue D recule
    analogWrite(pinVitesseDroit, vitesseRotation);
  } else if (direction == 'G') {
    digitalWrite(pinMoteurGaucheA, LOW); digitalWrite(pinMoteurGaucheB, HIGH); // Roue G recule
    analogWrite(pinVitesseGauche, vitesseRotation);
    digitalWrite(pinMoteurDroitA, HIGH); digitalWrite(pinMoteurDroitB, LOW); // Roue D avance
    analogWrite(pinVitesseDroit, vitesseRotation);
  }

  delay(200); // Laisse le robot tourner un peu dans le vide pour sortir de la ligne noire initiale

  while (true) { // Boucle infinie : le robot continue de tourner sur lui-même...
    qtr.readLineBlack(sensorValues); // ...il lit le sol à chaque instant...
    if (sensorValues[3] > 600 || sensorValues[4] > 600) break; // ...et DÈS QUE les capteurs du milieu revoient le noir, il casse la boucle !
  }

  analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0); // Coupe les moteurs pour stopper la rotation
  delay(200); // Pause pour dissiper l'inertie
}

// === FONCTION MISSION 24 : LA COURBE DANS L'HERBE ===
void couperTraversChamp() {
  Serial.println("!!! MODE BULLDOZER : COURBE A GAUCHE JUSQU'A LA LIGNE !!!");

  // 1. On donne une consigne asymétrique pour forcer le robot à faire un grand arc de cercle à gauche
  digitalWrite(pinMoteurGaucheA, HIGH); digitalWrite(pinMoteurGaucheB, LOW); // Marche avant G
  analogWrite(pinVitesseGauche, 30); // Vitesse faible à l'intérieur du virage

  digitalWrite(pinMoteurDroitA, HIGH); digitalWrite(pinMoteurDroitB, LOW); // Marche avant D
  analogWrite(pinVitesseDroit, 90); // Vitesse forte à l'extérieur pour pousser le robot

  // 2. On attend 400ms sans rien faire. Ça permet de rouler et de s'éloigner physiquement de la ligne de départ.
  delay(400);

  // 3. On entre en mode "Recherche" :
  while (true) {
    qtr.readLineBlack(sensorValues); // On scanne le sol blanc

    // Si les capteurs centraux détectent une grosse valeur (> 500 = ligne noire de l'arrivée)...
    if (sensorValues[3] > 500 || sensorValues[4] > 500) {
      break; // ...On "casse" le while(true) pour sortir de la fonction !
    }
  }
  // Fin de la fonction. Le robot garde sa vitesse d'élan et le programme principal reprendra la main.
}

// Fonction centrale : Maintien du robot sur la ligne noire
void suivreLignePID() {
  // readLineBlack donne une valeur entre 0 (tout à gauche) et 7000 (tout à droite). 3500 est le centre.
  unsigned int positionActuelle = qtr.readLineBlack(sensorValues);

  // Calcul de l'écart. Si on est à 4000, l'erreur est de +500 (le robot dérive à droite).
  int erreur = positionActuelle - consigne;

  // L'algorithme mathématique : Corrige proportionnellement à l'erreur (Kp) et anticipe les tremblements (Kd)
  int correction = (Kp * erreur) + (Kd * (erreur - lastError));
  lastError = erreur; // Sauvegarde l'erreur pour le prochain tour de piste

  // On applique la correction sur la vitesse de base des roues
  int vitesseGauche = vitesseBase + correction;
  int vitesseDroit = vitesseBase - correction;

  // Protections pour ne pas envoyer de valeurs absurdes au pont en H (qui n'accepte que 0 à 255)
  if (vitesseGauche < 0) vitesseGauche = 0;
  if (vitesseGauche > 255) vitesseGauche = 255;
  if (vitesseDroit < 0) vitesseDroit = 0;
  if (vitesseDroit > 255) vitesseDroit = 255;

  // On envoie le courant adapté aux moteurs
  digitalWrite(pinMoteurGaucheA, HIGH); digitalWrite(pinMoteurGaucheB, LOW);
  analogWrite(pinVitesseGauche, vitesseGauche);
  digitalWrite(pinMoteurDroitA, HIGH); digitalWrite(pinMoteurDroitB, LOW);
  analogWrite(pinVitesseDroit, vitesseDroit);
}

// ==========================================
// 6. INITIALISATION (Exécutée 1 seule fois au démarrage de la carte)
// ==========================================
void setup() {
  // Déclare que l'Arduino doit ENVOYER du courant sur ces broches
  pinMode(pinMoteurGaucheA, OUTPUT); pinMode(pinMoteurGaucheB, OUTPUT); pinMode(pinVitesseGauche, OUTPUT);
  pinMode(pinMoteurDroitA, OUTPUT); pinMode(pinMoteurDroitB, OUTPUT); pinMode(pinVitesseDroit, OUTPUT);

  Serial.begin(9600); // Ouvre la communication avec le PC pour le débogage

  qtr.setTypeRC(); // Précise à la bibliothèque qu'on utilise des capteurs analogiques (RC = Résistance/Capacité)
  qtr.setSensorPins((const uint8_t[]){25, 27, 29, 31, 33, 35, 37, 41}, SensorCount); // Liste les broches de la barrette
  qtr.setEmitterPin(255); // Désactive la broche de contrôle des LED (elles sont toujours allumées)

  Serial.println("CALIBRATION...");
  delay(1000);

  // Boucle de calibration de 5 secondes : balaye le sol pour que le robot apprenne ce qu'est le "blanc" et le "noir" du jour
  for (uint16_t i = 0; i < 150; i++) {
    qtr.calibrate();
    delay(20);
  }

  actualiserVitesseEtPID(vitessesEtapes[0]); // Charge la vitesse de la toute première étape de notre partition
  Serial.println("GO !");
  delay(2000); // Temps de sécurité pour retirer tes mains du robot avant qu'il ne parte
}

// ==========================================
// 7. BOUCLE PRINCIPALE (Exécutée en boucle à l'infini)
// ==========================================
void loop() {

  // Si le drapeau de fin de course est levé, on force les moteurs à zéro et on stop la boucle avec "return;"
  if (robotArrete == true) {
    analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
    return;
  }

  qtr.readLineBlack(sensorValues); // Actualise la vision des capteurs

  // --- SÉCURITÉ ANTI-SORTIE DE PISTE ---
  bool toutBlanc = true; // On part du principe qu'on est hors-piste...
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 300) { toutBlanc = false; break; } // ...Sauf si un capteur lit du sombre.
  }

  if (toutBlanc == false) {
    chronoDerniereLigne = millis(); // Si on est sur la piste, on rafraîchit l'heure du chronomètre
  }
  else {
    // Si on est dans le vide (herbe blanche), on vérifie si ça fait plus de 200 millisecondes
    if (millis() - chronoDerniereLigne > tempsToleranceBlanc) {
      analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0); // Si oui, c'est un crash, on coupe tout !
      return;
    }
    // Si on est dans le vide depuis seulement 50ms, le programme continue et ignore l'erreur !
  }

  // --- COMPTAGE GAUCHE ---
  // Si le capteur 0 voit du noir ET qu'il n'était pas déjà dessus au tour de boucle précédent
  if (sensorValues[0] > 750 && surLigneGauche == false) {
    compteurGauche++; // On ajoute +1 au compteur
    surLigneGauche = true; // On verrouille pour ne plus compter
  }
  else if (sensorValues[0] < 300) {
    surLigneGauche = false; // On est de retour sur le blanc, on déverrouille pour le prochain trait !
  }

  // --- COMPTAGE DROIT ---
  if (sensorValues[7] > 750 && surLigneDroite == false) {
    compteurDroite++;
    surLigneDroite = true;
  }
  else if (sensorValues[7] < 300) {
    surLigneDroite = false;
  }

  // --- LE SÉQUENCEUR D'ACTIONS (L'Intelligence du parcours) ---
  if (indexEtapeActuelle < totalEtapes) {
    // On charge les consignes de l'étape en cours depuis nos tableaux (Section 3)
    int cibleMarqueurAttendu = ciblesMarqueurs[indexEtapeActuelle];
    char coteCapteurASurveiller = cotesCapteurs[indexEtapeActuelle];
    char typeActionDemandee = typesActions[indexEtapeActuelle];

    // === LA LIGNE CORRIGÉE EST ICI ===
    char sensDeLaRotation = sensRotations[indexEtapeActuelle];

    int tempoActuelle = temposAvantAction[indexEtapeActuelle];
    int pauseActuelle = tempsPauses[indexEtapeActuelle];

    // On sélectionne le bon compteur à lire selon la demande
    int compteurEnCours = (coteCapteurASurveiller == 'D') ? compteurDroite : compteurGauche;

    // LE MOMENT CLÉ : Si le robot a croisé assez de traits pour valider l'étape
    if (compteurEnCours >= cibleMarqueurAttendu) {

      // 1. Dégagement : Laisse le robot rouler un peu sur l'intersection pour bien se positionner
      if (tempoActuelle > 0) {
        unsigned long tempsDepart = millis();
        while (millis() - tempsDepart < tempoActuelle) {
          suivreLignePID();
        }
      }

      // 2. Pause au stand : Si on a programmé un arrêt sur le trait
      if (pauseActuelle > 0) {
        analogWrite(pinVitesseGauche, 0); analogWrite(pinVitesseDroit, 0);
        delay(pauseActuelle);
      }

      // 3. Exécution de l'action physique
      if (typeActionDemandee == 'T') {
        faireDemiTour(sensDeLaRotation);
      }
      else if (typeActionDemandee == 'V') {
        Serial.println(">>> TRONÇON DE LIGNE DROITE VALIDE <<<"); // Ne fait rien de spécial
      }
      else if (typeActionDemandee == 'C') {
        couperTraversChamp(); // Lance la courbe à l'aveugle.
      }

      // 4. Validation : L'action est finie, on passe à l'index suivant et on remet les compteurs à 0
      indexEtapeActuelle++;
      compteurGauche = 0;
      compteurDroite = 0;
      actualiserVitesseEtPID(vitessesEtapes[indexEtapeActuelle]); // Change la vitesse pour le prochain tronçon
      return; // Casse le cycle actuel du loop pour reprendre proprement d'en haut
    }
  }

  // --- ARRÊT FINAL CLASSIQUE ---
  // On ne rentre ici que si on a terminé toutes les étapes de la partition (index >= totalEtapes)
  if (indexEtapeActuelle >= totalEtapes) {
    // Si on capte le nombre de traits final demandé (cibleArretFinale)
    if (cibleArretFinale > 0 && (compteurDroite >= cibleArretFinale || compteurGauche >= cibleArretFinale)) {

      // Dégagement pour passer la ligne bleue (la barrière dans notre cas)
      if (tempoAvantArret > 0) {
        unsigned long tempsDepart = millis();
        while (millis() - tempsDepart < tempoAvantArret) {
          suivreLignePID(); // Continue de foncer tout droit avec le PID actif !
        }
      }

      robotArrete = true; // Valide la fin totale du programme. Au prochain tour de boucle, tout sera stoppé.
      Serial.println("!!! BARRIÈRE ATTEINTE : ARRÊT TOTAL !!!");
      return;
    }
  }

  // Comportement par défaut : si le robot n'a aucune action spécifique à faire, il suit sa ligne noire !
  suivreLignePID();
}
