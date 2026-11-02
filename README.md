# L oscillo bot : robot suiveur de ligne (SAE Concours Cachan)

Robot autonome suiveur de ligne, développé en équipe dans le cadre de la SAE robotique du BUT GEII (IUT de Vélizy-Rambouillet) pour le Concours de Cachan (juin 2026).

**Équipe :** Nassim Belabed, Yanni Kadir, Gerald Binet

**Vidéos des missions et présentation du projet :** https://nassimbelabed.wixsite.com/sae-robotique

> Le code source n'est pas publié : c'est le travail de l'équipe pour un concours. Il peut être montré sur demande.

## Ce que fait le robot

- **Suivi de ligne par PID** (proportionnel + dérivé) avec une barrette infrarouge Pololu de 8 capteurs (bibliothèque `QTRSensors`).
- **Séquenceur d'actions (« la partition »)** : le parcours est décrit étape par étape (traits à compter, côté, action, sens de rotation, temporisations, vitesse du tronçon).
- **Actions :** ligne droite, demi-tour gauche/droite, arrêt, avance à l'aveugle.
- **Comptage de traits** à gauche et à droite avec anti-double comptage.
- **Détection automatique du type de piste** (noir sur blanc ou l'inverse).
- **Réglage automatique du PID** selon la vitesse du tronçon.
- **Sécurités :** arrêt en cas de sortie de piste prolongée, limitation de la correction.

## Matériel

| Élément | Détail |
|---|---|
| Carte | Arduino |
| Capteurs | Barrette infrarouge QTR de 8 capteurs (type RC) |
| Moteurs | 2 moteurs CC pilotés en PWM |
| Puissance | Pont en H |

## Organisation du programme

| Fonction | Rôle |
|---|---|
| `setup()` | Initialisation et calibration des capteurs |
| `loop()` | Lecture de la piste, comptage des traits, déroulé de la partition |
| `lirePisteEtDetecterMode()` | Lecture de la position de la ligne + détection piste inversée |
| `suivreLignePID()` | Correction PID et commande des moteurs |
| `actualiserVitesseEtPID()` | Changement de vitesse et adaptation du PID par tronçon |
| `faireDemiTour()` | Demi-tour à gauche ou à droite |
| `avancerAveugle()` | Avance sans suivi de ligne pendant une durée donnée |

11 versions ont été écrites entre le 8 et le 13 juin 2026, une par mission ou presque.
