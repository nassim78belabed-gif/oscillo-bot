# L oscillo bot — robot suiveur de ligne (SAE Concours Cachan)

Code Arduino d'un robot autonome suiveur de ligne, développé en équipe dans le cadre de la SAE robotique du BUT GEII (IUT de Vélizy-Rambouillet) pour le Concours de Cachan (juin 2026).

**Équipe :** Nassim Belabed, Yanni Kadir, Gerald Binet

**Vidéos des missions et présentation du projet :** https://nassimbelabed.wixsite.com/sae-robotique

## Ce que fait le code

- **Suivi de ligne par PID** (termes proportionnel et dérivé) avec une barrette infrarouge Pololu de 8 capteurs (bibliothèque `QTRSensors`).
- **Séquenceur d'actions (« la partition »)** : le parcours est décrit dans des tableaux. Chaque colonne est une étape : combien de traits compter, de quel côté, quelle action faire, quel sens de rotation, quel temps de dégagement, quelle pause et quelle vitesse sur le tronçon suivant.
- **Actions disponibles :** `V` (ligne droite), `T` (demi-tour à gauche ou à droite), `S` (arrêt), `A` (avancer à l'aveugle pendant une durée donnée) et `C` (courbe à travers champ, dans les premières versions seulement).
- **Comptage de traits** à gauche et à droite avec un verrou anti-double comptage.
- **Détection automatique du type de piste** (ligne noire sur fond blanc, ou l'inverse) et bascule entre `readLineBlack` et `readLineWhite`.
- **Réglage automatique du Kp et du Kd** selon la vitesse du tronçon.
- **Sécurités :** arrêt si le robot reste plus de 200 ms hors de la piste, limitation de la correction PID, arrêt final au Nème trait.

## Matériel (d'après le code)

| Élément | Détail |
|---|---|
| Carte | Arduino |
| Capteurs | Barrette infrarouge QTR de 8 capteurs (type RC), broches 25, 27, 29, 31, 33, 35, 37, 41 |
| Moteur gauche | PWM sur la broche 3, sens sur les broches 7 et 6 |
| Moteur droit | PWM sur la broche 2, sens sur les broches 5 et 4 |
| Puissance | Pont en H pour les deux moteurs |

## Versions du code

Chaque dossier est un sketch Arduino indépendant. Ce sont les versions envoyées par Yanni par mail entre le 8 et le 13 juin 2026, dans l'ordre. Le nom de chaque dossier reprend la date et l'objet du mail.

| Dossier | Ce qui change dans cette version |
|---|---|
| `v01_..._code_booste_pro_max` | Première version : 7 étapes, pause de 2 s à chaque trait, PID simple, vitesse 45 |
| `v02_..._code_super_booste_avec_delay` | Ajout de l'action `C` (courbe à travers champ) et d'une tolérance de 200 ms hors piste avant l'arrêt |
| `v03_..._cachan` | Détection automatique de la piste inversée, limitation de la correction PID à 30, nouveaux Kp et Kd |
| `v04_..._mission_11` | Ajout de l'action `S` (arrêt demandé par la partition) |
| `v05_..._sans_objet` | Mail sans objet : 5 étapes |
| `v06_..._mission_6` | 5 étapes, vitesse de rotation de 45 |
| `v07_..._mission_7` | 4 étapes terminées par `S`, vitesse 40 |
| `v08_..._course` | Parcours en 2 étapes (dégagement de 18,2 s au départ), vitesses 50, 254 puis 45 |
| `v09_..._mission_5` | 5 étapes, vitesse 44 |
| `v10_..._mission_40` | Ajout de l'action `A` (avancer à l'aveugle) avec une durée par étape |
| `v11_..._vrai_mission_5` | 6 étapes, vitesse 44 |

## Utilisation

1. Installer la bibliothèque **QTRSensors** de Pololu depuis le gestionnaire de bibliothèques de l'Arduino IDE.
2. Ouvrir le dossier d'une version, puis téléverser le sketch sur la carte.
3. Au démarrage, le robot se calibre pendant environ 3 s : le passer sur le blanc **et** sur le noir. Il attend ensuite 2 s avant de partir.
4. Régler le parcours dans la section « séquenceur » du sketch (tableaux `ciblesMarqueurs`, `cotesCapteurs`, `typesActions`, etc.).

## Remarques

- Le code a été récupéré à partir des mails : les retours à la ligne ajoutés par la messagerie ont été corrigés à la main, rien d'autre n'a été modifié. Les sketches n'ont pas été recompilés après cette remise en forme.
- Dans `v02`, les tableaux ont plus de valeurs que `totalEtapes` (2) : cette version ne compile pas telle quelle.
- Les commentaires des anciennes versions ont parfois été copiés d'un parcours à l'autre et ne correspondent plus au code (par exemple « 1 seule étape » ou « 4ème trait »).
