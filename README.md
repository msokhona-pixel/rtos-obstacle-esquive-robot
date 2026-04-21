# Robot autonome avec ROS – Évitement d’obstacles

## Description
Ce projet consiste en le développement d’un comportement robotique autonome basé sur une architecture temps réel.
Le robot est capable de se déplacer de manière autonome et de réagir à son environnement en évitant les obstacles détectés par collision.
Le système repose sur une machine à états finis (FSM) permettant de structurer les différents comportements du robot.

---

## Fonctionnalités
- Mode autonome de déplacement
- Détection d’obstacles par capteur de collision
- réaction automatique :
  -arrêt
  - recul
  - rotation
-reprise du déplacement
- Architecture multi-comportements basée sur FSM

---

## Architecture du système

Le comportement du robot est organisé sous forme d’une machine à états finis :

### États principaux :
Contrôle manuel (joystick)
- Mode autonome
  Arrêt après collision
- Recul
  Rotation
- Reprise du déplacement

### Logique de fonctionnement :
Lors dela collision :
1. Le robot s’arrête
   Il recule pendant un court instant
3. Il effectue une rotation
4. Il reprend son déplacement autonome



## =Implémentation
- programmation en Python
- Utilisation de ROS (Robot Operating System)
- Communication via topics ROS (`cmd_vel`, `joy`, capteurs)
- Gestion des comportements avec une machine à états finis (FSM)


##Technologies utilisées
- python
- ROS
- Machine à états finis
- Systèmes embarqué


## Démonstration
Vidéo disponible 


## Objectif
Ce projet illustre la conception d’un comportement robotique réactif en environnement réel, en combinant systèmes embarqués, logique de contrôle et architecture logicielle structurée.
