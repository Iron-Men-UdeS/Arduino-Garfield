#include "Niveau1.h"
#include "Capteurs.h"
#include "Mouvement.h"

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * suit la ligne détectee par les capteurs de contraste
 *
 * @return couleur (integer) valeur de la coueleur lue
 ******************************************************************************************/
int suivreLigne(void)
{
    unsigned long currentTime;
    unsigned long previousTime;

    float i0 = 1; // Indice correcteur moteur gauche
    float i1 = 1; // Indice correcteur moteur droit

    int ligne = 0;

    previousTime = millis();
    while (detectCouleur() == -1)
    {
        currentTime = millis();
        if ((currentTime - previousTime) >= INTERVALLE)
        {
            previousTime = currentTime;
            ligne = lireCapteurs(0);
            ligne = (lireCapteurs(1) << 3) + ligne;

            switch (ligne)
            {
            case 0x01: // 000 001  Grosse correction vers la gauche
            case 0x03: // 000 011
                       // Serial.println("1");
                i0 = -0.50;
                i1 = 1;
                break;

            case 0x02: // 000 010  Petite correction vers la gauche
            case 0x06: // 000 110
                       // Serial.println("2");
                i0 = -0.25;
                i1 = 1;
                break;

            case 0x04: // 000 100  Aucune correction
                i0 = 0.90;
                i1 = 1;
                break;
            case 0x0C: // 001 100
                i0 = 1;
                i1 = 1;
                break;
            case 0x08: // 001 000
                i0 = 1;
                i1 = 0.90;
                // Serial.println("3");
                break;

            case 0x18: // 011 000  Petite correction vers la droite
            case 0x10: // 010 000
                       // Serial.println("4");
                i0 = 1;
                i1 = -0.25;
                break;

            case 0x30: // 110 000  Grosse correction vers la droite
            case 0x20: // 100 000
                       // Serial.println("5");
                i0 = 1;
                i1 = -0.50;
                break;

            case 0x3F: // 111 111 ligne partout  Arret complet
                // Serial.println("6");
                i0 = 0;
                i1 = 0;
                break;
            case 0x00: // 000 000 pas de ligne
                       // Serial.println("7");

                break;
            }
            // Serial.println(i0);
            // Serial.println(i1);
            vitesseRoues(VITESSE_MOTEUR * i0, VITESSE_MOTEUR * i1);
        }
    }
    return detectCouleur();
}

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * suit la ligne détecter par les capteurs de contraste
 *
 * @param positionA (integer) Position actuelle du robot
 *
 * @param positionF (integer) Position finale du robot
 ******************************************************************************************/
void changePlace(int positionA, int positionF)
{
}
