#include "Niveau1.h"
#include "Capteurs.h"

//////////////////////////////////////////////////////////////////////////////////////
// Nom de la fonction : suivreLigne                                                          
// Auteur : Alexandre Dionne                                                                         
// Entree(s) : N/A
// Sortie : couleur
// Description : suit la ligne et quitte la fonction lorsqu'une couleur est lue
//////////////////////////////////////////////////////////////////////////////////////
int suivreLigne(void)
{
    unsigned long currentTime;
    unsigned long previousTime;

    float i0 = 1; // Indice correcteur moteur gauche
    float i1 = 1; // Indice correcteur moteur droit
    
    int ligne = 0;

    previousTime = millis();
    while(1)
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
                i0 = 0.75;
                i1 = 1;
                break;

            case 0x02: // 000 010  Petite correction vers la gauche
            case 0x06: // 000 110
                i0 = 0.90;
                i1 = 1;
                break;

            case 0x04: // 000 100  Aucune correction 
            case 0x0C: // 001 100
            case 0x08: // 001 000
                i0 = 1;
                i1 = 1;

                break;

            case 0x18: // 011 000  Petite correction vers la droite
            case 0x10: // 010 000
                i0 = 1;
                i1 = 0.90;
                break;

            case 0x30: // 110 000  Grosse correction vers la droite
            case 0x20: // 100 000
                i0 = 1;
                i1 = 0.75;
                break;

            case 0x3F: // 111 111 ligne partout  Arret complet
            case 0x00: // 000 000 pas de ligne
                i0 = 0;
                i1 = 0;
                break;
            }
            Serial.println(i0);
            Serial.println(i1);
            //Avance(VITESSE_MOTEUR*i0, VITESSE_MOTEUR*i1);
        }

        if(detectCouleur() != -1)
        {
          return detectCouleur();  
        }
    }
}