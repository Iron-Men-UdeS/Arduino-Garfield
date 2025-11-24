#include "Communication.h"

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * Lit une trame sur UART1 et stocke les donnes dans un tableau
 *
 * @param trame (Tableau uint8_t) Addresse du tableau pour la trame recu
 * @param sizeTrame (uint8_t) longueur de la trame a recevoir (+2 pour start et checksum)
 ******************************************************************************************/
void litUART(uint8_t *trame, uint8_t sizeTrame)
{
    int somme = 0;
    uint8_t temporaire[sizeTrame - 1];
    int i;

    if (Serial1.available() >= sizeTrame)
    {
        Serial1.readBytes(temporaire, 1);
        if (temporaire[0] == 0x24)
        {
            Serial1.readBytes(temporaire, sizeTrame - 1);
            for (i = 0; i < sizeTrame - 2; i++)
            {
                somme = somme + temporaire[i];
            }
            if (temporaire[sizeTrame - 2] == somme)
            {
                for (i = 0; i < sizeTrame - 2; i++)
                {
                    trame[i] = temporaire[i];
                }
            }
        }
    }
}

/*******************************************************************************************
 * Auteur : Alexandre Dionne
 *
 * Initialisation du port UART1 a 115200 bauds
 *
 ******************************************************************************************/
void initUART1(void)
{
    Serial1.begin(115200);
}