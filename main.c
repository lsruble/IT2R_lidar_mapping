#include "LPC17xx.h"
#include "Driver_USART.h"
#include "Board_GLCD.h"
#include "GLCD_Config.h"
#include "stdio.h"
#include <math.h>

extern ARM_DRIVER_USART Driver_USART1;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

// Définition des constantes pour la communication et le traitement des données
#define START_CHAR 0xA5
#define START_SCAN 0x20
#define STOP_SCAN 0x25
#define DATA_SIZE 2200
#define ANGLE_COUNT 400
#define DISPLAY_POINTS 72

// Déclarations des fonctions
void Init_PWM(void);
void Init_UART(void);
void Ligne(int x0, int y0, int x1, int y1);
int findDataStart(const char* data);
void processAngleData(const char* data, int start, float* angles);
void processDistanceData(const char* data, int start, float* distances);
void processQualityData(const char* data, int start, char* quality);
void calculateAverageMeasurements(const float* angles, const float* distances, float* averages);
void updateDisplay(float* averages, float* posX, float* posY);

int main(void) {
    // Déclaration des tableaux pour stocker les données
    char data[DATA_SIZE];
    float angles[ANGLE_COUNT], distances[ANGLE_COUNT];
    char quality[ANGLE_COUNT];
    float averageMeasurements[DISPLAY_POINTS];
    float displayPosX[DISPLAY_POINTS] = {0}, displayPosY[DISPLAY_POINTS] = {0};

    // Initialisation des périphériques
    Init_PWM();
    Init_UART();
    GLCD_Initialize();
    GLCD_ClearScreen();
    GLCD_SetFont(&GLCD_Font_6x8);

    while(1) {
        // Envoi des commandes de démarrage
        while(Driver_USART1.GetStatus().tx_busy == 1);
        Driver_USART1.Send(&START_CHAR, 1);
        while(Driver_USART1.GetStatus().tx_busy == 1);
        Driver_USART1.Send(&START_SCAN, 1);

        // Réception des données
        Driver_USART1.Receive(data, DATA_SIZE);
        while (Driver_USART1.GetRxCount() < DATA_SIZE);

        // Envoi des commandes d'arrêt
        while(Driver_USART1.GetStatus().tx_busy == 1);
        Driver_USART1.Send(&START_CHAR, 1);
        while(Driver_USART1.GetStatus().tx_busy == 1);
        Driver_USART1.Send(&STOP_SCAN, 1);

        // Traitement des données reçues
        int dataStart = findDataStart(data);
        processAngleData(data, dataStart, angles);
        processDistanceData(data, dataStart, distances);
        processQualityData(data, dataStart, quality);

        // Calcul des moyennes et mise à jour de l'affichage
        calculateAverageMeasurements(angles, distances, averageMeasurements);
        updateDisplay(averageMeasurements, displayPosX, displayPosY);
    }

    return 0;
}

// Fonction pour trouver le début des données utiles dans la trame reçue
int findDataStart(const char* data) {
    for(int i = 0; i < 50; i++) {
        // Recherche d'un motif spécifique indiquant le début des données
        if ((data[i] == 0x02 && data[i+5] == 0x02) ||
            (data[i] == 0x3e && data[i+5] == 0x02) ||
            (data[i] == 0x02 && data[i+5] == 0x3e) ||
            (data[i] == 0x3e && data[i+5] == 0x3e)) {
            return i;
        }
    }
    return -1;  // Erreur : début non trouvé
}

// Fonction pour extraire les données d'angle
void processAngleData(const char* data, int start, float* angles) {
    for(int i = 0; i < ANGLE_COUNT; i++) {
        // Extraction et conversion des données d'angle
        angles[i] = ((data[(start+1)+i*5] & 0xFE) + data[(start+2)+i*5]*128) / 64.0f;
    }
}

// Fonction pour extraire les données de distance
void processDistanceData(const char* data, int start, float* distances) {
    for(int i = 0; i < ANGLE_COUNT; i++) {
        // Extraction et conversion des données de distance
        distances[i] = (data[(start+3)+i*5] + data[(start+4)+i*5]*256) / 65535.0f * 6 * 2;
    }
}

// Fonction pour extraire les données de qualité
void processQualityData(const char* data, int start, char* quality) {
    for(int i = 0; i < ANGLE_COUNT; i++) {
        quality[i] = data[start+i*5];
    }
}

// Fonction pour calculer les moyennes des mesures par tranche de 5 degrés
void calculateAverageMeasurements(const float* angles, const float* distances, float* averages) {
    for (int i = 0; i < DISPLAY_POINTS; i++) {
        int count = 0;
        averages[i] = 0;
        for (int j = 0; j < ANGLE_COUNT; j++) {
            // Vérification si l'angle est dans la tranche de 5 degrés actuelle
            if (angles[j] > (5*i) && angles[j] < (5*i+5)) {
                count++;
                averages[i] += distances[j];
            }
        }
        if (count > 0) {
            averages[i] /= count;
        }
    }
}

// Fonction pour mettre à jour l'affichage sur l'écran LCD
void updateDisplay(float* averages, float* posX, float* posY) {
    // Effacement des anciennes lignes
    GLCD_SetForegroundColor(GLCD_COLOR_WHITE);
    for (int i = 0; i < DISPLAY_POINTS; i++) {
        Ligne(160, 120, (int)posX[i], (int)posY[i]);
    }

    // Dessin des nouvelles lignes
    GLCD_SetForegroundColor(GLCD_COLOR_BLACK);
    for (int i = 0; i < DISPLAY_POINTS; i++) {
        float angle = i * 0.0872664f;  // 5 degrés en radians
        // Calcul des coordonnées d'affichage
        if (averages[i] > 2) {
            posX[i] = 160 + 110 * cosf(angle);
            posY[i] = 120 + 110 * sinf(angle);
        } else {
            posX[i] = 160 + 55 * averages[i] * cosf(angle);
            posY[i] = 120 + 55 * averages[i] * sinf(angle);
        }
        Ligne(160, 120, (int)posX[i], (int)posY[i]);
    }
}

// Fonction d'initialisation du PWM
void Init_PWM(void) {
    LPC_SC->PCONP |= (1 << 6);  // Activation du périphérique PWM1
    LPC_PINCON->PINSEL4 |= (1 << 4);  // Configuration de P2.2 en mode PWM1.3
    LPC_PWM1->CTCR = 0;  // Mode de comptage : périodes d'horloge
    LPC_PWM1->PR = 0;  // Pas de prescaler
    LPC_PWM1->MR0 = 999;  // Valeur max du comptage pour la période PWM
    LPC_PWM1->MR3 = 600;  // Valeur pour le rapport cyclique de 41% sur la sortie 3
    LPC_PWM1->MCR |= (1 << 1);  // RAZ du compteur si correspondance avec MR0
    LPC_PWM1->PCR |= (1 << 11);  // Activation de la sortie PWM1.3
    LPC_PWM1->TCR = 1;  // Démarrage du comptage du Timer
}

// Fonction d'initialisation de l'UART
void Init_UART(void) {
    Driver_USART1.Initialize(NULL);
    Driver_USART1.PowerControl(ARM_POWER_FULL);
    Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8 |
                          ARM_USART_STOP_BITS_1 |
                          ARM_USART_PARITY_NONE |
                          ARM_USART_FLOW_CONTROL_NONE,
                          115200);
    Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
    Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);
}

// Fonction pour dessiner une ligne sur l'écran LCD
void Ligne(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (1) {
        GLCD_DrawRectangle(x0, y0, 1, 1);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
