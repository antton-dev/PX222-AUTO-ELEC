#include <stdint.h>

//// --- ADRESSES REGISTRES STM32F103 ---
//#define RCC_BASE      0x40021000
//#define GPIOA_BASE    0x40010800
//#define ADC1_BASE     0x40012400
//#define TIM1_BASE     0x40012C00
//
//// Horloges
//#define RCC_APB2ENR   (*(volatile uint32_t *)(RCC_BASE + 0x18))
//#define RCC_CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x04))
//
//// GPIOA
//#define GPIOA_CRL     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
//#define GPIOA_CRH     (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
//
//// ADC1 (PA5 est ADC_IN5 sur F103)
//#define ADC1_SR       (*(volatile uint32_t *)(ADC1_BASE + 0x00))
//#define ADC1_CR2      (*(volatile uint32_t *)(ADC1_BASE + 0x08))
//#define ADC1_SQR3     (*(volatile uint32_t *)(ADC1_BASE + 0x34))
//#define ADC1_DR       (*(volatile uint32_t *)(ADC1_BASE + 0x4C))
//
//// TIM1 (PWM sur PA8)
//#define TIM1_CR1      (*(volatile uint32_t *)(TIM1_BASE + 0x00))
//#define TIM1_PSC      (*(volatile uint32_t *)(TIM1_BASE + 0x28))
//#define TIM1_ARR      (*(volatile uint32_t *)(TIM1_BASE + 0x2C))
//#define TIM1_CCR1     (*(volatile uint32_t *)(TIM1_BASE + 0x34))
//#define TIM1_CCMR1    (*(volatile uint32_t *)(TIM1_BASE + 0x18))
//#define TIM1_CCER     (*(volatile uint32_t *)(TIM1_BASE + 0x20))
//#define TIM1_BDTR     (*(volatile uint32_t *)(TIM1_BASE + 0x44))
//
//// --- PARAMÈTRES DU CORRECTEUR PI ---
//const float Kp = 1.0f;           // Gain proportionnel
//const float Wi = 36.95f;        // Paramètre d'intégration
//const float Ts = 0.001f;        // Période d'échantillonnage (1ms)
//float integrale = 0.0f;
//float consigne = 1.0f;          // 1V <=> 1A
//
//void setup(void) {
//    // 1. Activer horloges : GPIOA, ADC1, TIM1 et AFIO
//    RCC_APB2ENR |= (1 << 0) | (1 << 2) | (1 << 9) | (1 << 11);
//
//    // 2. Configurer PA8 en "Alternate Function Push-Pull" (PWM TIM1_CH1)
//    GPIOA_CRH &= ~(0xF << 0);
//    GPIOA_CRH |= (0xB << 0); // Speed 50MHz, AF Push-Pull
//
//    // 3. Configurer PA5 en mode Analogique (Entrée mesure courant)
//    GPIOA_CRL &= ~(0xF << 20);
//
//    // 4. Configurer TIM1 pour PWM à 22kHz
//    TIM1_PSC    = 0;              // Pas de diviseur
//    TIM1_ARR    = 3272;           // 72MHz / 22kHz ≈ 3272
//    TIM1_CCMR1 |= (0x6 << 4);  // Mode PWM 1 sur CH1
//    TIM1_CCER  |= (1 << 0);     // Activer sortie CH1
//    TIM1_BDTR  |= (1 << 15);    // Main Output Enable (MOE) - Spécifique TIM1
//    TIM1_CR1   |= (1 << 0);      // Lancer Timer
//
//    // 5. Configurer ADC1
//    ADC1_SQR3 = 5;             // Sélectionner canal 5 (PA5)
//    ADC1_CR2 |= (1 << 0);      // Allumer ADC
//}
//
//uint32_t read_ADC(void) {
//    ADC1_CR2 |= (1 << 22);     // Lancer conversion
//    while (!(ADC1_SR & (1 << 1))); // Attendre fin
//    return ADC1_DR;
//}
//
//void delay_ms(uint32_t ms) {
//    for (uint32_t i = 0; i < ms * 8000; i++) __asm__("nop");
//}
//
//int main(void) {
//    setup();
//
//    while (1) {
//        // --- 1. ACQUISITION ---
//        uint32_t val_adc = read_ADC();
//        // Conversion ADC (12 bits) en Volts : val * 3.3 / 4095
//        float courant_mesure = (float)val_adc * (3.3f / 4095.0f);
//
//        // --- 2. CALCUL DU PI ---
//        float erreur = consigne - courant_mesure;
//        integrale += erreur * Ts;
//
//        // Commande = Kp * (erreur + Wi * integrale)
//        float commande = Kp * (erreur + Wi * integrale);
//
//        // --- 3. SATURATION ET PROTECTION ---
//        // On bride la commande entre 0 et 1V pour rester en zone linéaire
//        if (commande > 1.0f) commande = 1.0f;
//        if (commande < 0.0f) commande = 0.0f;
//
//        // --- 4. ACTION (PWM) ---
//        // Le rapport cyclique est proportionnel à la commande (0V=0, 1V=100%)
//        // On ramène la commande 0-1V vers l'ARR du Timer (3272)
//        TIM1_CCR1 = (uint32_t)(commande * 3272.0f);
//
//        delay_ms(1); // Échantillonnage à 1kHz
//    }
//}


#include "main.h"

TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

int main(void)
{
    int32_t CH1_DC = 0;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    while (1)
    {
    	while(CH1_DC < 65535)
    	{
    	    TIM2->CCR1 = CH1_DC;
    	    CH1_DC += 70;
    	    HAL_Delay(1);
    	}
    	while(CH1_DC > 0)
    	{
    	    TIM2->CCR1 = CH1_DC;
    	    CH1_DC -= 70;
    	    HAL_Delay(1);
    	}
    }
}