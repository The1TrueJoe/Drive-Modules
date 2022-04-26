/**
 * @file id_light.h
 * 
 * @author Joseph Telaak
 * 
 * @brief ID light on the modules
 * 
 * @version 0.1
 * 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// --------- Lib
#include <Arduino.h>

// --------- Pin Definitions

// Indentification Light
#define ID_LIGHT_RED A0
#define ID_LIGHT_GREEN A1
#define ID_LIGHT_BLUE A2

public:
    void setupIDLight();
    void setIDLightColor(int red, int green, int blue);
    void errorLight();
    void offLight();