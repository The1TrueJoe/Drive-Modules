// --------- Lib
#include <Arduino.h>

// --------- Pin Definitions

// Indentification Light
#define ID_LIGHT_RED A0
#define ID_LIGHT_GREEN A1
#define ID_LIGHT_BLUE A2

private:

public:
    void setupIDLight();
    void setIDLightColor(int red, int green, int blue);
    void errorLight();
    void offLight();