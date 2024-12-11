#include <Arduino.h>
#define AR 2
#define AY 3
#define AG 4

#define BR 5
#define BY 6
#define BG 7

#define CR 8
#define CY 9
#define CG 10

int allLedPins[9] = {AR, AY, AG, BR, BY, BG, CR, CY, CG};

void initLed() {
  for (int i = 0; i < 9; i++) {
    pinMode(allLedPins[i], OUTPUT);
    digitalWrite(allLedPins[i], LOW);
  }
}


bool toggleLed(char side) {
  switch(side){
    
  }

}