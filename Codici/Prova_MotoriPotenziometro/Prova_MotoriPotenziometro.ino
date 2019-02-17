/*#####################################################################
 * Codice utilizzato per testare i motori con un potenziometro.
 * I motori vengono controllati tramite gli ESC.
 * Gli ESC ricevono valori tra 0 e 2000.
 * Nel mio caso i motori iniziano a muoversi a 1021.
 * Utilizzo quindi come valore minimo 1000.
 ######################################################################*/

//Esiste una libreria apposta per i motori BrushLess, ma preferisco usare i comandi dei Servo perchè sono più precisi
#include <Servo.h> 

Servo esc; 
Servo esc1;
Servo esc2;
Servo esc3;

int c = 1000;

void setup() {
  //Utilizzo questi pin per i 4 motori
  esc.attach(D5);
  esc1.attach(D6);
  esc2.attach(D7);
  esc3.attach(D8);
  
  Serial.begin(9600);

  //IMPORTANTE: mando agli esc dei valori molto piccoli, per "armarli" (svegliarli).
  //Saltando questo passaggio, gli esc non rispondono ai segnali
  while(c<=1006) {
    esc.writeMicroseconds(c);
    esc1.writeMicroseconds(c);
    esc2.writeMicroseconds(c);
    esc3.writeMicroseconds(c);
    delay(1000);
    Serial.println(c);
    c++; 
  } 

}

void loop() {

  //Leggo il valore del potenziometro
  int potenz= analogRead(A0);
  //Mappo tra 1000 e 2000
  potenz= map(potenz, 0, 1023,1000,2000); 

  //Nelle prove non voglio salire troppo con i giri dei motori
  if(potenz>1600) {
    potenz=1600;
  }

  Serial.println(potenz);

  esc.writeMicroseconds(potenz);
  esc1.writeMicroseconds(potenz);
  esc2.writeMicroseconds(potenz);
  esc3.writeMicroseconds(potenz);

}
