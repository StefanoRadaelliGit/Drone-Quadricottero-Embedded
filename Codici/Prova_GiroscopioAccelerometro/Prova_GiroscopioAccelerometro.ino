/*##################################################################
 * Codice utilizzato per fare delle prove con la scheda MPU6050.
 * MPU6050 formisce gli output del Giroscopio e dell'Accelerometro.
 * MPU6050 comunica con il NodeMCU tramite protocollo I2C.
 * valore Giroscopio: gradi al secondo.
 * valore Accelerometro: g che sta agendo sull'asse in questione
 * Utilizzo un Loop di 250Hz come molte schede per droni
 ###################################################################*/


#include <Wire.h> //Libreria per la comunicazione I2C

int16_t gyroX, gyroY, gyroZ, accX, accY, accZ;
long gyroCalX, gyroCalY, gyroCalZ, vettoreTotale;
int temp;
long loop_timer;
float angle_pitch, angle_roll;
boolean settaGyro=false;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;


void setup() {
  Wire.begin();                                                      
  Serial.begin(2000000);                                               

  //setto le scale dei valori del gyro e dell'accelerometro
  setupMPU6050(); 

  //IMPORTANTE: Calibrazione iniziale del giroscopio, eseguita inizialmente mentre il sensore è fermo
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){ 
    letturaDatiMPU6050();
    gyroCalX += gyroX;   
    gyroCalY += gyroY;   
    gyroCalZ += gyroZ;   
    delay(3);                                                          //Delay 3us per simulare loop di 250Hz
  }
  gyroCalX /= 2000;
  gyroCalY /= 2000;
  gyroCalZ /= 2000;

  //Faccio partire il timer
  loop_timer = micros();
}

void loop(){

  letturaDatiMPU6050(); 

  //Sottraggo ai valore letti del gyro quelli letti durante la calibrazione, che rappresentano l'errore
  gyroX -= gyroCalX;
  gyroY -= gyroCalY;
  gyroZ -= gyroCalZ;
  
  //Calcolo gli angoli dal giroscopio: Ad ogni loop sommo ai gradi precedenti, quelli letti durante i 250Hz di loop
  //Se, ad esempio, ho un valore del giroscpio di 3°/s, e ho un loop di 1 secondo, devo aggiungere ai gradi raccolti precedentemente 3°
  //Al valore letto prima divido 65.5 come da documentazione dell'MPU6050 per la scala +-500. poi divido per i 250Hz del loop
  angle_pitch += gyroX/(250*65.5);
  angle_roll += gyroY/(250*65.5);
  
  //Calcolo gli angoli dall'accelerometro
  vettoreTotale = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));
  //57.296 = 1 / (3.142 / 180). Moltiplico per questo valore per avere il valore in gradi e non in radianti
  angle_pitch_acc = asin((float)accY/vettoreTotale)* 57.296;
  angle_roll_acc = asin((float)accX/vettoreTotale)* -57.296;
  
  //Tolgo i gradi di errore dell'accelerometro, da fare a mano guardando che valori legge l'accelerometro a livello
  angle_pitch_acc -= -0.3;
  angle_roll_acc -= -2.8;

  if(settaGyro){
    //Non mi posso affidare all'accelerometro, perchè con le vibazioni dei motori è troppo instabile
    //Il giroscopio è meno instabile ma ha il problema di "drift", e lo correggo con i valori dell'accelerometro
    //In questo modo la lettura dell'inclinazione sarà poco reattiva e lenta, ma precisa
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  }
  else{
    //Viene fatto solo la prima volta, per settare al giroscopio i valori letti dall'accelerometro, in quanto il drone potrebbe essere posizionato un pò storto
    angle_pitch = angle_pitch_acc; 
    angle_roll = angle_roll_acc;
    settaGyro = true;
  }
  
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
  
  Serial.print("PITCH ");
  Serial.print(angle_pitch_output);
  Serial.print("  ROLL ");
  Serial.println(angle_roll_output);

  //Aspetto per i 250hz. 250Hz=250 loop al secondo. 1/250 = 0.004 secondi a loop, ovvero 4000 microsecondi
  while(micros() - loop_timer < 4000);

  //resetto il timer
  loop_timer = micros();
}


void letturaDatiMPU6050(){ 
  Wire.beginTransmission(0x68);
  //I dati partono dal registro 0x3B
  Wire.write(0x3B);
  Wire.endTransmission();
  //I dati vengono memorizzati in 14 byte 
  Wire.requestFrom(0x68,14); 
  while(Wire.available() < 14);
  //I registri sono da un byte ma ogni valore è grande 2 byte, quindi memorizzo nei float il byte più significativo e quello meno significativo
  accX = Wire.read()<<8|Wire.read();
  accY = Wire.read()<<8|Wire.read();
  accZ = Wire.read()<<8|Wire.read();
  
  //L'MPU6050 ha anche il sensore di temperatura, potrebbe tornarmi utile...
  temp = Wire.read()<<8|Wire.read();

  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();

}

void setupMPU6050(){
  //L'indirizzo standard dell'MPU6050 è 0x68
  Wire.beginTransmission(0x68); 
  //Indirizzo 0x6B utilizzato per svegliare l'MPU6050                                       
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                    
  int ret = Wire.endTransmission();                                              
  if(ret==0) {
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("KO, MPU6050 non comunicante");
  }
  //Configuro l'accelerometro nella scala di valori +- 8g
  Wire.beginTransmission(0x68); 
  //0x1C indirizzo per settings dell'accelerometro, consultare documentazione MPU6050                                       
  Wire.write(0x1C);                                                    
  Wire.write(0x10);
  Wire.endTransmission(); 
  //Configure il giroscopio nella scala di valori +- 500°/s
  Wire.beginTransmission(0x68);
  //0x1B indirizzo per settings del giroscopio, consultare documentazione MPU6050   
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}














