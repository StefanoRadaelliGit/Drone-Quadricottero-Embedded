/*##################################################################
 * Seconda versione, utilizza GPS e Bussola.
 * Il drone si stabilizza usando l'MPU6050 e l'algoritmo dei PID.
 * Come input riceve un segnale ai motori tramite OSC.
 * Utilizza GPS e bussola per stare fermo nelle coordinate del decollo.
 * NON funzionante. Ancora in fase di testing dei nuovi componenti.
 ###################################################################*/



//Roll: negativo, 3 e 4, Pitch: negativo, 2,4

#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
//#include <WiFiClient.h>
#include <stdio.h>
#include <string.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <WiFiUdp.h>
#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

boolean gps = true;

int val = 1000;

//const char* ssid     = "HUAWEI P8 lite 2017";
//const char* password = "ciaonoah123";
//const char* ssid = "Vodafone-33789299";
//const char* pass = "a24lifltjf4pdva";
const char* ssid     = "Honor 10";
const char* password = "Noah1996";
WiFiClient client;
WiFiUDP udp;
byte stringa [300];

Servo dummy0;
Servo dummy1;
Servo dummy2;
Servo dummy3;
Servo esc; //1
Servo esc1; //2
Servo esc2; //4
Servo esc3; //3

float somma1 = 0;
float somma2 = 0;
float somma3 = 0;
float somma4 = 0;
boolean b=true;

int c = 1000;

int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal, acc_total_vector;
int temperature;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float errore_roll, errore_pitch, errore_yaw, errore_lat, errore_lon;
float pid_p_roll, pid_i_roll, pid_d_roll, pid_p_pitch, pid_i_pitch, pid_d_pitch, pid_p_yaw, pid_i_yaw, pid_d_yaw, pid_p_lat, pid_i_lat, pid_d_lat, pid_p_lon, pid_i_lon, pid_d_lon;
float errorePrec_roll = 0;
float errorePrec_pitch = 0;
float errorePrec_yaw = 0;
float errorePrec_lat = 0;
float errorePrec_lon = 0;
float pid_roll, pid_pitch, pid_yaw, pid_lat, pid_lon;
float angolo = 0;
float angolo_roll = 0;
float compass_x_horizontal, compass_y_horizontal;

float kp_roll = 1.8;//2;
float ki_roll = 0.04; //0.01;
float kd_roll = 0.45;//1; //0.5
float kp_pitch = 1.5;//1;
float ki_pitch = 0.04;//0.05; 
float kd_pitch = 0.4;//0.5;
float kp_yaw = 2;
float ki_yaw = 0.01;
float kd_yaw = 1;
boolean fineRiga = false;
char buf[100];
char ora[] = {0,0,0,0,0,0,0,0};
int16_t compass_x, compass_y, compass_z;
double Heading = 0.00;
int lat, lon;
int latIniz, lonIniz;
boolean badGPSsignal = true;

void setup() {
  Serial.begin(500000);
  Serial2.begin(9600);
  Wire.begin(0,22, 400000);
  setup_display();
  u8x8.drawString(0,0,"MI CONNETTO...");
  connetti();
  setup_mpu_6050_registers();                                          

  u8x8.drawString(0,4,"CALIBRO IL GYRO:");
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  
    read_mpu_6050_data();                                              
    gyro_x_cal += gyro_x;                                              
    gyro_y_cal += gyro_y;                                              
    gyro_z_cal += gyro_z;                                              
    delay(3);                                                          
  }
  u8x8.drawString(0,5,"GYRO CALIBRATO.");
  gyro_x_cal /= 2000;                                                  
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;    

  setup_Compass();   

    setupGPS();

    Serial.println("Cerco segnale GPS");
    while(badGPSsignal) {
      readLatLong();
      delay(2);
    }
    Serial.println("Segnale GPS trovato");
    readLatLong();
    lonIniz = lon;
    latIniz = lat;
    lat = 0;
    lon = 0;

  esc.attach(25);
  dummy0.attach(5);
  esc1.attach(26);
  dummy1.attach(18);
  esc2.attach(27);
  dummy2.attach(19);
  esc3.attach(14);
  //esc.attach(5); //1 ---->5---->D5
  //esc1.attach(18); //2 ----->18----->D6
  //esc2.attach(23); //4 ----->23----->D7
  //esc3.attach(19); //3 ----->19----->D8

  //Piccoli segnali per "armare" (svegliare) i motori
  while(c<=1006) {
    esc.writeMicroseconds(c);
    esc1.writeMicroseconds(c);
    esc2.writeMicroseconds(c);
    esc3.writeMicroseconds(c);
    delay(1000);
    Serial.println(c);
    c++; 
  } 
  // 
  
  loop_timer = micros();                                               
}

void loop(){
  
  readLatLong();
  
  if(gps && abs(lat - latIniz)<700) {
    //GPS latitude PID
    errore_lat = (lat - latIniz)/100.0;
    pid_p_lat = 3*errore_lat;
    //pid_i_lat += 0.1*errore_lat;
    pid_d_lat = ((errore_lat - errorePrec_lat)/0.004)*2;
    //pid = pid_d;
    pid_lat = pid_p_lat + pid_d_lat;
    errorePrec_lat = errore_lat;

    if(pid_lat>=100) {
      pid_lat=100;
    }
    if(pid_lat<=-100) {
      pid_lat=-100;
    }


    //GPS longitude PID
    errore_lon = (lon - lonIniz)/100.0;
    pid_p_lon = 3*errore_lon;
    //pid_i_lat += 0.1*errore_lat;
    pid_d_lon = ((errore_lon - errorePrec_lon)/0.004)*2;
    //pid = pid_d;
    pid_lon = pid_p_lon + pid_d_lon;
    errorePrec_lon = errore_lon;

    if(pid_lon>=100) {
      pid_lon=100;
    }
    if(pid_lon<=-100) {
      pid_lon=-100;
    }
  
  } else {
    latIniz = lat;
    lonIniz = lon;
  }

  read_mpu_6050_data();                                                
  
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
  
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   
  angle_roll += gyro_y * 0.0000611;
  angle_yaw += gyro_z * 0.0000611;   
  //angle_yaw = gyro_z/65.5;                                 
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  angle_pitch_acc -= -1.4;                                              
  angle_roll_acc -= -3.3;                                              

  if(set_gyro_angles){                                                 
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004; 
    read_Compass(); 
    if (angle_yaw < 0) angle_yaw += 360;
    else if (angle_yaw >= 360) angle_yaw -= 360;
    angle_yaw -= course_deviation(angle_yaw, Heading) / 1200.0;
    if (angle_yaw < 0) angle_yaw += 360;
    else if (angle_yaw >= 360) angle_yaw -= 360;
  }
  else{                                                                
    angle_pitch = angle_pitch_acc;                                     
    angle_roll = angle_roll_acc;                                       
    set_gyro_angles = true;
    read_Compass();
    angle_yaw = Heading;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;     

  leggi();

  if(val>1800 || val<1000) {
    val=1000;
  }
//ROLL PID
  errore_roll = angle_roll - angolo_roll;
  pid_p_roll = kp_roll*errore_roll;
  pid_i_roll += ki_roll*errore_roll;
  pid_d_roll = ((errore_roll - errorePrec_roll)/0.004)*kd_roll;
  //pid = pid_d;
  pid_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  errorePrec_roll = errore_roll;

  if(pid_roll>=100) {
    pid_roll=100;
  }
  if(pid_roll<=-100) {
    pid_roll=-100;
  }
////////////////////////////////////////
//PITCH PID
  errore_pitch = angle_pitch - angolo;
  pid_p_pitch = kp_pitch*errore_pitch;
  pid_i_pitch += ki_pitch*errore_pitch; 
  pid_d_pitch = ((errore_pitch - errorePrec_pitch)/0.004)*kd_pitch;
  //pid = pid_d;
  pid_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  errorePrec_pitch = errore_pitch;

  if(pid_pitch>=100) {
    pid_pitch=100;
  }
  if(pid_pitch<=-100) {
    pid_pitch=-100;
  }
///////////////////////////////////////////////////
//YAW pid
  if(angle_yaw < 180) {
    errore_yaw = angle_yaw*-1;
  } else {
    errore_yaw = (angle_yaw - 360)*-1;
  }
  //angle_yaw = gyro_z*-1/65.5;  //DA COMMENTARE
  //errore_yaw = (angle_yaw) - 0; //DA COMMENTARE
  pid_p_yaw = kp_yaw*errore_yaw;
  pid_i_yaw += ki_yaw*errore_yaw;
  pid_d_yaw = ((errore_yaw - errorePrec_yaw)/0.004)*kd_yaw;
  //pid = pid_d;
  pid_yaw = pid_p_yaw + pid_i_yaw + pid_d_yaw;
  errorePrec_yaw = errore_yaw;

  if(pid_yaw>=100) {
    pid_yaw=100;
  }
  if(pid_yaw<=-100) {
    pid_yaw=-100;
  }
///////////////////////////////////////////////////

  if(!gps) {
    pid_lat = 0;
    pid_lon = 0;
  }
  
  somma1=pid_roll + pid_pitch - pid_yaw - pid_lat + pid_lon;
  somma2=pid_roll - pid_pitch + pid_yaw + pid_lat + pid_lon;
  somma3=-pid_roll + pid_pitch + pid_yaw - pid_lat - pid_lon;
  somma4=-pid_roll - pid_pitch - pid_yaw + pid_lat - pid_lon;

  if(val<1100) {
    somma1=0;
    somma2=0;
    somma3=0;
    somma4=0;
  }
  if(val<1200) {
    pid_i_roll=0;
    pid_i_pitch=0;
    pid_i_yaw=0;
  }

  esc.writeMicroseconds(val+somma1); 
  esc1.writeMicroseconds(val+somma2); 
  esc2.writeMicroseconds(val+somma4); 
  esc3.writeMicroseconds(val+somma3); 

  Serial.print(" PITCH ");
  Serial.print(angle_pitch);
  Serial.print("  ROLL ");
  Serial.print(angle_roll);
  Serial.print(" YAW ");
  Serial.print(angle_yaw);
  Serial.print(" m1 ");
  Serial.print(somma1);
  Serial.print(" m2 ");
  Serial.print(somma2);
  Serial.print(" m3 ");
  Serial.print(somma3);
  Serial.print(" m4 ");
  Serial.print(somma4);
//  Serial.print(" ANG_ROLL ");
//  Serial.print(angolo_roll);
//  Serial.print(" ANG_PITCH ");
//  Serial.print(angolo);
Serial.print(" LAT ");
  Serial.print(pid_lat);
  Serial.print(" LONG ");
  Serial.print(pid_lon);
  Serial.print(" ");
  Serial.print(lat-latIniz);
  Serial.print(" ");
  Serial.print(lon-lonIniz);
  Serial.print(" ");
  Serial.print(Heading);
  Serial.print(" ");
  Serial.print(micros() - loop_timer);
  
  while(micros() - loop_timer < 4000);  
  Serial.print(" ");
  Serial.println(val);                              
  loop_timer = micros();                                               
}


void read_mpu_6050_data(){                                             
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x3B);                                                    
  Wire.endTransmission();                                              
  Wire.requestFrom(0x68,14);                                           
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temperature = Wire.read()<<8|Wire.read();                            
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read(); 
  gyro_z*=-1;
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                    
  int boh = Wire.endTransmission();                                    
  if(boh==0) {
    Serial.println("OK");
    u8x8.drawString(0,3,"MPU6050 OK");
  } else {
    Serial.println("NO");
    u8x8.drawString(0,3,"MPU6050 FAILED");
  }
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                    
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission(); 
  //Filtro passa basso 43Hz
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1A);                                                    
  Wire.write(0x03);                                                    
  Wire.endTransmission();                                            
}

void connetti() {
  delay(10);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    int punto = 0;

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if(punto==0) {
          u8x8.drawString(0,0,"MI CONNETTO....");
          punto=1;
        } else {
          u8x8.drawString(0,0,"MI CONNETTO... ");
          punto=0;
        }
    }

    Serial.println("");
    Serial.print("WiFi connesso a: ");
    Serial.println(ssid);
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    u8x8.drawString(0,1,"CONNESSO A WIFI");
    u8x8.drawString(0,2,WiFi.localIP().toString().c_str());
  udp.begin(8000);
}

void leggi() {
  OSCMessage message;
  int size;
  if(size = udp.parsePacket() > 0) {
    udp.read(stringa, 300);
    Serial.printf("%s\n", stringa);
    message.fill(stringa, 300);
    if(!message.hasError()) {
      message.route("/drone/slider1", su);
    }
    if(!message.hasError()) {
      message.dispatch("/drone/button4", sx);
    }
    if(!message.hasError()) {
      message.dispatch("/drone/button5", dx);
    }
    if(!message.hasError()) {
      message.dispatch("/drone/button2", stopp);
    }
    if(!message.hasError()) {
      message.dispatch("/drone/button1", av);
    }
    if(!message.hasError()) {
      message.dispatch("/drone/button3", in);
    }
  }
}

void su(OSCMessage &msg, int addrOffset) {
  val = msg.getFloat(0);
//  char boh[3];
//  msg.getString(0, boh, 4);
//  Serial.println(boh);
//  int valX = atoi(boh);
//  Serial.println(valX);
}

void dx(OSCMessage &msg) {
  angolo_roll=5;
}

void sx(OSCMessage &msg) {
  angolo_roll=-5;
}

void stopp(OSCMessage &msg) {
  //angolo=0;
  //angolo_roll=0;
  kd_roll=1;
}

void av(OSCMessage &msg) {
    //angolo = -5;
    kd_roll+=0.1;
}

void in(OSCMessage &msg) {
    //angolo = 5;
    kd_roll-=0.1;
}

void setupGPS() {

  uint8_t saveConf[21] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};

  //Disabilita GPGSV.
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial2.write(Disable_GPGSV, 11);

  uint8_t disable_GNGLL[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};

  Serial2.write(disable_GNGLL, 11);

  uint8_t disable_GNRMC[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};

  Serial2.write(disable_GNRMC, 11);

  uint8_t disable_GNVTG[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};

  Serial2.write(disable_GNVTG, 11);

  uint8_t disable_GNGSA[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};

  Serial2.write(disable_GNGSA, 11);
  
  uint8_t Set_to_10Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
  Serial2.write(Set_to_10Hz, 14);
  
 // Baud Rate a 57k.
  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                              };

uint8_t Set_to_115k[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                           0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
                          };
  Serial2.write(Set_to_115k, 28);
  delay(200);
  //loopWrite();

  Serial2.begin(115200);
  delay(200);

}

void readOrario() {
  while(Serial2.available()) {
  int pos = 0;
  uint8_t lettura = Serial2.read();
  if(lettura == '$') {
    while(Serial2.available() && fineRiga == false) {
      lettura = Serial2.read();
      buf[pos] = lettura;
      pos++;
      if(lettura == '*') fineRiga=true;
    }
    if(buf[3] == 'G' && buf[4] == 'A' && buf[6] != ',') {
//      orario1 = (buf[6] - '0')*10 + (buf[7] - '0');
//      orario2 = (buf[8] - '0')*10 + (buf[9] - '0');
//      orario3 = (buf[10] - '0')*10 + (buf[11] - '0');
//      orario4 = (buf[13] - '0')*10 + (buf[14] - '0');
//      ora = "";
//      ora = ora + buf[6] + buf[7] + buf[8] + buf[9] + buf[10] + buf[11] + buf[13] + buf[14];
      ora[0] = buf[6];
      ora[1] = buf[7];
      ora[2] = buf[8];
      ora[3] = buf[9];
      ora[4] = buf[10];
      ora[5] = buf[11];
      ora[6] = buf[13];
      ora[7] = buf[14];
    } 
  }
  fineRiga = false;
  }
}

void setup_Compass() {
  
  Wire.beginTransmission(0x0D); //start talking
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(0x0D); //start talking
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x09); // Set the Register
  int ret = Wire.endTransmission();                                    
  if(ret==0) {
    Serial.println("COMPASS OK");
    u8x8.drawString(0,6,"QMC5883L OK.");
  } else {
    Serial.println("COMPASS KO");
    u8x8.drawString(0,6,"QMC5883L FAILED.");
  }

}

void read_Compass() {
  Wire.beginTransmission(0x0D);
  Wire.write(0x00); //start with register 3.
  Wire.endTransmission();

  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(0x0D, 6);
  if (6 <= Wire.available()) {
    compass_x = Wire.read(); //LSB  x
    compass_x |= Wire.read() << 8; //MSB  x
    compass_y = Wire.read(); //LSB  z
    compass_y |= Wire.read() << 8; //MSB z
    compass_z = Wire.read(); //LSB y
    compass_z |= Wire.read() << 8; //MSB y
  }

  compass_x_horizontal = (float)compass_x * cos(angle_pitch * 0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * 0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * 0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
  //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
  if (compass_y_horizontal < 0) {
    Heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  } else {
    Heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);
  }

  Heading += 2.37;      //Declinazione magnetica italiana
  Heading+=70;
  if (Heading < 0) {
    Heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  } else if (Heading >= 360) {
    Heading -= 360;         //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
  }
}

void setup_display() {
  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();
  u8x8.setFlipMode(0);
  u8x8.setInverseFont(0);
}

//The following subrouting calculates the smallest difference between two heading values.
float course_deviation(float course_b, float course_c) {
  float base_course_mirrored, actual_course_mirrored;
  float course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}

void readLatLong() {
  while(Serial2.available()) {
  int pos = 0;
  uint8_t lettura = Serial2.read();
  if(lettura == '$') {
    while(Serial2.available() && fineRiga == false) {
      lettura = Serial2.read();
      buf[pos] = lettura;
      pos++;
      if(lettura == '*') fineRiga=true;
    }
    if(buf[3] == 'G' && buf[4] == 'A' && buf[6] != ',' && buf[16] != ',' && (buf[43] == '1' || buf[43] == '2')) {
      lat = 0;
      lon = 0;
      //Leggo Latitudine
      String latString;
      for(int i = 16; i<26; i++) {
        if(buf[i]=='.') {
          i++;
        } else {
          latString += buf[i];
        }
      }
      lat = latString.toInt();

      //Leggo longitudine
      String lonString;
      for(int i = 29; i<40; i++) {
        if(buf[i]=='.') {
          i++;
        } else {
          lonString += buf[i];
        }
      }
      lon = lonString.toInt();
      
      badGPSsignal = false;
    } else {
      badGPSsignal = true;
    }
  }
  fineRiga = false;
  }
}









