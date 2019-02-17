
//Roll: negativo, 3 e 4, Pitch: negativo, 2,4

#include <Wire.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <stdio.h>
#include <string.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <WiFiUdp.h>

int val = 1000;

const char* ssid = "HUAWEI P8 lite 2017";
const char* pass = "ciaonoah123";
//const char* ssid = "Vodafone-33789299";
//const char* pass = "a24lifltjf4pdva";
WiFiClient client;
WiFiUDP udp;
byte stringa [300];

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
float errore_roll, errore_pitch, errore_yaw;
float pid_p_roll, pid_i_roll, pid_d_roll, pid_p_pitch, pid_i_pitch, pid_d_pitch, pid_p_yaw, pid_i_yaw, pid_d_yaw;
float errorePrec_roll = 0;
float errorePrec_pitch = 0;
float errorePrec_yaw = 0;
float pid_roll, pid_pitch, pid_yaw;
float angolo = 0;
float angolo_roll = 0;

float kp_roll = 2;
float ki_roll = 0.07; //0.01;
float kd_roll = 0.7; //0.5
float kp_pitch = 2;
float ki_pitch = 0.07; //0.01;
float kd_pitch = 0.5;
float kp_yaw = 1;
float ki_yaw = 0.01;
float kd_yaw = 0;


void setup() {
  Serial.begin(2000000);
  connetti();
  Wire.begin();
  
  setup_mpu_6050_registers();                                          
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  
    read_mpu_6050_data();                                              
    gyro_x_cal += gyro_x;                                              
    gyro_y_cal += gyro_y;                                              
    gyro_z_cal += gyro_z;                                              
    delay(3);                                                          
  }
  gyro_x_cal /= 2000;                                                  
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;                                                  

  esc.attach(D5); //1
  esc1.attach(D6); //2
  esc2.attach(D7); //4
  esc3.attach(D8); //3

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

  read_mpu_6050_data();                                                
  
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
  
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   
  angle_roll += gyro_y * 0.0000611;
//  angle_yaw += gyro_z * 0.0000611;   
  angle_yaw = gyro_z/65.5;                                 
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  angle_pitch_acc -= 0.5;                                              
  angle_roll_acc -= -3.1;                                              

  if(set_gyro_angles){                                                 
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{                                                                
    angle_pitch = angle_pitch_acc;                                     
    angle_roll = angle_roll_acc;                                       
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      

  //val= analogRead(A0);

  //val= map(val, 0, 1023,1000,2000);

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
//PITCH yaw
  errore_yaw = (angle_yaw) - 0;
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

  somma1=pid_roll + pid_pitch - pid_yaw;
  somma2=pid_roll - pid_pitch + pid_yaw;
  somma3=-pid_roll + pid_pitch + pid_yaw;
  somma4=-pid_roll - pid_pitch - pid_yaw;

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
  Serial.print(" ANG_ROLL ");
  Serial.print(angolo_roll);
  Serial.print(" ANG_PITCH ");
  Serial.print(angolo);
  
  while(micros() - loop_timer < 4000);  
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

}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                    
  int boh = Wire.endTransmission();                                    
  if(boh==0) {
    Serial.println("OK");
  } else {
    Serial.println("NO");
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
}

void connetti() {
  delay(1000);
  Serial.println();
  Serial.print("conecting to: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA); 
  delay(1000);
  WiFi.begin(ssid, pass);
  //IPAddress subnet(255,255,255,0);
  //WiFi.config(IPAddress(192,168,1,150),IPAddress(192,168,1,10),subnet);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.print("\nConnesso");

  Serial.println();
  Serial.print("My IP: ");
  Serial.println(WiFi.localIP());
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
  angolo=0;
  angolo_roll=0;
}

void av(OSCMessage &msg) {
    angolo = -5;
}

void in(OSCMessage &msg) {
    angolo = 5;
}














