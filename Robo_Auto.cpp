#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>

// Bluetooth
SoftwareSerial bluetooth(A3, A2); //TX, RX (Bluetooth)
// utilizei bluetooth para mostrar os valores de 
// deslocamento do acelerometro e giroscópio.
// variaveis para contagem do tempo (ms)
unsigned long millis_I = 0;
unsigned long millis_F = 0;
unsigned long millis_R = 0;

// Motor M1 e M3 Left Side
  const uint8_t pwmLeft = 3;   // ENA1 - Enable and PWM
  const uint8_t fwdLeft = 2;   // IN1 - Forward Drive 
  const uint8_t rwdLeft = 4;   // IN2 - Reverse Drive 
  //-----------------
  // Motor M2 e M4 Right Side
  const uint8_t pwmRight = 5;   // ENB - Enable and PWM
  const uint8_t fwdRight = 7;   // IN3 - Forward Drive 
  const uint8_t rwdRight = 6;  //  IN4 - Reverse Drive

//Configurando HCSR04
  Ultrasonic HCF(13,10); //Dist Frente
  Ultrasonic HCR(13,11); //Dist Direita
  Ultrasonic HCL(13,9); //Dist Esquerda
  //armazenar distâncias.
  int distF = 0, distR = 0, distL = 0;


//Endereço I²C do MPU 6050
  const int MPU = 0x68;
  //SCL     ->     A5
  //SDA     ->     A4

//Variaveis para armazenar valores do acelerometro e giroscópio
  float eX, eY, eZ, temp, giX, giY, giZ;
  float eXT, eYT, giZT;
  float z = 0, x = 0, y = 0;
  float acY = 0, acYT = 0, acYTcm = 0, acX = 0, acXT = 0, acXTcm = 0; 
  float giroZ = 0;
  float giroZT = 0;
  int ctt = 0, cttA = 0, speed = 255;
  int girar = 90, descX = 0, descY = 0;

// variáveis para identificar sentido de deslocamento, esquerda ou direita.
  int deskR = 0, deskL = 0;
  float fatorR = 1, fatorL = 1;
  int a = 1, est = 0;
//funções do MPU6050
  void Solic_MPU6050();
// funções de deslocamento
  void Contagem_AcDescX();
  void Regulagem_AcDescY();
  void Contagem_giro();
// funções para movimentar veículo
  void Em_frenteDesc();
  void Em_frente();
  void Back();
  void All_stop();
  void Giro_Esquerda();
  void Giro_Direita();

void setup(){
  // define banda de comunicação serial com computador
  Serial.begin(9600);
  bluetooth.begin(9600);
  // configura a escala de variação do giroscópio em graus/segundos
  Wire.begin();
  //Inicializa o MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0); 
  Wire.endTransmission(true);
  delay(5); 
  // configurando pinos do motor como OUT
  pinMode(pwmRight, OUTPUT);
  pinMode(fwdRight, OUTPUT);
  pinMode(rwdRight, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(fwdLeft, OUTPUT);
  pinMode(rwdLeft, OUTPUT);
}

void loop(){
  
  if(est == 0) {
    delay(10000);
    est++;
  }
  
  if(est%2 == 0){
    while(HCF.read(CM) > 25) Em_frente();
    All_stop();
    delay(500);
    Giro_Direita();
    while((HCF.read(CM) > 25) && acXT < 0.50) Em_frenteDesc();
    All_stop();
    if(acXT > 0) acXT = 0;
    est++;
  }
  else if(est%2 == 1){
    while(HCF.read(CM) > 25) Em_frente();
    All_stop();
    delay(500);
    Giro_Esquerda();
    while((HCF.read(CM) > 25) && (acXT < 50)) Em_frenteDesc();
    if(acXT > 0) acXT = 0;
    est++;
  }
}

void Solic_MPU6050(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,14,true);
  //Armazena o valor dos sensores nas variaveis correspondentes
  eX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  eY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  eZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  giX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  giY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  giZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  if(giZ < 0 ) giZ = giZ * -1; // converte para + em caso de -
  giZT = giZ/131; // converte o dado puro do MPU6050 para um range de 0 a 250º/s
  acXT = acXT/1638.4; // converte dado puro, para velocidade em 
  acYT = acYT/1638.4;

}

void Regulagem_AcDescY(){
   millis_I = millis();
  while(millis_R <= 25){
    ctt++;
    Solic_MPU6050();
    y =+ eYT;
    millis_F = millis();
    millis_R = millis_F - millis_I;
  }
  millis_R = 0;
  y = y/ctt;
  y = y * 9.8;
  y = y/10;
  ctt = 0;
  acY = y * 0.025;
  acYT = acYT + acY; // deslocamento eixo Y
  //+ positivo, carrinho se deslocando
  //indevidamente para direita, frear a roda esquerda
  if((acYT < 0.10) && (acYT < -0.10)){
    fatorL = 1;
    fatorR = 1;
  }else if((acYT > 0.10) && (acYT < 0.20)){ 
    fatorL = 0.8;
    fatorR = 1;
  }else if((acYT < -0.10) && (acYT > -0.20)) {
    fatorR = 0.8;
    fatorL = 1;
  }else if(acYT > 0.20){
    fatorL = 0.6;
    fatorR = 1;
  }else if(acYT < -0.20){
    fatorR = 0.6;
    fatorL = 1;
  }
}

void Contagem_AcDescX(){
  millis_I = millis();
  while(millis_R <= 25){
    ctt++;
    Solic_MPU6050();
    x =+ eXT;
    millis_F = millis();
    millis_R = millis_F - millis_I;
  }
  millis_R = 0;
  x = x/ctt;
  x = x * 9.8;
  x = x/10;
  ctt = 0;
  acX = x * 0.025;
  acXT = acXT + acX; // vetor final de deslocamento
}

void Contagem_giro(){
  while(giroZT <= 90){
    millis_I = millis(); //pega o valor inicial em ms.
    while(millis_R <= 25){// enquanto não se passar 100ms ele pega valores do MPU6050
      ctt++; //contador para média dos graus por segundo em 100ms
      Solic_MPU6050(); //func para salvar os dados puros do MPU6050.
      z = z + giZT; //faz somatório dos valores de º/s para tirar média depois
      millis_F = millis();//pega valor em ms atual
      millis_R = millis_F - millis_I; //vai atualizando Millis até estourar em 100ms 
    }// após passar os 100ms, vamos calcular o quanto já girou
    millis_R = 0;// zera millis para a proxima rotina
    z = z/ctt; // tiramos a média dos valores em º/s
    giroZ = z * 0.025; // multiplica o valor da média que é em º/s
                        // e multiplica por 0,1 que equivale a 0,1s ou 100ms
    giroZT = giroZT + giroZ; //a cada 100ms um valor já deslocado é adicionado ao giro.
    ctt = 0; //zera o contador para média
    millis_I = millis(); //adiciona o novo valor para a próxima contagem dos 100ms
  }
  All_stop(); // após girar 90º no eixo Z, desliga os motores setando (LOW)
  giroZT = 0;
}

void Em_frente(){
    // M1 && M3 Left
    // XO
    // XO
    analogWrite(pwmLeft, speed/fatorL);
    digitalWrite(fwdLeft, HIGH);
    digitalWrite(rwdLeft, LOW);
    // M2 && M4 Right 
    // OX
    // OX
    analogWrite(pwmRight, speed/fatorR);  
    digitalWrite(fwdRight, HIGH);
    digitalWrite(rwdRight, LOW);
    Regulagem_AcDescY();
}

void Em_frenteDesc(){
  // M1 && M3 Left
  // XO
  // XO
  analogWrite(pwmLeft, speed/1);
  digitalWrite(fwdLeft, HIGH);
  digitalWrite(rwdLeft, LOW);
  // M2 && M4 Right 
  // OX
  // OX
  analogWrite(pwmRight, speed/1);  
  digitalWrite(fwdRight, HIGH);
  digitalWrite(rwdRight, LOW);
  Contagem_AcDescX();
}

void Back(){
  // M1 && M3 Left
  // XO
  // XO
  analogWrite(pwmLeft, speed/1);
  digitalWrite(fwdLeft, LOW);
  digitalWrite(rwdLeft, HIGH);
  // M2 && M4 Right 
  // OX
  // OX
  analogWrite(pwmRight, speed/1);  
  digitalWrite(fwdRight, LOW);
  digitalWrite(rwdRight, HIGH);
}

void Giro_Esquerda(){
  // M1 && M3 Left
  // XO
  // XO
  analogWrite(pwmLeft, speed/1);
  digitalWrite(fwdLeft, LOW);
  digitalWrite(rwdLeft, HIGH);
  // M2 && M4 Right 
  // OX
  // OX
  analogWrite(pwmRight, speed/1);  
  digitalWrite(fwdRight, HIGH);
  digitalWrite(rwdRight, LOW);
  Contagem_giro();
}

void Giro_Direita(){
  // M1 && M3 Left
  // XO
  // XO
  analogWrite(pwmLeft, speed/1);
  digitalWrite(fwdLeft, HIGH);
  digitalWrite(rwdLeft, LOW);
  // M2 && M4 Right 
  // OX
  // OX
  analogWrite(pwmRight, speed/1);  
  digitalWrite(fwdRight, LOW);
  digitalWrite(rwdRight, HIGH);
  Contagem_giro();
}

void All_stop(){
  // M1 && M3 Left
  // XO
  // XO
  analogWrite(pwmLeft, LOW);
  digitalWrite(fwdLeft, LOW);
  digitalWrite(rwdLeft, LOW);
  // M2 && M4 Right 
  // OX
  // OX
  analogWrite(pwmRight, LOW);  
  digitalWrite(fwdRight, LOW);
  digitalWrite(rwdRight, LOW);
}