#include <SoftwareSerial.h>
#include <Servo.h> // Inclui a biblioteca do servo

SoftwareSerial mySerial(12, 13); // RX, TX

Servo servo; // define o nome do servo
Servo garra;

int valores[10] = {};
int total = 0;
int pos = 0;
int media;

String height = "";
char c;
int altura, val;

int convert (String height);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  Serial.println("Conectado");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  servo.attach(13);
  servo.write(145); // 67 - 145

  garra.attach(A0);
  garra.write(50); // 105 - 50

}

void loop() { // run over and over
  if (mySerial.available()) {
     c = mySerial.read();
     if (c == ')'){ // Abre
      garra.write(50);
     }
     else if(c == '('){ // Fecha
      garra.write(105);
      }
      else{
     
     if (c!='*') height += c;
     else{
      //height += '*';
      altura = convert(height);
      height = "";
      //Serial.print(altura);
      //Serial.print ("  -  ");
      val = map(altura, 60, 190, 50,145);

      total -= valores[pos];
      valores[pos++] = val;
      total += val;

      media = total/10;

      if (pos == 10) pos = 0;
      
      
      if (media >= 67 and media <= 145) servo.write(media);
      Serial.println(media);
     }
    }
  }
}

int convert (String height){

  int altura = 0;
  
  for (int i = 0; height[i] != '\0'; i++){
    altura = altura*10;
    altura += height[i] - '0';
  }

  return altura;
}

