/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13.
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead().

 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground

 * Note: because most Arduinos have a built-in LED attached
 to pin 13 on the board, the LED is optional.


 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/AnalogInput

 */

int inputPinPot = A0;    // select the input pin for girouette angle
int outputPinPot = 9;    // select the output pin for girouette angle
int angle;
int angleServo;
int angle_min = 0;
int angle_max = 360;


//int inputPinOpto = A1;    // select the input pin for girouette angle
int outputPinOpto = 10;    // select the output pin for girouette angle

int nb_teeth = 1;
long duree_test = 1000; // test sur 1 secondes
int vitesse_min = 0;
int vitesse_max = 90; //(tr/s)
long vitesse = 0;
int vitesseServo;
long chrono = 0; // valeur courante du chrono
long chrono_depart = 0; // valeur de départ du chrono

volatile long nb_chgt = 0; // nb de changement etat Pin
  
// Gestion de l'interruption 0
  
void gere_int0() { 
    nb_chgt = nb_chgt + 1 ;  
    Serial.print("interrupt\n");
}

int scale_output(int input, int input_min, int input_max){

  int output_min = 2233;
  int output_max = 3823;

  if (input > input_max) return output_max;
  if (input < input_min) return output_min;
  int output = (1. / (input_max - input_min)) * (input - input_min) * (output_max - output_min) + output_min;

  return output;
}


void setup() {

  // declare pin modes
  pinMode(inputPinPot, INPUT);
  pinMode(outputPinPot, OUTPUT);
  pinMode(outputPinOpto, OUTPUT);

  chrono_depart = millis();
  
  // l'interruption zero correspond à la pin 2 
  // on enregistre tous les changements d'état

  pinMode (2, INPUT);
  attachInterrupt(0,gere_int0,RISING); 

  Serial.begin(9600); 
}

void loop() {

  //read voltage from potentiometer mounted on the girouette
  //sensorValue ranges from 0 to 360 (deg)
  angle = analogRead(inputPinPot)*360./1023;

  //scale sensorValue on 2000;4000
  angleServo = scale_output(angle, angle_min, angle_max);
  
  // copy value to PWM output ([0 ; 255])
  //scale sensorValue by 255./5000
  analogWrite(outputPinPot, angleServo*255./5000);

  //count number of rising pulse per seconds
  chrono = millis();
  
  // est-ce que le test est fini ?
    
  if (chrono - chrono_depart > duree_test) {  

     //determine vitesse de rotation in tr/s
     vitesse=nb_chgt/nb_teeth;
  
     //scale vitesse on [2000 ; 4000]
     vitesseServo = scale_output(vitesse, vitesse_min, vitesse_max);
  
     Serial.print("girouette angle\n");
     Serial.print(angle);
     Serial.print('\n');  
     Serial.print(angleServo);
     Serial.print('\n');
  
     Serial.print("vitesse rotation (tr/s)\n");
     Serial.print(vitesse);
     Serial.print('\n');  
     Serial.print(vitesseServo);
     Serial.print('\n');  
     
     // copy value to PWM output
     //scale vitesse by 255./5000
     analogWrite(outputPinOpto, vitesseServo*255./5000);
       
     chrono_depart = millis(); 
     nb_chgt=0;
  }     

}
