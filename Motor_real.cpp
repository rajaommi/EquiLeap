#include <Arduino.h>
#include <string.h>
#include "Motor_real.h"


Motor::Motor()
{
//  MOVE[0] = &Motor::Forward;
//  MOVE[1] = &Motor::Back;
//  MOVE[2] = &Motor::Left;
//  MOVE[3] = &Motor::Right;
}

void Motor::Stop()
{
  Serial2.print("!M "+ String(0)+" "+ String(0)+ "\r");
}

void Motor::Forward(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.println("!M "+ String(speed)+" "+ String(-speed));
}

void Motor::Back(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.print("!M "+ String(-speed)+" "+ String(-speed)+ "\r");
}


void Motor::Left(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.print("!M "+ String(speed)+" "+ String(0)+ "\r");
}

void Motor::Right(int speed)
{
  speed=constrain(speed, -300, 300);
  Serial2.print("!M "+ String(0)+" "+ String(speed)+ "\r");
}

unsigned long tps_cmd=millis();

void Motor::Control(int l_spd, int r_spd)
{
  if (Serial2.available() == 0) {
    Serial2.println("!M " + String(l_spd) + " " + String(r_spd));
    tps_cmd = millis();
    // Serial.print("send | ");
  }

  while (Serial2.available() > 0) {
    char c = Serial2.read(); // Lecture caractère par caractère
    if (c == '+') {
        // Serial.print(" rec = ");
        // Serial.println(millis() - tps_cmd);
        // break;
    }
  }

}
  /*
  // Envoyer une commande initiale si le buffer est vide
  if (Serial2.available() == 0) {
      Serial2.print("!M " + String(0) + " " + String(0) + "\r");

      tps_cmd = millis();
  }

  // Boucle pour lire le buffer du port série
  while (Serial2.available() > 0) {
      String incomingData = Serial2.readString(); // Lire les données entrantes sous forme de chaîne
      Serial.print("buf = ");
      Serial.println(incomingData); // Afficher sur le moniteur série

      // Vérifier si la réponse contient le caractère d'acquittement
      if (incomingData.indexOf('+') != -1) {

          Serial.print("tps_cmd = ");
          Serial.println(millis()-tps_cmd);
      }
  }
  */


  // Serial2.println("!M 70 70");  // Envoi de commande
  // delay(1000);  // Attente de la réponse

  // while (Serial2.available() > 0) {
  //   char c = Serial2.read();  // Lecture caractère par caractère
  //   Serial.println(c);  // Affichage sur le moniteur série
  // }

  // delay(2000); // Attendre avant la prochaine commande


