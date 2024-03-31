#include "DeltaControl.h"

bool replaying = false;

void setup() {
  Serial.begin(9600);
  initMotors();
}

void cartesian_control(int coordinates[]) {
  // Function for Cartesian control
  pose pos = {coordinates[0], coordinates[1], coordinates[2], 0, 0};
  movetoPose(pos);
}

void phase_control(int degrees[]) {
  // Function for phase control
  setMotorDegrees(degrees);
}

void replay_control(int degrees[]) {
  // Function to replay memorized path
  if (!replaying){
    restartMotors();
    replaying = true;
  }
  replayMemorized(degrees);
}

void learn_mode() {
  deactivateMotors();
  String input = "";
  int degrees[3];
  int old_degrees[3];
  while (true) {
    if (Serial.available() > 0) {
      restartMotors();
      break;
    }
    
    motorDegrees(degrees);

    if (degrees[0] != old_degrees[0] || degrees[1] != old_degrees[1] || degrees[2] != old_degrees[2]){
      // Sending "replay" followed by three numbers
      Serial.print("replay ");
      Serial.print(degrees[0]);
      Serial.print(" ");
      Serial.print(degrees[1]);
      Serial.print(" ");
      Serial.println(degrees[2]);
    }

    old_degrees[0] = degrees[0];
    old_degrees[1] = degrees[1];
    old_degrees[2] = degrees[2];
  }
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read serial input until newline

    char command[10];
    int numbers[3];
    replaying = false;

    // Parse the input
    int parsed = sscanf(input.c_str(), "%s %d %d %d ", command, &numbers[0], &numbers[1], &numbers[2]);

    if (parsed >= 1) {
      if (strcmp(command, "cartesian") == 0) {
        if (parsed == 4) {
          cartesian_control(numbers);
        } else {
          Serial.println("Invalid number of arguments for cartesian control.");
        }
      } else if (strcmp(command, "phase") == 0) {
        if (parsed == 4) {
          phase_control(numbers);
        } else {
          Serial.println("Invalid number of arguments for phase control.");
        }
      } else if (strcmp(command, "replay") == 0) {
        if (parsed == 4) {
          replay_control(numbers);
        } else {
          Serial.println("Invalid number of arguments for replay control.");
        }
      } else if (strcmp(command, "learn") == 0) {
        if (parsed == 1) {
          learn_mode();
        } else {
          Serial.println("Invalid arguments for learn mode.");
        }
      } else if (strcmp(command, "end") != 0){
        //end should terminate a loop command like learn
        Serial.println("Unknown command.");
      }
    } else {
      Serial.println("Invalid command format.");
    }
  }
}
