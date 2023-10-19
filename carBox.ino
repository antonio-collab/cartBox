#include <Bluepad32.h>

GamepadPtr myJoystick;
int leftMotorPWM = 12;        // Pino PWM para o motor esquerdo
int leftMotorDirectioIN1 = 13; // Pino para a direção 1 do motor esquerdo
int leftMotorDirectioIN2 = 27; // Pino para a direção 2 do motor esquerdo
int rightMotorPWM = 33;       // Pino PWM para o motor direito
int rightMotorDirectioIN3 = 25; // Pino para a direção 1 do motor direito
int rightMotorDirectioIN4 = 26; // Pino para a direção 2 do motor direito

void onConnectedGamepad(GamepadPtr gp) {
  myJoystick = gp;
  Serial.println("Conectado");
}

void onDisconnectedGamepad(GamepadPtr gp) {
  myJoystick = nullptr;
  Serial.println("Desconectado");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Aguardando conexão");

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDirectioIN1, OUTPUT);
  pinMode(leftMotorDirectioIN2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDirectioIN3, OUTPUT);
  pinMode(rightMotorDirectioIN4, OUTPUT);

  const uint8_t *addr = BP32.localBdAddress();
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
}

void controlMotors(int leftSpeed, int rightSpeed) {
  // Define a direção dos motores com base nos valores positivos ou negativos.
  if (leftSpeed >= 0) {
    digitalWrite(leftMotorDirectioIN1, HIGH);
    digitalWrite(leftMotorDirectioIN2, LOW);
  } else {
    digitalWrite(leftMotorDirectioIN1, LOW);
    digitalWrite(leftMotorDirectioIN2, HIGH);
  }

  if (rightSpeed >= 0) {
    digitalWrite(rightMotorDirectioIN3, HIGH);
    digitalWrite(rightMotorDirectioIN4, LOW);
  } else {
    digitalWrite(rightMotorDirectioIN3, LOW);
    digitalWrite(rightMotorDirectioIN4, HIGH);
  }

  int leftPWM = map(abs(leftSpeed), 0, 1023, 0, 255);
  int rightPWM = map(abs(rightSpeed), 0, 1023, 0, 255);

  // Aplique o PWM aos motores.
  analogWrite(leftMotorPWM, leftPWM);
  analogWrite(rightMotorPWM, rightPWM);
}

void loop() {
  BP32.update();
  GamepadPtr myGamepad = myJoystick;

  if (myGamepad && myGamepad->isConnected()) {
    int axisX = myGamepad->axisX();
    int axisY = myGamepad->axisY();
    int throttle = myGamepad->throttle();
// para o carro
  if (abs(axisX) < 100 && abs(axisY) < 100) {
      // Quando ambos os eixos estão próximos de zero, pare o carrinho.
      controlMotors(0, 0);
    }
 
    int leftMotorSpeed = axisY + axisX;
    int rightMotorSpeed = axisY - axisX;
// modo turbo
    if (throttle > 500) {
      leftMotorSpeed = 1023;
      rightMotorSpeed = 1023;
    }

    controlMotors(leftMotorSpeed, rightMotorSpeed);
  }

  delay(150);
}

