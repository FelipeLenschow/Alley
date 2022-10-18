#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>
#include <espnow.h>
#include <ESP8266WiFi.h>

/// 1 para ligar todos os Serial.print, 0 para desliga-los
#define DEBUG 0
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define AI1 4
#define AI2 5
#define BI1 13
#define BI2 12
#define W_pwm 14
#define V_bat A0
#define NUM_LEDS 2
#define LED_PIN 16

/// millis() do ultimo contato com o controle
unsigned long time_last_receive = 0;
/// Estrutura de comunicação EspNow
typedef struct struct_message
{
  /// Valor entre -255 e 255
  signed int motor_e;
  /// Valor entre -255 e 255
  signed int motor_d;
  /// Valor entre 0 e 180
  int arma_vel;
  /// Primeiro indice é o led, o segundo são os valores de RGB (0-255 cada)
  byte led[2][3];
} struct_message;

struct_message Controle;

/// Endereço do ESP32 do controle
uint8_t Controle_Address[2][6] = {
    {0x24, 0x0A, 0xC4, 0x58, 0xF1, 0xF0},
    {0xC4, 0x5B, 0xBE, 0x61, 0xFA, 0x3A}};

/// Leds decorativos
CRGB leds[2];
/// Arma rorativa BLDC
Servo Arma;

/// Aciona os motores em qualquer sentido
void Motor(signed int velocidade1, signed int velocidade2)
{

  if (velocidade1 > 0)
  {
    analogWrite(AI1, velocidade1);
    analogWrite(AI2, 0);
  }
  else
  {
    if (velocidade1 == 0)
    {
      analogWrite(AI1, 0);
      analogWrite(AI2, 0);
    }
    else
    {
      analogWrite(AI1, 0);
      analogWrite(AI2, -velocidade1);
    }
  }

  if (velocidade2 > 0)
  {
    analogWrite(BI1, velocidade2);
    analogWrite(BI2, 0);
  }
  else
  {
    if (velocidade2 == 0)
    {
      analogWrite(BI1, 0);
      analogWrite(BI2, 0);
    }
    else
    {
      analogWrite(BI1, 0);
      analogWrite(BI2, -velocidade2);
    }
  }
}
/// Desliga o robo quando perde contato com o controle
void FailSafe()
{
  if (millis() - time_last_receive > 500) // FAIL SAFE
  {
    Arma.writeMicroseconds(1500);
    Motor(0, 0);
    leds[0] = CRGB(0, 0, 0);
    leds[1] = CRGB(0, 0, 0);
    FastLED.show();
    Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
  }
}
/// Aciona o Hardware com os valores recebidos
void Value2Hardware(struct_message value)
{
  Motor(value.motor_e, value.motor_d);
  Arma.writeMicroseconds(value.arma_vel);
  if (analogRead(V_bat) >= 700)
  {
    leds[0] = CRGB(value.led[0][0], value.led[0][1], value.led[0][2]);
    leds[1] = CRGB(value.led[1][0], value.led[1][1], value.led[1][2]);
    FastLED.show();
  }
}
/// Calback de recepção de dados
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t data_len)
{

  if ((mac_addr[0] == Controle_Address[0][0] && mac_addr[1] == Controle_Address[0][1] && mac_addr[2] == Controle_Address[0][2]) 
  || (mac_addr[0] == Controle_Address[1][0] && mac_addr[1] == Controle_Address[1][1] && mac_addr[2] == Controle_Address[1][2]))
  {
    ESP.wdtFeed();
    memcpy(&Controle, data, sizeof(Controle));
    time_last_receive = millis();
    Value2Hardware(Controle);

    Serial.println("Recebeu");
    Serial.print(Controle.motor_d);
    Serial.print("  ");
    Serial.print(Controle.motor_e);
    Serial.print("  ");
    Serial.println(Controle.arma_vel);
  }
}
/// Liga o EspNow
void ConnectEspNow()
{
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0)
    debugln("ESPNow Init Failed");
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  debugln("nowReady");

 wifi_set_channel(10);
}

void setup()
{
  pinMode(AI1, OUTPUT);
  digitalWrite(AI1, LOW);
  pinMode(AI2, OUTPUT);
  digitalWrite(AI2, LOW);
  pinMode(BI1, OUTPUT);
  digitalWrite(BI1, LOW);
  pinMode(BI2, OUTPUT);
  digitalWrite(BI2, LOW);

  pinMode(W_pwm, OUTPUT);
  Arma.attach(W_pwm, 0, 2000);
  Arma.writeMicroseconds(10);
  ConnectEspNow();
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  pinMode(V_bat, INPUT);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

}

void loop()
{
  FailSafe();
  Serial.println(analogRead(V_bat));

  if (analogRead(V_bat) < 700)
  {
    leds[0] = CRGB(150, 0, 0);
    leds[1] = CRGB(150, 0, 0);
    FastLED.show();
  }
}
