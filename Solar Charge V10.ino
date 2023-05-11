#include <Wire.h>
#include <LiquidCrystal_I2C.h>

String frase;                  // Para la frase que queremos mostrar en el lcd
int ancho = 20;                // ancho pantalla en caracteres
int alto = 4;                  // alto pantalla en líneas
int backlightTimeout = 5000;   // Tiempo en ms antes de apagar la luz de fondo
unsigned long lastSerialTime;  // Tiempo en ms desde la última trama recibida

LiquidCrystal_I2C lcd(0x27, ancho, alto);

void setup() {
  // Mensaje de bienvenida
  lcd.init();                   // Iniciamos la pantalla LCD
  lcd.backlight();              // Encendemos la luz de fondo de la pantalla
  lcd.clear();                  // Limpiamos pantalla
  lcd.setCursor(0, 0);          // Posición del cursor en la primera línea
  lcd.print("Bienvenido!");

  lcd.setCursor(0, 1);          // Posición del cursor en la segunda línea
  lcd.print("Este es un texto");

  lcd.setCursor(0, 2);          // Posición del cursor en la tercera línea
  lcd.print("que ocupa 4 líneas");

  lcd.setCursor(0, 3);          // Posición del cursor en la cuarta línea
  lcd.print("en la pantalla LCD");

  // Iniciamos la comunicación serie
  Serial.begin(9600);

  // Guardamos el tiempo actual para empezar el temporizador
  lastSerialTime = millis();
}

void loop() {
  if (Serial.available() > 0) {
    // Si hay datos disponibles en el puerto serie, leemos la trama
    String trama = Serial.readStringUntil('\n');

    // Actualizamos la pantalla con la trama recibida
    lcd.clear();                    // Limpiamos pantalla
    lcd.setCursor(0, 0);            // Posición del cursor en la primera línea
    lcd.print(trama);               // Imprimimos la trama

    // Encendemos la luz de fondo
    lcd.backlight();

    // Actualizamos el tiempo de la última trama recibida
    lastSerialTime = millis();
  }

  // Si ha pasado el tiempo límite, apagamos la luz de fondo
  if (millis() - lastSerialTime > backlightTimeout) {
    lcd.noBacklight();
  }
}
