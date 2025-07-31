#include <Wire.h>              //Library required for I2C comms (LCD)
#include <LiquidCrystal_I2C.h> //Library for LCD display via I2C
#include <math.h>              //Mathematics library for pow function (CO2 computation)

// Debuging
#define HABILITAR_EMERGENCIA_CO2 false
#define HABILITAR_EMERGENCIA_SIMULADA true

// I/O pin labeling
#define LDR1 12 // LDR Light sensor from traffic light 1 connected in pin A0
#define LDR2 13 // LDR Light sensor from traffic light 2 connected in pin A1
#define CO2 14  // CO2 sensor connected in pin A3
#define P1 1    // Traffic light 1 button connected in pin 1
#define P2 2    // Traffic light 2 button connected in pin 2
#define CNY1 42 // Infrared sensor 1 in traffic light 1 connected in pin 42
#define CNY2 41 // Infrared sensor 2 in traffic light 1 connected in pin 41
#define CNY3 40 // Infrared sensor 3 in traffic light 1 connected in pin 40
#define CNY4 39 // Infrared sensor 4 in traffic light 2 connected in pin 39
#define CNY5 38 // Infrared sensor 5 in traffic light 2 connected in pin 38
#define CNY6 37 // Infrared sensor 6 in traffic light 2 connected in pin 37
#define LR1 5   // Red traffic light 1 connected in pin 5
#define LY1 4   // Yellow traffic light 1 connected in pin 4
#define LG1 6   // Green traffic light 1 connected in pin 6
#define LR2 7   // Red traffic light 2 connected in pin 7
#define LY2 15  // Yellow traffic light 2 connected in pin 15
#define LG2 16  // Green traffic light 2 connected in pin 16

// --- Constantes sensor CO2 ---
const float DC_GAIN = 8.5;                                                               // define the DC gain of amplifier CO2 sensor
const float ZERO_POINT_VOLTAGE = 0.265;                                                  // define the output of the sensor in volts when the concentration of CO2 is 400PPM
const float REACTION_VOLTAGE = 0.059;                                                    // define the “voltage drop” of the sensor when move the sensor from air into 1000ppm CO2
const float CO2Curve[3] = {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTAGE / (2.602 - 3))}; // Line curve with 2 points

// Variable definitions
float volts = 0; // Variable to store current voltage from CO2 sensor
float co2 = 0;   // Variable to store CO2 value

// Library definitions
LiquidCrystal_I2C lcd(0x27, 20, 4);

// --- Estructuras de control ---
// La struct ahora organiza los pines y el valor del sensor de cada semáforo.
struct Semaforo {
  uint8_t pinR, pinY, pinG, pinLDR, pinPeaton;
  uint8_t pinCNY[3];        // Pines sensores infrarojos
  int vLDR = 0;
  int pesoTrafico = 0;      // Congestion vehicular
  bool R = 0, Y = 0, G = 0; // Variables de control por semáforo de los LEDs
};

// Instancias de semáforos
Semaforo semaforos[2] = {
  {LR1, LY1, LG1, LDR1, P1, {CNY1, CNY2, CNY3}}, // Semaforo 1: (Via principal)
  {LR2, LY2, LG2, LDR2, P2, {CNY4, CNY5, CNY6}}  // Semaforo 2: (Via secundaria/Tunel)
};

// --- Logica MEF - Estados de la MEF de la Intersección ---
// Describe qué luz está verde (la otra estará en rojo/amarillo)
#define VERDE_S1_ROJO_S2    0
#define AMARILLO_S1_ROJO_S2 1
#define ROJO_S1_VERDE_S2    2
#define ROJO_S1_AMARILLO_S2 3
#define PASO_PEATONAL_S1    4 // Peatones de la calle 1 cruzan
#define PASO_PEATONAL_S2    5 // Peatones de la calle 2 cruzan
int estadoMEF = VERDE_S1_ROJO_S2;

// --- Modos de Operación (Tarea 4) ---
#define MODO_DIA 0
#define MODO_NOCHE 1
int modoOperacion = MODO_DIA;

// --- Umbrales de Sensores (Tareas 4 y 5) ---
#define UMBRAL_LUZ_NOCHE 600     // Valor bajo para LDR indica oscuridad
//#define UMBRAL_CO2_PELIGROSO 800 // 800 PPM de CO2 para cerrar el túnel
#define UMBRAL_LUZ_EMERGENCIA 99 // Umbral muy bajo para simulacion C02 con LDR2

// --- Variables de Petición Peatonal ---
bool peticionPeatonS1 = false;
bool peticionPeatonS2 = false;
bool ultimoEstadoBotonS1 = HIGH;
bool ultimoEstadoBotonS2 = HIGH;

// --- Variables de control de Tiempo no bloqueante ---
long unsigned tini, tactual, tdelta;

// --- Variables para almacenar valores de sensores ---
float dCO2 = 0; // Valor de CO2 en PPM

void setup() {
  // Input pin config
  pinMode(P1, INPUT);   // Traffic light 1 button as Input
  pinMode(P2, INPUT);   // Traffic light 2 button as Input
  pinMode(CNY1, INPUT); // Infrared sensor 1 in traffic light 1 as Input
  pinMode(CNY2, INPUT); // Infrared sensor 2 in traffic light 1 as Input
  pinMode(CNY3, INPUT); // Infrared sensor 3 in traffic light 1 as Input
  pinMode(CNY4, INPUT); // Infrared sensor 4 in traffic light 2 as Input
  pinMode(CNY5, INPUT); // Infrared sensor 5 in traffic light 2 as Input
  pinMode(CNY6, INPUT); // Infrared sensor 6 in traffic light 2 as Input
  
  // Output pin config
  pinMode(LR1, OUTPUT); // Red traffic light 1 as Output
  pinMode(LY1, OUTPUT); // Yellow traffic light 1 as Output
  pinMode(LG1, OUTPUT); // Green traffic light 1 as Output
  pinMode(LR2, OUTPUT); // Red traffic light 2 as Output
  pinMode(LY2, OUTPUT); // Yellow traffic light 2 as Output
  pinMode(LG2, OUTPUT); // Green traffic light 2 as Output
  
  // Output cleaning
  digitalWrite(LR1, LOW); // Turn Off Red traffic light 1
  digitalWrite(LY1, LOW); // Turn Off Yellow traffic light 1
  digitalWrite(LG1, LOW); // Turn Off Green traffic light 1
  digitalWrite(LR2, LOW); // Turn Off Red traffic light 2
  digitalWrite(LY2, LOW); // Turn Off Yellow traffic light 2
  digitalWrite(LG2, LOW); // Turn Off Green traffic light 2
  
  // Communications
  Serial.begin(115200); // Start Serial communications with computer via Serial0 (TX0 RX0) at 9600 bauds
  lcd.init();
  lcd.backlight(); // Turn on LCD backlight

  // Init sensors
  for (int i = 0; i < 2; i++) {
    pinMode(semaforos[i].pinR, OUTPUT);
    pinMode(semaforos[i].pinY, OUTPUT);
    pinMode(semaforos[i].pinG, OUTPUT);
    pinMode(semaforos[i].pinLDR, INPUT);
    pinMode(semaforos[i].pinPeaton, INPUT_PULLUP);
    // Init CNY pines
    for (int j = 0; j < 3; j++) {
      pinMode(semaforos[i].pinCNY[j], INPUT_PULLUP);
    }
  }

  // --- Inicialización ---
  tini = millis(); // Inicializa el temporizador
  lcd.print("Sistema Iniciado...");
  Serial.println("Smart City - Sistema Control Trafico - Nivel Bajo - INICIADO");
  delay(1500);

  // Sincroniza estado inicial de P1 y P2
  ultimoEstadoBotonS1 = digitalRead(semaforos[0].pinPeaton);
  ultimoEstadoBotonS2 = digitalRead(semaforos[1].pinPeaton);
  
  lcd.clear();
  lcd.print("Sistema Listo.");
  delay(500);
}

void loop() {
  // 1. MEDIR: Obtiene los datos del entorno.
  readAllSensors();
  medirTiempo();
  
  // 2. GESTIONAR: Decide el estado global del sistema. La emergencia tiene prioridad.
  // La función devuelve 'true' Si hay emergencia, detiene el ciclo normal
  if (gestionarCondicionesGlobales()) return;
  
  // 3. CONTROLAR: Ejecuta la lógica de la Máquina de Estados de los semáforos.
  controlarSemaforos();
  actuarSemaforos();
  
  // 4. MOSTRAR: Actualiza la pantalla LCD con la información relevante.
  displayInfo();
}
  
// --- Functions and Subroutines ---
void readAllSensors() {
  // Init Light Sensors
  semaforos[0].vLDR = analogRead(semaforos[0].pinLDR);
  delay(10);
  semaforos[1].vLDR = analogRead(semaforos[1].pinLDR);
  delay(10);
  
  // Tarea 2 - Logica de deteccion de pulsacion antirebote
  // Si se presiona botón de peatón, se activa la petición al semaforo indicado.
  
  bool estadoActualBotonS1 = digitalRead(semaforos[0].pinPeaton);
  if (estadoActualBotonS1 == LOW && ultimoEstadoBotonS1 == HIGH) {
    peticionPeatonS1 = true;
    Serial.println(">>> Peticion Peaton S1 REGISTRADA");
  }
  ultimoEstadoBotonS1 = estadoActualBotonS1;

  bool estadoActualBotonS2 = digitalRead(semaforos[1].pinPeaton);
  if (estadoActualBotonS2 == LOW && ultimoEstadoBotonS2 == HIGH) {
    peticionPeatonS2 = true;
    Serial.println(">>> Peticion Peaton S2 REGISTRADA");
  }
  ultimoEstadoBotonS2 = estadoActualBotonS2;


  // Tarea 3: Calcular peso del tráfico para cada calle
  // Sensores CNY: LOW = coche detectado. Le damos más peso al sensor más cercano.
  semaforos[0].pesoTrafico = (!digitalRead(semaforos[0].pinCNY[0]) * 1) + 
                             (!digitalRead(semaforos[0].pinCNY[1]) * 2) + 
                             (!digitalRead(semaforos[0].pinCNY[2]) * 3);
  semaforos[1].pesoTrafico = (!digitalRead(semaforos[1].pinCNY[0]) * 1) + 
                             (!digitalRead(semaforos[1].pinCNY[1]) * 2) + 
                             (!digitalRead(semaforos[1].pinCNY[2]) * 3);
  
  // C02 read
  float volts = analogRead(CO2) * 3.3 / 4096.0;
  if (volts / DC_GAIN >= ZERO_POINT_VOLTAGE) {
    dCO2 = 400; // Se asume ~400ppm para aire limpio
  } else {
    dCO2 = pow(10, ((volts / DC_GAIN) - CO2Curve[1]) / CO2Curve[2] + CO2Curve[0]);
  }
}

void medirTiempo() {
  tactual = millis();
  tdelta = tactual - tini;
}

bool gestionarCondicionesGlobales() {
  // Tarea 5: Override de Emergencia del Túnel (Prioridad Máxima)
  //if (HABILITAR_EMERGENCIA_CO2 && dCO2 > UMBRAL_CO2_PELIGROSO) {
  // Simulacion LDR2
  if (HABILITAR_EMERGENCIA_SIMULADA && semaforos[1].vLDR < UMBRAL_LUZ_EMERGENCIA) {  // Si el LDR2 detecta oscuridad extrema (<100), se cierra el túnel.
    actuarSemaforos(semaforos[1], 1, 0, 0); // S2 a Rojo para detener el tráfico hacia el túnel
    actuarSemaforos(semaforos[0], 0, 0, 1); 

    // Muestra un display especial para la emergencia
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("   MODO EMERGENCIA    ");
    lcd.setCursor(0, 1); lcd.print("  Tunel CERRADO (S2)  ");
    lcd.setCursor(0, 2); lcd.print(" EVACUANDO RUTA (S1)  ");
    lcd.setCursor(0, 3); lcd.print("LDR2:"); lcd.print(semaforos[1].vLDR);
    
    delay(1000); 
    return true; // Hay una emergencia, se detiene el ciclo normal.
  }

  // Tarea 4: Determinar si es de día o de noche ---
  /*
  int promedioLuz = (semaforos[0].vLDR + semaforos[1].vLDR) / 2;
  //Serial.print(">>> Luz promedio: "); Serial.println(promedioLuz);
  int modoAnterior = modoOperacion;
  if (promedioLuz < UMBRAL_LUZ_NOCHE) {
    modoOperacion = MODO_NOCHE;
  } else {
    modoOperacion = MODO_DIA;
  }
  */
  // Simulacion LDR1 
  // Día/Noche ahora depende exclusivamente del sensor de luz de la vía principal LDR2 (S2)
  int modoAnterior = modoOperacion;
  if (semaforos[0].vLDR < UMBRAL_LUZ_NOCHE) {
    modoOperacion = MODO_NOCHE;
  } else {
    modoOperacion = MODO_DIA;
  }
  
  // Si hay un cambio de modo
  if(modoAnterior != modoOperacion){
    Serial.print("CAMBIO DE MODO DE OPERACION -> ");
    Serial.println((modoOperacion == MODO_DIA) ? "DIA" : "NOCHE");
    estadoMEF = VERDE_S1_ROJO_S2;
    // Reiniciamos el timer para empezar el nuevo ciclo/estado limpiamente
    tini = millis();
  }
  return false; // No hay emergencia, proceder con el control normal.
}

void controlarSemaforos() {

  Semaforo& s1 = semaforos[0];
  Semaforo& s2 = semaforos[1];

  // El comportamiento del sistema depende del MODO DE OPERACIÓN GLOBAL
  // Ajustar duraciones según el modo de operación
  if (modoOperacion == MODO_DIA) { // --- Logica de la MEF para el MODO DÍA ---

    // Tiempos fijos
    long duracionAmarillo = 2000;
    long duracionPasoPeatonal = 6000;
    // Dinamic Tiempo Verde
    long duracionVerdeBase = 5000; // 5 segundos base
    long duracionVerdeS1 = duracionVerdeBase;
    long duracionVerdeS2 = duracionVerdeBase;

    // Tarea 3: Duración del verde ahora es dinámica
    // Si hay + trafico en una calle, se le añade tiempo de verde a esa calle
    // y se le resta a la otra, manteniendo el ciclo total similar
    if (s1.pesoTrafico > s2.pesoTrafico) {
      duracionVerdeS1 += 3000; // Añade 3s
      duracionVerdeS2 -= 2000; // Quita 2s
    } else if (s2.pesoTrafico > s1.pesoTrafico) {
      duracionVerdeS2 += 3000;
      duracionVerdeS1 -= 2000;
    }
    // Asegurarse de que el tiempo de verde no sea demasiado corto
    if (duracionVerdeS1 < 3000) duracionVerdeS1 = 3000;
    if (duracionVerdeS2 < 3000) duracionVerdeS2 = 3000;

    /*
    // DEBUG: Imprime los pesos y tiempos calculados
    static int ultimoPesoS1 = -1, ultimoPesoS2 = -1;
    if (s1.pesoTrafico != ultimoPesoS1 || s2.pesoTrafico != ultimoPesoS2) {
      Serial.print(">> Pesos Trafico -> S1: "); Serial.print(s1.pesoTrafico);
      Serial.print(" | S2: "); Serial.println(s2.pesoTrafico);
      Serial.print("   Tiempos Verdes Calculados -> S1: "); Serial.print(duracionVerdeS1);
      Serial.print("ms | S2: "); Serial.print(duracionVerdeS2); Serial.println("ms");
      ultimoPesoS1 = s1.pesoTrafico;
      ultimoPesoS2 = s2.pesoTrafico;
    }
    */
    
    switch (estadoMEF) {
      case VERDE_S1_ROJO_S2:
      s1.G = 1; s1.Y = 0; s1.R = 0;
      s2.G = 0; s2.Y = 0; s2.R = 1;
      
        if (tdelta >= duracionVerdeS1) {
          estadoMEF = AMARILLO_S1_ROJO_S2;
          tini = millis();
          Serial.println("MEF: S1 -> Amarillo, S2 -> Rojo");
        }
        break;
    
      case AMARILLO_S1_ROJO_S2:
        s1.G = 0; s1.Y = 1; s1.R = 0;
        s2.G = 0; s2.Y = 0; s2.R = 1;
        
        if (tdelta >= duracionAmarillo) {
          if (peticionPeatonS2) {
          // Hay un peaton para S2 esperando? (S2 esta en rojo, es seguro)
            estadoMEF = PASO_PEATONAL_S2;
            Serial.println("MEF: Peticion -> PASO PEATONAL S2");
          } else { // No hay peatón, ciclo normal
          estadoMEF = ROJO_S1_VERDE_S2;
          Serial.println("MEF: S1 -> Rojo, S2 -> Verde");
          }
          tini = millis();
        }
        break;
    
      case ROJO_S1_VERDE_S2:
        s1.G = 0; s1.Y = 0; s1.R = 1;
        s2.G = 1; s2.Y = 0; s2.R = 0;
      
        if (tdelta >= duracionVerdeS2) {
          estadoMEF = ROJO_S1_AMARILLO_S2;
          tini = millis();
          Serial.println("MEF: S1 -> Rojo, S2 -> Amarillo");
        }
        break;
      
      case ROJO_S1_AMARILLO_S2:
        s1.G = 0; s1.Y = 0; s1.R = 1;
        s2.G = 0; s2.Y = 1; s2.R = 0;
        
        if (tdelta >= duracionAmarillo) {
          if (peticionPeatonS1) { 
            // Hay un peaton para S1 esperando? (S1 esta en rojo, es seguro)
            estadoMEF = PASO_PEATONAL_S1;
            Serial.println("MEF: Peticion -> PASO PEATONAL S1");
          } else { // No hay peatón, ciclo normal
            estadoMEF = VERDE_S1_ROJO_S2;
            Serial.println("MEF: S1 -> Verde, S2 -> Rojo");
          }
          tini = millis();
        }
        break;
        
      case PASO_PEATONAL_S1:
        s1.G = 0; s1.Y = 0; s1.R = 1; 
        s2.G = 0; s2.Y = 0; s2.R = 1;
        
        if (tdelta >= duracionPasoPeatonal) {
          peticionPeatonS1 = false;
          estadoMEF = ROJO_S1_VERDE_S2; // Reanuda dando paso a la otra calle (S2)
          tini = millis();
          Serial.println("MEF: Fin paso peatones S1. Reanudando -> S2 Verde");
        }
        break;
    
      case PASO_PEATONAL_S2:
        s1.G = 0; s1.Y = 0; s1.R = 1; 
        s2.G = 0; s2.Y = 0; s2.R = 1;
        if (tdelta >= duracionPasoPeatonal) {
          peticionPeatonS2 = false;
          estadoMEF = VERDE_S1_ROJO_S2; // Reanuda dando paso a la otra calle (S1)
          tini = millis();
          Serial.println("MEF: Fin paso peatones S2. Reanudando -> S1 Verde");
        }
        break;  
    }
  } else { // --- Logica para el MODO NOCHE ---
    long flashInterval = 500; // Parpadeo cada 500ms

    // La vía principal (S1) parpadea en amarillo (precaución).
    s1.G = 0; s1.R = 0;
    if (tdelta > flashInterval) s1.Y = !s1.Y; // Invierte el estado de Y cada 500ms
     
    // La vía secundaria/túnel (S2) parpadea en rojo (detenerse y proceder).
    s2.G = 0; s2.Y = 0;
    if (tdelta > flashInterval) s2.R = !s2.R; // Invierte el estado de R cada 500ms
      
    if (tdelta > flashInterval) tini = millis(); // Resetea el timer del parpadeo
  }
  
}


void actuarSemaforos(Semaforo& s, bool R, bool Y, bool G) {
  digitalWrite(s.pinR, R);
  digitalWrite(s.pinY, Y);
  digitalWrite(s.pinG, G);
}

void actuarSemaforos() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(semaforos[i].pinR, semaforos[i].R);
    digitalWrite(semaforos[i].pinY, semaforos[i].Y);
    digitalWrite(semaforos[i].pinG, semaforos[i].G);
  }
}

// Función para mostrar información relevante en el LCD
void displayInfo() {
  static long lastDisplayTime = 0;
  if (tactual - lastDisplayTime > 500) {
    lastDisplayTime = tactual;
    lcd.clear();
    
    // --- Fila 0: Modo y Peticiones Pendientes ---
    lcd.setCursor(0, 0);
    lcd.print("Modo:");
    lcd.print((modoOperacion == MODO_DIA) ? "DIA  " : "NOCHE");
    if(peticionPeatonS1) { lcd.setCursor(12,0); lcd.print("P1*"); }
    if(peticionPeatonS2) { lcd.setCursor(16,0); lcd.print("P2*"); }

    if (modoOperacion == MODO_DIA || estadoMEF == PASO_PEATONAL_S1 || estadoMEF == PASO_PEATONAL_S2) {
        // --- Display para MODO DÍA ---
        long duracionTotal = 0;
        int countdown = 0;
        
        switch(estadoMEF) {
            case VERDE_S1_ROJO_S2:
                duracionTotal = (semaforos[0].pesoTrafico > semaforos[1].pesoTrafico) ? 8000 : (semaforos[1].pesoTrafico > semaforos[0].pesoTrafico ? 3000 : 5000);
                countdown = (duracionTotal - tdelta)/1000;
                lcd.setCursor(0,1); lcd.print("S1:VERDE T-"); lcd.setCursor(11,1); lcd.print(countdown < 0 ? 0 : countdown);
                lcd.setCursor(0,2); lcd.print("S2:ROJO");
                break;
            case AMARILLO_S1_ROJO_S2:
                duracionTotal = 2000; countdown = (duracionTotal - tdelta)/1000 + 1;
                lcd.setCursor(0,1); lcd.print("S1:AMARILLO T-"); lcd.print(countdown < 0 ? 0 : countdown);
                lcd.setCursor(0,2); lcd.print("S2:ROJO");
                break;
            case ROJO_S1_VERDE_S2:
                duracionTotal = 8000; countdown = (duracionTotal - tdelta)/1000 + 1;
                lcd.setCursor(0,1); lcd.print("S1:ROJO");
                lcd.setCursor(0,2); lcd.print("S2:VERDE   T-"); lcd.print(countdown < 0 ? 0 : countdown);
                break;
            case ROJO_S1_AMARILLO_S2:
                duracionTotal = 2000; countdown = (duracionTotal - tdelta)/1000 + 1;
                lcd.setCursor(0,1); lcd.print("S1:ROJO");
                lcd.setCursor(0,2); lcd.print("S2:AMARILLO T-"); lcd.print(countdown < 0 ? 0 : countdown);
                break;
            case PASO_PEATONAL_S1:
            case PASO_PEATONAL_S2:
                duracionTotal = 6000; countdown = (duracionTotal - tdelta)/1000 + 1;
                lcd.setCursor(0,1); lcd.print("   PASE PEATONAL    ");
                lcd.setCursor(0,2); lcd.print("     TIEMPO: "); lcd.print(countdown < 0 ? 0 : countdown); lcd.print("s");
                break;
        }
    } else {
        // --- Display para MODO NOCHE ---
        lcd.setCursor(0, 1); lcd.print("S1: AMARILLO INTERM.");
        lcd.setCursor(0, 2); lcd.print("S2: ROJO INTERM.");
    }
   
    // --- Fila 3: Sensores ---
    lcd.setCursor(0, 3);
    lcd.print("L");     // luz 2
    lcd.print(semaforos[1].vLDR);
    //lcd.print((semaforos[0].vLDR + semaforos[1].vLDR)/2);
    lcd.setCursor(5, 3);
    lcd.print("TS1:"); // trafico
    lcd.print(semaforos[0].pesoTrafico);
    lcd.setCursor(10, 3);
    lcd.print("S2:");
    lcd.print(semaforos[1].pesoTrafico);
    /*
    lcd.setCursor(14, 3);
    lcd.print("C:");
    lcd.print(dCO2, 0);
    */
    lcd.setCursor(14, 3);
    lcd.print("L:"); lcd.print(semaforos[0].vLDR);
  }
}