/*
  ============================================================
  RECINTO DE CURADO - PROTOTIPO
  ============================================================

  Firmware: Revision 0
  Hardware: ESP32-WROOM / NodeMCU-ESP32S
  Entorno de desarrollo: Arduino IDE

  DESCRIPCION:
  Firmware de control para un prototipo de recinto de curado.
  Gestiona la interfaz de usuario, el control de los actuadores,
  la lectura de sensores, las condiciones de seguridad y el
  registro y consulta de los ciclos de funcionamiento.

  BIBLIOTECAS:
  - Keypad: lectura del teclado matricial.
  - LiquidCrystal_I2C: control de la pantalla LCD.
  - Preferences: almacenamiento no volatil de configuraciones.
  - WiFi: conexion a redes inalambricas.
  - WebServer: servidor web integrado en el ESP32.
  - HTTPClient: realizacion de solicitudes HTTP/HTTPS.
  - WiFiClientSecure: comunicacion segura mediante HTTPS.
  - Wire: comunicacion mediante bus I2C.
  - time: gestion de fecha y hora.
  - FreeRTOS: gestion de tareas y colas de comunicacion.
  - Arduino: funciones y definiciones basicas del entorno.

  CONSIDERACIONES IMPORTANTES:
  - Los valores de calibracion ADC de los LDR son provisionales.
  - La escala porcentual de los LDR requiere calibracion mediante
    mediciones reales.
  - El selector ON/OFF realiza un apagado logico y no interrumpe
    fisicamente la alimentacion del equipo.
  - En el equipo definitivo, la seguridad de apertura de tapa
    debe disponer de un sistema de corte de potencia independiente
    del firmware.

  ============================================================
*/

// ------------------------------------------------------------
// 1. INCLUSION DE BIBLIOTECAS
// ------------------------------------------------------------
#include <Arduino.h>            // Funciones y definiciones básicas del entorno Arduino
#include <Wire.h>               // Comunicación I2C
#include <Preferences.h>        // Para almacenamiento no-volatil de configuraciones
#include <Keypad.h>             // Lectura del teclado matricial
#include <LiquidCrystal_I2C.h>  // Control de la pantalla LCD mediante I2C
#include <WiFi.h>               // Conexión del ESP32 a redes Wi-Fi
#include <WebServer.h>          // Implementación del servidor web alojado en el ESP32
#include <time.h>               // Gestión de fecha y hora mediante funciones de tiempo
#include <HTTPClient.h>         // Envio HTTPS a Google Apps Script
#include <WiFiClientSecure.h>   // Cliente HTTPS para Google
#include <freertos/FreeRTOS.h>  // Gestión de tareas y colas para ejecutar y comunicar procesos concurrentes
#include <freertos/queue.h>     // Gestión de tareas y colas para ejecutar y comunicar procesos concurrentes
#include <freertos/task.h>      // Gestión de tareas y colas para ejecutar y comunicar procesos concurrentes

// -----------------------------------------------------------------------------
// 2. ASIGNACIÓN DE PINES
// -----------------------------------------------------------------------------
const uint8_t PIN_LDR_1       = 36; // Sensor de intensidad de luz 1
const uint8_t PIN_LDR_2       = 39; // Sensor de intensidad de luz 2
const uint8_t PIN_LM35        = 34; // Sensor de temperatura
const uint8_t PIN_LIMIT       = 35; // Límite de carrera
const uint8_t PIN_ON_OFF      = 17; // Tecla ON-OFF

const uint8_t PIN_BUZZER      = 18; // Indicador auditivo
const uint8_t PIN_LED_ROJO    = 19; // Led de estatus rojo
const uint8_t PIN_LED_VERDE   = 23; // Led de estatus verde
const uint8_t PIN_LED_CICLO   = 16; // Leds de curado
const uint8_t PIN_MOTOR       = 4;  // Cooler

const uint8_t PIN_LCD_SDA     = 21; // SDA de I2C
const uint8_t PIN_LCD_SCL     = 22; // SCL de I2C

// Definición de TECLADO MATRICIAL
const byte FILAS = 4;
const byte COLUMNAS = 3;
byte pinesFilas[FILAS] = {13, 14, 27, 26};
byte pinesColumnas[COLUMNAS] = {25, 33, 32};

char mapaTeclas[FILAS][COLUMNAS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

Keypad teclado = Keypad(makeKeymap(mapaTeclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);

// Definición de DISPLAY LCD
const uint8_t DIRECCION_LCD = 0x27;
LiquidCrystal_I2C lcd(DIRECCION_LCD, 16, 2);
Preferences preferencias;

// -----------------------------------------------------------------------------
// 3. PARAMETROS
// -----------------------------------------------------------------------------
// --- Periodos de lectura y antirrebote ---
const uint32_t PERIODO_LECTURA_MS = 200;  // Tiempo entre lecturas
const uint32_t REBOTE_ONOFF_MS = 40;      // Antirebote ON-OFF

// --- Umbrales de temperatura ---
const float TEMP_ENFRIAR_ON_C  = 40.0;    // Temperatura de activación COOLER
const float TEMP_ENFRIAR_OFF_C = 38.0;    // Temperatura de apagado COOLER
const float TEMP_CRITICA_C     = 50.0;    // Temperatura máxima

// --- Umbrales de iluminacion ---
const float CAIDA_LUZ_PUNTOS   = 10.0;    // Diferencia porcentual máxima durante curado
const float MARGEN_LUZ_PUNTOS = 10.0;     // Diferencia porcentual máxima al principio

// --- Temporizaciones del sistema ---
const uint32_t TIEMPO_MENSAJE_CONECTADO_MS = 3000;  // Tiempo que permanece en display CONECTADO
const uint32_t TIEMPO_INICIALIZACION_MS = 3000;     // Tiempo para inicialización
const uint32_t TIEMPO_MOSTRAR_PARAM_MS = 2000;      // Tiempo para mostrar parámetros
const uint32_t TIEMPO_REFERENCIA_LDR_MS = 1500;     // Tiempo de referencia para iluminación
const uint32_t INTERVALO_MENSAJE_FINAL_MS = 2000;   // Intervalo para cambio de mensaje final

// --- Configuracion PWM ---
// Resolucion de 8 bits: valores entre 0 y 255
const uint32_t FRECUENCIA_PWM = 5000;
const uint8_t RESOLUCION_PWM = 8;
const uint8_t CANAL_LED = 0;
const uint8_t CANAL_COOLER = 1;
const uint8_t PWM_COOLER = 128;       // 50 % PWM en COOLER (de 255)

// --- Calibracion de sensores LDR ---
// ADC_LUZ_0 = lectura correspondiente al 0 %.
// ADC_LUZ_100 = lectura correspondiente al 100 %.
int ADC_LDR1_LUZ_0 = 1000;
int ADC_LDR1_LUZ_100 = 2800;
int ADC_LDR2_LUZ_0 = 1000;
int ADC_LDR2_LUZ_100 = 2800;

// --- Configuracion inicial de curado ---
// Valores iniciales de parámetros si NVS aun no contiene configuracion valida.
int intensidadPorcentaje = 60;
uint16_t duracionSegundos = 60;

// --- Limites y pasos de configuracion ---
const int INTENSIDAD_MIN = 0;       // Rango de intensidad posible
const int INTENSIDAD_MAX = 100;
const uint16_t TIEMPO_MIN_S = 30;   // Rango de tiempo posible
const uint16_t TIEMPO_MAX_S = 120;
const uint16_t PASO_TIEMPO_S = 5;   // Paso para configuración de tiempo
const int PASO_INTENSIDAD = 5;      // Paso para configuración de intensidad

// -----------------------------------------------------------------------------
// 4. RED
// -----------------------------------------------------------------------------
// --- Conexión a red WIFI ---
const char* WIFI_SSID = "NOMBRE_RED";    // Nombre de red
const char* WIFI_PASSWORD = "CONTRASEÑA";   // Contraseña de red

// --- Configuración de fecha y hora ---
const char* NTP_SERVER = "pool.ntp.org";  // Servidor de sincronizacion horaria mediante NTP
const long GMT_OFFSET_SEC = -3L * 3600L;  // Corrección por zona horaria
const int DAYLIGHT_OFFSET_SEC = 0;        // Corrección por horario de verano

// --- Configuración de Google Apps Script: destino del historial de ciclos ---
const char* URL_GOOGLE_SCRIPT = "URL/exec";
const char* TOKEN_SECRETO = "TOKEN_SECRETO";

// --- Servidor web ---
WebServer servidor(80);  // Instancia del servidor HTTP del ESP32, utilizando el puerto 80

// --- Estados de conectividad y sincronizacion ---
bool wifiConectado = false;       // Indica si el ESP32 esta conectado a una red Wi-Fi
bool internetDisponible = false;  // Indica si se dispone de acceso a Internet
bool horaSincronizada = false;    // Indica si el reloj del ESP32 fue sincronizado mediante NTP

// --- Comprobacion periodica de Internet ---
uint32_t ultimaComprobacionInternetMs = 0;    // Instante de la ultima comprobacion, medido con millis()
const uint32_t INTERVALO_INTERNET_MS = 10000; // Intervalo entre comprobaciones de conectividad: 10 segundos

// --- Archivo de historial ---
const char* ARCHIVO_HISTORIAL = "/historial.jsonl";   // Ruta del archivo JSONL que almacena los registros de curado

// -----------------------------------------------------------------------------
// 5. VARIABLES GLOBALES
// -----------------------------------------------------------------------------
// --- Variables de medicion de sensores ---
int adcLdr1 = 0;                   // Lectura ADC del primer sensor LDR
int adcLdr2 = 0;                   // Lectura ADC del segundo sensor LDR
float porcentajeLdr1 = 0.0;        // Iluminacion del primer LDR, expresada en porcentaje
float porcentajeLdr2 = 0.0;        // Iluminacion del segundo LDR, expresada en porcentaje
float porcentajeLuzPromedio = 0.0; // Promedio de iluminacion de ambos sensores
float temperaturaC = 0.0;          // Temperatura medida, expresada en grados Celsius

// --- Estados de los actuadores ---
bool refrigeracionActiva = false;  // Indica si el sistema de refrigeracion esta activo
bool salidaCicloActiva = false;    // Indica si la salida de curado esta activa

// --- Control del selector ON/OFF ---
bool selectorEncendido = false;          // Estado interpretado del selector de encendido
bool ultimaLecturaSelector = false;      // Ultima lectura registrada del selector
uint32_t instanteCambioSelector = 0;     // Instante del ultimo cambio detectado en el selector

// --- Referencias temporales de funcionamiento ---
uint32_t inicioEstadoMs = 0;             // Instante de entrada al estado actual
uint32_t inicioCicloMs = 0;              // Instante de inicio del ciclo de curado
uint32_t duracionCicloMs = 0;             // Duracion del ciclo expresada en milisegundos
uint32_t instanteEncendidoLedsMs = 0;    // Instante de activacion de los LED de curado
uint32_t ultimaLecturaMs = 0;             // Instante de la ultima lectura periodica
uint32_t ultimoPuntoMs = 0;               // Referencia temporal para los puntos de seguimiento

// --- Estructura de datos para Google Sheets ---
struct RegistroCiclo {
  char fechaHoraInicio[24];              // Fecha y hora de inicio del ciclo
  char resultado[12];                    // Resultado del ciclo: completo o error
  char tipoError[32];                    // Descripcion del error, si corresponde
  uint32_t duracionSegundos;              // Duracion del ciclo en segundos
  int intensidadConfigurada;              // Intensidad seleccionada por el operador
  float intensidadSensadaPromedio;        // Iluminacion promedio medida durante el ciclo
};

// --- Comunicacion con Google Sheets ---
QueueHandle_t colaRegistrosGoogle = nullptr;

// --- Datos acumulados del ciclo ---
String fechaHoraInicioCiclo = "";         // Fecha y hora de inicio del ciclo actual
double sumaLuzPromedioCiclo = 0.0;         // Acumulador de las mediciones de iluminacion
uint32_t cantidadMuestrasCiclo = 0;       // Cantidad de muestras acumuladas durante el ciclo


// --- Referencia inicial de iluminacion ---
bool referenciaLuzPendiente = false;     // Indica si esta pendiente establecer la referencia inicial
float luzInicialCiclo = 0.0;             // Valor de iluminacion tomado como referencia del ciclo


// --- Control del LED verde de estado ---
uint32_t ultimoCambioLedVerdeMs = 0;      // Instante del ultimo cambio de estado del LED verde
bool estadoLedVerde = false;              // Estado actual del LED verde

// --- Secuencia de mensajes al finalizar el ciclo ---
uint32_t ultimoCambioMensajeFinalMs = 0;  // Instante del ultimo cambio de mensaje
bool mostrarRetirarPiezas = false;        // Indica si corresponde mostrar el mensaje para retirar las piezas

// --- Indicador de conexion ---
uint32_t ultimoCambioLedConectandoMs = 0; // Instante del ultimo cambio del LED de conexion


// -----------------------------------------------------------------------------
// 6. ESTADOS
// -----------------------------------------------------------------------------
// --- Estados generales del equipo ---
enum Estado {
  APAGADO_LOGICO,       // Equipo apagado mediante el selector logico
  INICIALIZACION,       // Secuencia de inicializacion del sistema
  MOSTRAR_PARAMETROS,   // Visualizacion de los parametros configurados
  STANDBY,              // Equipo encendido y en espera de iniciar un ciclo
  CONFIGURACION,        // Ajuste de los parametros de curado
  CURADO,               // Ejecucion y supervision del ciclo de curado
  FINALIZADO,           // Secuencia de finalizacion del ciclo
  ERROR_CICLO,          // Gestion de una condicion de error
  CONECTANDO,           // Proceso de conexion a la red
  CONECTADO             // Estado posterior a la conexion exitosa
};

Estado estado = APAGADO_LOGICO;  // Estado inicial del equipo

// --- Tipos de error del ciclo ---
enum ErrorCiclo {
  SIN_ERROR,                  // No hay errores registrados
  ERROR_TAPA,                 // Apertura de tapa durante el funcionamiento
  ERROR_LUZ_INSUFICIENTE,     // Iluminacion inicial inferior a la requerida
  ERROR_CAIDA_LUZ,            // Caida de iluminacion superior al limite permitido
  ERROR_TEMPERATURA           // Temperatura igual o superior al umbral critico
};

ErrorCiclo errorActual = SIN_ERROR;  // Error actualmente identificado

// --- Parametro seleccionado en el menu de configuracion ---
enum ParametroConfig {
  CONFIG_INTENSIDAD,  // Seleccion del ajuste de intensidad de iluminacion.
  CONFIG_DURACION     // Seleccion del ajuste de duracion del curado.
};

ParametroConfig parametroSeleccionado = CONFIG_INTENSIDAD; // Opcion seleccionada inicialmente

// -----------------------------------------------------------------------------
// 7. COMUNICACION CON GOOGLE SHEETS
// -----------------------------------------------------------------------------
/*
  Gestiona el registro y envio de informacion de los ciclos de curado a
  Google Sheets mediante Google Apps Script.

  Utiliza una cola y una tarea independiente de FreeRTOS para realizar los
  envios mediante HTTPS, evitando que los problemas de conectividad interfieran
  con el funcionamiento del control principal.

  Los registros que no pueden enviarse permanecen temporalmente en la cola
  de memoria RAM y se reintenta su envio cada 10 segundos.
  -----------------------------------------------------------------------------
*/

// Cantidad maxima de registros que puede almacenar la cola de envio
const uint8_t CANTIDAD_REGISTROS_EN_COLA = 10;

// Convierte los codigos de error en textos descriptivos
const char* textoErrorRegistro(ErrorCiclo error) {
  switch (error) {
    case ERROR_TAPA: return "TAPA ABIERTA";
    case ERROR_LUZ_INSUFICIENTE: return "LUZ INSUFICIENTE";
    case ERROR_CAIDA_LUZ: return "CAIDA DE LUZ";
    case ERROR_TEMPERATURA: return "TEMPERATURA";
    default: return "";
  }
}

// Encola los datos de un ciclo para su posterior envio a Google Sheets
void encolarRegistroCiclo(const char* resultado, const char* tipoError) {
  if (colaRegistrosGoogle == nullptr) {
    Serial.println("Google Sheets: cola no inicializada; registro no encolado.");
    return;
  }

  RegistroCiclo registro = {};
  snprintf(registro.fechaHoraInicio, sizeof(registro.fechaHoraInicio), "%s",
           fechaHoraInicioCiclo.c_str());
  snprintf(registro.resultado, sizeof(registro.resultado), "%s", resultado);
  snprintf(registro.tipoError, sizeof(registro.tipoError), "%s", tipoError);

  registro.duracionSegundos = (millis() - inicioCicloMs) / 1000UL;
  registro.intensidadConfigurada = intensidadPorcentaje;
  registro.intensidadSensadaPromedio = (cantidadMuestrasCiclo > 0)
      ? (float)(sumaLuzPromedioCiclo / cantidadMuestrasCiclo) : 0.0f;

  if (xQueueSend(colaRegistrosGoogle, &registro, 0) != pdTRUE) {
    Serial.println("Google Sheets: cola llena; no se pudo encolar el registro.");
  } else {
    Serial.println("Google Sheets: registro encolado para envio.");
  }
}

// Envia los registros pendientes a Google Sheets mediante una tarea independiente
void tareaEnviarRegistrosGoogle(void* parametro) 
{
  RegistroCiclo registro;

  for (;;) 
  {
    if (xQueueReceive(colaRegistrosGoogle, &registro, portMAX_DELAY) == pdTRUE) 
    {
      bool enviado = false;

      // Si no hay red o Google no responde, se conserva el registro en RAM
      // y se vuelve a intentar. Esta tarea no detiene el loop de control.
      while (!enviado) 
      {
        if (WiFi.status() != WL_CONNECTED) 
        {
          vTaskDelay(pdMS_TO_TICKS(10000));
          continue;
        }

        
        WiFiClientSecure clienteSeguro;
        clienteSeguro.setInsecure();

        HTTPClient http;
        http.setTimeout(15000);

        // Desactivamos las redirecciones automáticas.
        // Apps Script necesita POST inicial y GET para consultar la respuesta.
        http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);

        int codigoHTTP = -1;
        String respuesta = "";

        if (http.begin(clienteSeguro, URL_GOOGLE_SCRIPT)) 
        {
          // Capturar la dirección a la que Google redirige.
          const char* encabezados[] = {"Location"};
          http.collectHeaders(encabezados, 1);

          http.addHeader("Content-Type", "application/json");

          char json[384];
          snprintf(json, sizeof(json),
          "{\"token\":\"%s\",\"fechaHoraInicio\":\"%s\",\"resultado\":\"%s\",\"tipoError\":\"%s\",\"duracionSegundos\":%lu,\"intensidadConfigurada\":%d,\"intensidadSensadaPromedio\":%.2f}",
          TOKEN_SECRETO, registro.fechaHoraInicio, registro.resultado,
          registro.tipoError, (unsigned long)registro.duracionSegundos,
          registro.intensidadConfigurada, registro.intensidadSensadaPromedio);

          // Primera solicitud: POST a Apps Script.
          int codigoPost = http.POST((uint8_t*)json, strlen(json));

          String ubicacion = http.header("Location");

          if ((codigoPost == 301 || codigoPost == 302 || codigoPost == 303) && ubicacion.length() > 0) 
          {

            Serial.println("Google Sheets: redireccion recibida.");

            // Cerramos la primera conexión.
            http.end();

            // Segunda solicitud: GET a la dirección indicada por Google.
            // Se crea un cliente TLS nuevo para el segundo dominio.
            WiFiClientSecure clienteRespuesta;
            clienteRespuesta.setInsecure();

            HTTPClient httpRespuesta;
            httpRespuesta.setTimeout(15000);
            httpRespuesta.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

            if (httpRespuesta.begin(clienteRespuesta, ubicacion)) 
            {
              codigoHTTP = httpRespuesta.GET();
              respuesta = (codigoHTTP > 0) ? httpRespuesta.getString() : "";

              httpRespuesta.end();
            } else 
            {
              Serial.println("Google Sheets: fallo al abrir URL de respuesta.");
            }
          } else 
          {
            // Si no hubo redirección, conservamos la respuesta original.
            codigoHTTP = codigoPost;
            respuesta = (codigoPost > 0) ? http.getString() : "";

            http.end();
          }
        } else 
        {
          Serial.println("Google Sheets: no se pudo iniciar la conexion HTTPS.");
        }

        if (codigoHTTP >= 200 && codigoHTTP < 300 && respuesta.indexOf("\"ok\":true") >= 0) 
        {

          enviado = true;
          Serial.println("Google Sheets: registro enviado correctamente.");
        } else 
        {
          Serial.print("Google Sheets: fallo de envio. HTTP: ");
          Serial.println(codigoHTTP);

          if (respuesta.length() > 0) 
          {
            Serial.println(respuesta);
          }
        }
        if (!enviado) vTaskDelay(pdMS_TO_TICKS(10000));
      }
    }
  }
}

// Crea la cola y la tarea encargadas del envio a Google Sheets
void inicializarEnvioGoogle() {
  colaRegistrosGoogle = xQueueCreate(CANTIDAD_REGISTROS_EN_COLA,
                                     sizeof(RegistroCiclo));
  if (colaRegistrosGoogle == nullptr) {
    Serial.println("Google Sheets: no se pudo crear la cola de registros.");
    return;
  }

  BaseType_t tareaCreada = xTaskCreatePinnedToCore(
    tareaEnviarRegistrosGoogle,
    "EnviarGoogle",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );

  if (tareaCreada != pdPASS) {
    Serial.println("Google Sheets: no se pudo crear la tarea de envio.");
    vQueueDelete(colaRegistrosGoogle);
    colaRegistrosGoogle = nullptr;
  }
}

// -----------------------------------------------------------------------------
// 8. FUNCIONES DE HARDWARE Y CONFIGURACION
// -----------------------------------------------------------------------------
// Modifica la señal PWM de los LED de curado
void escribirPWMLED(uint8_t valor) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_LED_CICLO, valor);
#else
  ledcWrite(CANAL_LED, valor);
#endif
}

// Modifica la señal PWM del cooler
void escribirPWMCooler(uint8_t valor) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_MOTOR, valor);
#else
  ledcWrite(CANAL_COOLER, valor);
#endif
}

// Configura las salidas PWM y establece su estado inicial
void configurarPWM() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_LED_CICLO, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcAttach(PIN_MOTOR, FRECUENCIA_PWM, RESOLUCION_PWM);
#else
  ledcSetup(CANAL_LED, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcAttachPin(PIN_LED_CICLO, CANAL_LED);
  ledcSetup(CANAL_COOLER, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcAttachPin(PIN_MOTOR, CANAL_COOLER);
#endif
  escribirPWMLED(0);
  escribirPWMCooler(0);
}

// Aplica la intensidad de curado configurada mediante PWM
void aplicarIntensidadConfigurada() {
  int valor = map(intensidadPorcentaje, 0, 100, 0, 255);
  escribirPWMLED((uint8_t)constrain(valor, 0, 255));
}

// Apaga los LED de curado
void apagarSalidaCiclo() {
  salidaCicloActiva = false;
  escribirPWMLED(0);
}

// Actualiza la salida PWM del cooler según el estado de refrigeración
void aplicarCooler() {
  escribirPWMCooler(refrigeracionActiva ? PWM_COOLER : 0);
}

// Guarda los parámetros de configuración en memoria no volátil
void guardarConfiguracion() {
  preferencias.begin("recinto", false);
  preferencias.putInt("intensidad", intensidadPorcentaje);
  preferencias.putUShort("tiempo_s", duracionSegundos);
  preferencias.end();
}

// Recupera y valida los parámetros almacenados
void cargarConfiguracion() {
  preferencias.begin("recinto", true);
  intensidadPorcentaje = preferencias.getInt("intensidad", 60);
  duracionSegundos = preferencias.getUShort("tiempo_s", 60);
  preferencias.end();

  intensidadPorcentaje = constrain(intensidadPorcentaje, INTENSIDAD_MIN, INTENSIDAD_MAX);

  // Si hay un dato antiguo o corrupto, se reemplaza por un valor valido.
  if (duracionSegundos < TIEMPO_MIN_S ||
      duracionSegundos > TIEMPO_MAX_S ||
      ((duracionSegundos - TIEMPO_MIN_S) % PASO_TIEMPO_S != 0)) {
    duracionSegundos = 60;
  }
}

// Convierte la lectura ADC del LDR a una escala porcentual
float convertirLDRaPorcentaje(int lecturaADC, int adcLuz0, int adcLuz100) {
  if (adcLuz0 == adcLuz100) return 0.0;

  float valor = 100.0f * (lecturaADC - adcLuz0) / (float)(adcLuz100 - adcLuz0);
  return constrain(valor, 0.0f, 100.0f);
}

// Lee la temperatura del sensor LM35
float leerTemperaturaLM35() {
  // LM35DZ: 10 mV por grado Celsius.
  uint32_t milivoltios = analogReadMilliVolts(PIN_LM35);
  return milivoltios / 10.0f;
}

// Determina si la tapa del recinto está abierta.
bool tapaAbierta() {
  // Pull-up externo de 10 kOhm a 3,3 V: HIGH = tapa abierta.
  return digitalRead(PIN_LIMIT) == HIGH;
}

// Actualiza las mediciones de los sensores y acumula datos del ciclo
void leerSensores() {
  adcLdr1 = analogRead(PIN_LDR_1);
  adcLdr2 = analogRead(PIN_LDR_2);

  porcentajeLdr1 = convertirLDRaPorcentaje(
    adcLdr1, ADC_LDR1_LUZ_0, ADC_LDR1_LUZ_100);
  porcentajeLdr2 = convertirLDRaPorcentaje(
    adcLdr2, ADC_LDR2_LUZ_0, ADC_LDR2_LUZ_100);

  porcentajeLuzPromedio = (porcentajeLdr1 + porcentajeLdr2) / 2.0f;
  temperaturaC = leerTemperaturaLM35();

  // Acumulacion para el promedio de todo el ciclo; no modifica la supervision.
  if (estado == CURADO) {
    sumaLuzPromedioCiclo += porcentajeLuzPromedio;
    cantidadMuestrasCiclo++;
  }
}

// Gestiona el encendido y apagado del cooler según la temperatura
void gestionarTemperatura() {
  if (!refrigeracionActiva && temperaturaC > TEMP_ENFRIAR_ON_C) {
    refrigeracionActiva = true;
  } else if (refrigeracionActiva && temperaturaC <= TEMP_ENFRIAR_OFF_C) {
    refrigeracionActiva = false;
  }
  aplicarCooler();
}

// -----------------------------------------------------------------------------
// 9. PANTALLA: ACTUALIZACION SIN PARPADEO Y CARACTERES PERSONALIZADOS
// -----------------------------------------------------------------------------
// --- Caracteres tipo FLECHA ---
byte flechaArriba[8] = {
  B00100, B01110, B10101, B00100,
  B00100, B00100, B00100, B00000
};
byte flechaAbajo[8] = {
  B00100, B00100, B00100, B00100,
  B10101, B01110, B00100, B00000
};
byte flechaIzquierda[8] = {
  B00100, B01000, B11111, B01000,
  B00100, B00000, B00000, B00000
};
byte flechaDerecha[8] = {
  B00100, B00010, B11111, B00010,
  B00100, B00000, B00000, B00000
};

// Inicializa los caracteres personalizados utilizados en el LCD
void inicializarCaracteresLCD() {
  lcd.createChar(0, flechaArriba);
  lcd.createChar(1, flechaAbajo);
  lcd.createChar(2, flechaIzquierda);
  lcd.createChar(3, flechaDerecha);
}

// Imprime una línea de texto ajustada a las dimensiones del LCD
void imprimirLineaLCD(uint8_t fila, String texto) {
  if (texto.length() > 16) texto = texto.substring(0, 16);
  while (texto.length() < 16) texto += ' ';
  lcd.setCursor(0, fila);
  lcd.print(texto);
}

// Actualiza la pantalla evitando escrituras innecesarias y parpadeos
void actualizarPantalla(const String &linea1, const String &linea2) {
  static String ultimaLinea1 = "";
  static String ultimaLinea2 = "";
  static String ultimoTextoDesplazado = "";
  static uint32_t ultimoDesplazamientoMs = 0;
  static uint8_t desplazamiento = 0;

  String l1 = linea1.substring(0, 16);
  while (l1.length() < 16) l1 += ' ';

  String l2;
  if (estado == FINALIZADO && linea2.length() > 16) {
    // Las leyendas completas exceden las 16 columnas del LCD.
    // Se desplazan horizontalmente sin alterar la alternancia cada 2 s.
    if (linea2 != ultimoTextoDesplazado) {
      ultimoTextoDesplazado = linea2;
      desplazamiento = 0;
      ultimoDesplazamientoMs = millis();
    } else if (millis() - ultimoDesplazamientoMs >= 300) {
      ultimoDesplazamientoMs = millis();
      if (desplazamiento < linea2.length() - 16) desplazamiento++;
    }
    l2 = linea2.substring(desplazamiento, desplazamiento + 16);
  } else {
    l2 = linea2.substring(0, 16);
    ultimoTextoDesplazado = "";
    desplazamiento = 0;
  }

  while (l2.length() < 16) l2 += ' ';

  if (l1 != ultimaLinea1) {
    imprimirLineaLCD(0, l1);
    ultimaLinea1 = l1;
  }
  if (l2 != ultimaLinea2) {
    imprimirLineaLCD(1, l2);
    ultimaLinea2 = l2;
  }
}

// Reinicia el contenido y la memoria de actualización de la pantalla
void reiniciarCachePantalla() {
  // Fuerza la proxima actualizacion de ambas lineas.
  lcd.clear();
  actualizarPantalla(" ", " ");
}

// -----------------------------------------------------------------------------
// 10. TRANSICIONES DE ESTADO Y ERRORES
// -----------------------------------------------------------------------------
// Actualiza el estado operativo del equipo y sus temporizadores asociados
void cambiarEstado(Estado nuevoEstado) {
  estado = nuevoEstado;
  inicioEstadoMs = millis();

  if (nuevoEstado == STANDBY || nuevoEstado == FINALIZADO) {
    estadoLedVerde = false;
    ultimoCambioLedVerdeMs = millis();
  }

  if (nuevoEstado == FINALIZADO) {
    mostrarRetirarPiezas = false;
    ultimoCambioMensajeFinalMs = millis();
  }
}

// Gestiona los errores, detiene el curado y activa la alarma sonora
void activarError(ErrorCiclo nuevoError) {
  // Solo registrar si el error interrumpe un ciclo que ya estaba en CURADO.
  if (estado == CURADO) {
    encolarRegistroCiclo("ERROR", textoErrorRegistro(nuevoError));
  }
  apagarSalidaCiclo();
  referenciaLuzPendiente = false;
  errorActual = nuevoError;
  cambiarEstado(ERROR_CICLO);
  tone(PIN_BUZZER, 2200, 500);
}

// Prepara el equipo para iniciar la secuencia de encendido
void iniciarSecuenciaEncendido() {
  apagarSalidaCiclo();
  cargarConfiguracion();
  errorActual = SIN_ERROR;
  cambiarEstado(INICIALIZACION);
}

// Inicializa los parámetros y comienza un nuevo ciclo de curado
void iniciarCiclo() {
  if (tapaAbierta()) 
  {
    activarError(ERROR_TAPA);
    return;
  }

  errorActual = SIN_ERROR;
  duracionCicloMs = (uint32_t)duracionSegundos * 1000UL;
  fechaHoraInicioCiclo = obtenerFechaHora();
  sumaLuzPromedioCiclo = 0.0;
  cantidadMuestrasCiclo = 0;
  inicioCicloMs = millis();

  aplicarIntensidadConfigurada();
  salidaCicloActiva = true;
  instanteEncendidoLedsMs = millis();
  referenciaLuzPendiente = true;
  cambiarEstado(CURADO);

  tone(PIN_BUZZER, 1600, 120);
}

// Detiene el ciclo por solicitud del usuario
void detenerCicloPorUsuario() {
  if (estado == CURADO) {
    encolarRegistroCiclo("CANCELADO", "");
  }
  apagarSalidaCiclo();
  referenciaLuzPendiente = false;
  cambiarEstado(STANDBY);
  tone(PIN_BUZZER, 1000, 150);
}

// Finaliza el ciclo de curado y registra su resultado
void terminarCiclo() {
  if (estado == CURADO) {
    encolarRegistroCiclo("EXITOSO", "");
  }
  apagarSalidaCiclo();
  referenciaLuzPendiente = false;
  cambiarEstado(FINALIZADO);
  tone(PIN_BUZZER, 1800, 250);
}

// -----------------------------------------------------------------------------
// 11. LEDS DE ESTADO
// -----------------------------------------------------------------------------
// Actualiza los indicadores luminosos según el estado operativo del equipo
void actualizarLEDsEstado() {
  uint32_t ahora = millis();

  if (estado == STANDBY) 
  {
    // Cambia de estado cada 1 segundo: 1 s encendido / 1 s apagado.
    if (ahora - ultimoCambioLedVerdeMs >= 1000) 
    {
      ultimoCambioLedVerdeMs = ahora;
      estadoLedVerde = !estadoLedVerde;
    }
    digitalWrite(PIN_LED_VERDE, estadoLedVerde ? HIGH : LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
  else if (estado == INICIALIZACION)
  {
    digitalWrite(PIN_LED_VERDE, HIGH);
    digitalWrite(PIN_LED_ROJO, HIGH);
  }
  else if (estado == FINALIZADO) 
  {
    // Cambia de estado cada 0,5 segundos: 0,5 s encendido / 0,5 s apagado.
    if (ahora - ultimoCambioLedVerdeMs >= 500) 
    {
      ultimoCambioLedVerdeMs = ahora;
      estadoLedVerde = !estadoLedVerde;
    }
    digitalWrite(PIN_LED_VERDE, estadoLedVerde ? HIGH : LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
  else if (estado == CURADO) 
  {
    digitalWrite(PIN_LED_VERDE, HIGH);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
  else if (estado == ERROR_CICLO) 
  {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_ROJO, HIGH);
  }
  else if (estado == CONECTANDO) 
  {
    // Cambia de estado cada 0,2 segundos: 0,2 s encendido / 0,2 s apagado.
    if (ahora - ultimoCambioLedConectandoMs >= 200) 
    {
      ultimoCambioLedConectandoMs = ahora;
      estadoLedVerde = !estadoLedVerde;
    }
    digitalWrite(PIN_LED_VERDE, estadoLedVerde ? HIGH : LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
  else if (estado == CONECTADO) 
  {
    digitalWrite(PIN_LED_VERDE, HIGH);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
  else 
  {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  }
}

// -----------------------------------------------------------------------------
// 12. TECLADO Y CONFIGURACION
// -----------------------------------------------------------------------------
// Ingresa al modo de configuración y habilita la prueba de intensidad
void entrarConfiguracion() {
  parametroSeleccionado = CONFIG_INTENSIDAD;
  cambiarEstado(CONFIGURACION);
  aplicarIntensidadConfigurada();  // Prueba visual al nivel guardado.
  salidaCicloActiva = true;
}

// Procesa las entradas del teclado según el estado operativo del equipo
void gestionarTecla(char tecla) {
  if (tecla == NO_KEY) return;

  // Durante OFF e inicio no se aceptan comandos.
  if (estado == APAGADO_LOGICO ||
      estado == INICIALIZACION ||
      estado == MOSTRAR_PARAMETROS) return;

  if (estado == CURADO) {
    if (tecla == '#') detenerCicloPorUsuario();
    return;
  }

  if (estado == ERROR_CICLO) { 
    if ((tecla == '5' || tecla == '#') && !tapaAbierta() &&
        temperaturaC < TEMP_CRITICA_C) {
      errorActual = SIN_ERROR;
      cambiarEstado(STANDBY);
    }
    return;
  }

  if (estado == FINALIZADO) {
    if (tecla == '5' || tecla == '#') {
      cambiarEstado(STANDBY);
    }
    return;
  }

  if (estado == STANDBY) {
    if (tecla == '*') iniciarCiclo();
    else if (tecla == '5') {
      entrarConfiguracion();
    }
    return;
  }

  if (estado == CONFIGURACION) {
    switch (tecla) {
      case '2': // Flecha arriba
        parametroSeleccionado = CONFIG_INTENSIDAD;
        break;

      case '8': // Flecha abajo
        parametroSeleccionado = CONFIG_DURACION;
        break;

      case '4': // Flecha izquierda: disminuir
        if (parametroSeleccionado == CONFIG_INTENSIDAD) {
          intensidadPorcentaje = max(INTENSIDAD_MIN, intensidadPorcentaje - PASO_INTENSIDAD);
        } else {
          duracionSegundos = max((int)TIEMPO_MIN_S, (int)duracionSegundos - PASO_TIEMPO_S);
        }
        break;

      case '6': // Flecha derecha: aumentar
        if (parametroSeleccionado == CONFIG_INTENSIDAD) {
          intensidadPorcentaje = min(INTENSIDAD_MAX,
                                     intensidadPorcentaje + PASO_INTENSIDAD);
        } else {
          duracionSegundos = min((int)TIEMPO_MAX_S,
                                 (int)duracionSegundos + PASO_TIEMPO_S);
        }
        break;

      case '5': // ENTER: guardar en NVS
        guardarConfiguracion();
        apagarSalidaCiclo();
        cambiarEstado(STANDBY);
        tone(PIN_BUZZER, 1500, 100);
        break;

      case '#': // STOP: descartar cambios no guardados
        cargarConfiguracion();
        apagarSalidaCiclo();
        cambiarEstado(STANDBY);
        break;
    }
    if (estado == CONFIGURACION && parametroSeleccionado == CONFIG_INTENSIDAD)
      aplicarIntensidadConfigurada(); // PWM cambia inmediatamente con 4/6.
  }
}

// -----------------------------------------------------------------------------
// 13. SELECTOR ON/OFF
// -----------------------------------------------------------------------------
// Gestiona el selector ON/OFF mediante lectura con antirrebote
void gestionarSelectorOnOff() {
  // Pull-up externo de 10 kOhm a 3,3 V: LOW = ON, HIGH = OFF.
  bool lecturaEncendido = (digitalRead(PIN_ON_OFF) == LOW);

  if (lecturaEncendido != ultimaLecturaSelector) {
    ultimaLecturaSelector = lecturaEncendido;
    instanteCambioSelector = millis();
  }

  if (millis() - instanteCambioSelector >= REBOTE_ONOFF_MS &&
      lecturaEncendido != selectorEncendido) {

    selectorEncendido = lecturaEncendido;

    if (!selectorEncendido) {
      // Si se apaga durante un curado, registrar la interrupcion como cancelacion.
      if (estado == CURADO) {
        encolarRegistroCiclo("CANCELADO", "");
      }
      apagarSalidaCiclo();
      referenciaLuzPendiente = false;
      cambiarEstado(APAGADO_LOGICO);
      lcd.noBacklight();
    } else {
      iniciarSecuenciaEncendido();
    }
  }
}

// -----------------------------------------------------------------------------
// 14. ACTUALIZACION DE PANTALLA SEGUN ESTADO
// -----------------------------------------------------------------------------
// Actualiza los mensajes de pantalla según el estado operativo del equipo
void gestionarPantalla() {
  String l1 = "";
  String l2 = "";

  switch (estado) {
    case APAGADO_LOGICO:
      l1 = "";
      break;

    case INICIALIZACION:
      l1 = "Inicializando";
      break;

    case MOSTRAR_PARAMETROS:
      l1 = "Intensidad: " + String(intensidadPorcentaje) + "%";
      l2 = "Tiempo: " + String(duracionSegundos) + " s";
      break;

    case STANDBY:
      // Pantalla solicitada: solamente estas dos indicaciones.
      l1 = "START: INICIO";
      l2 = "ENTER: CONFIG";
      break;

    case CONFIGURACION:
      if (parametroSeleccionado == CONFIG_INTENSIDAD) {
        l1 = ">INTENSIDAD ";
        l1 += String(intensidadPorcentaje) + "%";
      } else {
        l1 = ">TIEMPO ";
        l1 += String(duracionSegundos);
        l1 += " s";
      }

      // En la segunda linea se muestran las funciones de las flechas.
      // Los caracteres 0..3 son flechas personalizadas del LCD.
      l2 = "";
      l2 += "MOVE:";
      l2 += char(0); // UP: seleccionar parametro anterior
      l2 += char(1); // DOWN: seleccionar parametro siguiente
      l2 += " ADJ:";
      l2 += char(2); // LEFT: disminuir
      l2 += char(3); // RIGHT: aumentar
      break;

    case CURADO: {
      uint32_t transcurrido = millis() - inicioCicloMs;
      uint32_t restante = (transcurrido >= duracionCicloMs)
                          ? 0 : duracionCicloMs - transcurrido;
      uint32_t segundos = restante / 1000UL;

      l1 = "Luz:";
      l1 += String((int)porcentajeLuzPromedio);
      l1 += "% T:";
      l1 += String(temperaturaC, 1);
      l1 += "C";

      l2 = "Restan ";
      l2 += String(segundos);
      l2 += " s";
      break;
    }

    case FINALIZADO:
      l1 = "CICLO COMPLETO";
      if (mostrarRetirarPiezas) l2 = "RETIRAR LAS PIEZAS";
      else l2 = "ENTER PARA CONTINUAR";
      break;

    case ERROR_CICLO:
      if (errorActual == ERROR_LUZ_INSUFICIENTE) {
        l1 = "ERROR: LUZ BAJA";
        l2 = "Ciclo abortado";
      } else if (errorActual == ERROR_TAPA) {
        l1 = "ERROR: TAPA";
        l2 = "Ciclo abortado";
      } else if (errorActual == ERROR_CAIDA_LUZ) {
        l1 = "ERROR: CAIDA LUZ";
        l2 = "Ciclo abortado";
      } else if (errorActual == ERROR_TEMPERATURA) {
        l1 = "ERROR: TEMP ALTA";
        l2 = "Ciclo abortado";
      }
      break;

    case CONECTANDO:
      l1 = "Conectando a RED";
      break;

    case CONECTADO:
      l1 = "Conectado OK";
      break;
    
  }

  actualizarPantalla(l1, l2);
}

// -----------------------------------------------------------------------------
// 15. FECHA, WIFI E INTERNET
// -----------------------------------------------------------------------------
// Inicia la conexión Wi-Fi y sincroniza la fecha y hora del sistema
void iniciarWiFi() 
{
  cambiarEstado(CONECTANDO);
  // Inicializar parpadeo del LED verde.
  //ultimoCambioLedConectandoMs = millis();
  //estadoLedVerde = true;
  actualizarLEDsEstado();
  gestionarPantalla();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 12000) 
  {
  // Actualizar parpadeo del LED.
  actualizarLEDsEstado();
  // Mostrar un punto cada 500 ms.
  if (millis() - ultimoPuntoMs >= 500)
  {
    ultimoPuntoMs = millis();
    Serial.println(".");
  }
  // Permitir que el ESP32 atienda sus tareas internas.
  yield();
  }
  wifiConectado = WiFi.status() == WL_CONNECTED;
  if (wifiConectado) 
  {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    struct tm t;
    horaSincronizada = getLocalTime(&t, 5000);
    internetDisponible = comprobarInternet();
    Serial.println("");
    Serial.println("WiFi conectado.");
    Serial.print("Direccion IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("");
    Serial.println("Fecha y hora recuperadas: ");
    Serial.print(obtenerFechaHora());
    cambiarEstado(CONECTADO);
    actualizarLEDsEstado();
    gestionarPantalla();
    delay(TIEMPO_MENSAJE_CONECTADO_MS);
  }
  ultimaComprobacionInternetMs = millis();
}

// Comprueba si existe acceso a Internet mediante una solicitud HTTP
bool comprobarInternet() 
{
  if (WiFi.status() != WL_CONNECTED) 
  {
    return false;
  }
  WiFiClient cliente;
  cliente.setTimeout(1500);
  bool ok = cliente.connect("clients3.google.com", 80);
  if (ok) 
  {
    cliente.print("GET /generate_204 HTTP/1.1\r\nHost: clients3.google.com\r\nConnection: close\r\n\r\n");
    uint32_t inicio = millis();
    while (cliente.connected() && !cliente.available() && millis() - inicio < 1500) delay(5);
    String primeraLinea = cliente.readStringUntil('\n');
    cliente.stop();
    return primeraLinea.indexOf("204") >= 0 || primeraLinea.indexOf("200") >= 0;
  }
  cliente.stop();
  return false;
}

// Actualiza el estado de conectividad y sincronización horaria
void actualizarEstadoRed() 
{
  wifiConectado = WiFi.status() == WL_CONNECTED;
  if (!wifiConectado) {
    internetDisponible = false;
    horaSincronizada = false;
    return;
  }
  if (millis() - ultimaComprobacionInternetMs >= INTERVALO_INTERNET_MS) {
    ultimaComprobacionInternetMs = millis();
    internetDisponible = comprobarInternet();
    if (internetDisponible && !horaSincronizada) {
      configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
      struct tm t;
      horaSincronizada = getLocalTime(&t, 1000);
    }
  }
}

// Obtiene la fecha y hora local en formato de texto
String obtenerFechaHora() 
{
  struct tm t;
  if (!getLocalTime(&t, 20)) return "";
  char buffer[24];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &t);
  return String(buffer);
}

// -----------------------------------------------------------------------------
// 16. SETUP
// -----------------------------------------------------------------------------
/*
  Se ejecuta una sola vez al iniciar el ESP32.

  Configura las entradas y salidas digitales, los sensores analogicos,
  la pantalla LCD y las salidas PWM. Recupera los parametros almacenados
  en memoria no volatil e inicializa las comunicaciones Wi-Fi y Google Sheets.

  Finalmente, verifica la posicion del selector ON/OFF, realiza las primeras
  lecturas de sensores y establece el estado operativo inicial del equipo.
  -----------------------------------------------------------------------------
*/
void setup() {
  // --- Configuración de entradas y salidas digitales ---
  pinMode(PIN_LIMIT, INPUT);   // Pull-up externo: HIGH = tapa abierta.
  pinMode(PIN_ON_OFF, INPUT);  // Pull-up externo: LOW = selector ON.

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);

  // --- Inicialización de las salidas ---
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_ROJO, LOW);
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_MOTOR, LOW);

  // --- Configuración del conversor analógico-digital ---
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LDR_1, ADC_11db);
  analogSetPinAttenuation(PIN_LDR_2, ADC_11db);
  analogSetPinAttenuation(PIN_LM35, ADC_11db);

  // --- Inicialización de la pantalla LCD ---
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();
  inicializarCaracteresLCD();

  // --- Configuración PWM y recuperación de parámetros ---
  configurarPWM();
  cargarConfiguracion();

  // --- Inicialización de comunicaciones ---
  Serial.begin(115200);

  inicializarEnvioGoogle();

  iniciarWiFi();

  // --- Lectura inicial del selector y los sensores ---
  selectorEncendido = (digitalRead(PIN_ON_OFF) == LOW);
  ultimaLecturaSelector = selectorEncendido;
  instanteCambioSelector = millis();

  leerSensores();
  gestionarTemperatura();

  // --- Definición del estado operativo inicial ---
  if (selectorEncendido) {
    iniciarSecuenciaEncendido();
  } else {
    cambiarEstado(APAGADO_LOGICO);
    lcd.noBacklight();
  }

  gestionarPantalla();
}

// -----------------------------------------------------------------------------
// 17. LOOP PRINCIPAL
// -----------------------------------------------------------------------------
/*
  Se ejecuta continuamente y coordina el funcionamiento general del equipo.

  Gestiona el selector ON/OFF, las comunicaciones, la lectura periodica
  de sensores, el control termico, el teclado y las transiciones entre
  estados operativos.

  Durante el ciclo de curado realiza las supervisiones de seguridad,
  controla la intensidad luminica y verifica el tiempo de funcionamiento,
  interrumpiendo el proceso ante condiciones de error o finalizando
  automaticamente al cumplirse el tiempo configurado.

  Finalmente, actualiza los indicadores luminosos y la pantalla LCD
  segun el estado actual del sistema.
  -----------------------------------------------------------------------------
*/
void loop() {
  // --- Gestión del selector y conectividad ---
  gestionarSelectorOnOff();
  actualizarEstadoRed();
  
  // --- Lectura periódica de sensores ---
  uint32_t ahora = millis();

  if (ahora - ultimaLecturaMs >= PERIODO_LECTURA_MS) 
  {
    ultimaLecturaMs = ahora;
    leerSensores();
    gestionarTemperatura();
  }

  // --- Transiciones temporizadas de inicialización ---
  if (estado == INICIALIZACION && ahora - inicioEstadoMs >= TIEMPO_INICIALIZACION_MS) 
  {
    cambiarEstado(MOSTRAR_PARAMETROS);
  }

  if (estado == MOSTRAR_PARAMETROS && ahora - inicioEstadoMs >= TIEMPO_MOSTRAR_PARAM_MS) {
    cambiarEstado(STANDBY);
  }

  // --- Alternancia de mensajes de finalización ---
  if (estado == FINALIZADO && ahora - ultimoCambioMensajeFinalMs >= INTERVALO_MENSAJE_FINAL_MS) {
    ultimoCambioMensajeFinalMs = ahora;
    mostrarRetirarPiezas = !mostrarRetirarPiezas;
  }

  // --- Lectura y procesamiento del teclado ---
  char tecla = teclado.getKey();
  gestionarTecla(tecla);

  // --- Supervisión de seguridad durante el curado ---
  if (estado == CURADO) {

    // --- Control de apertura de tapa ---
    if (tapaAbierta()) {
      activarError(ERROR_TAPA);
    }

    // --- Control de temperatura crítica ---
    else if (temperaturaC >= TEMP_CRITICA_C) {
      activarError(ERROR_TEMPERATURA);
    }
    else {

      // --- Obtención de referencia inicial de iluminación ---
      if (referenciaLuzPendiente &&
          ahora - instanteEncendidoLedsMs >= TIEMPO_REFERENCIA_LDR_MS) {

        luzInicialCiclo = porcentajeLuzPromedio;
        referenciaLuzPendiente = false;

        // La intensidad medida debe SUPERAR la intensidad configurada - 10 puntos.
        if (luzInicialCiclo <= ((float)intensidadPorcentaje - MARGEN_LUZ_PUNTOS) &&
          ahora - instanteEncendidoLedsMs >= TIEMPO_REFERENCIA_LDR_MS) {
          activarError(ERROR_LUZ_INSUFICIENTE);
        }
      }

      // --- Supervisión de caída de iluminación ---
      if (estado == CURADO && !referenciaLuzPendiente &&
          (luzInicialCiclo - porcentajeLuzPromedio) > CAIDA_LUZ_PUNTOS) {
        activarError(ERROR_CAIDA_LUZ);
      }

      // --- Control del tiempo de curado ---
      if (estado == CURADO &&
          ahora - inicioCicloMs >= duracionCicloMs) {
        terminarCiclo();
      }
    }
  }

  // --- Gestión del apagado lógico y retroiluminación ---
  if (estado == APAGADO_LOGICO) {
    apagarSalidaCiclo();
    referenciaLuzPendiente = false;
  } else {
    // El backlight se activa al salir del apagado logico.
    lcd.backlight();
  }

  actualizarLEDsEstado();
  gestionarPantalla();
}
