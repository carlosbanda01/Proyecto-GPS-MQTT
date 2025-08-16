#define SERIAL_RX_BUFFER_SIZE 1024
#define TINY_GSM_MODEM_SIM800
#include <TinyGPSPlus.h>//para entender la trama recibida del GPS
#include <HardwareSerial.h>//para los buses UART-GPS
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>  // Para la pantalla LCD
#include <SD.h>  // Para la tarjeta SD
#include <BigFont01_I2C.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <regex>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Pantalla LCD 20x4
BigFont01_I2C big(&lcd);

Preferences preferences; //objeto preferences para recordar el tiempo

// Configuración de los pines de la comunicacion con el GPS
  static const int GPSRXPin = 26;  // Pin RX en el ESP32
  static const int GPSTXPin = 25;  // Pin TX en el ESP32
  static const uint32_t GPSBaud = 9600;  // Velocidad de comunicación del GPS

// Configuración de los pines de la comunicacion con el SIM800L
  static const int SIMRXPin = 16;  // Pin RX en el ESP32
  static const int SIMTXPin = 17;  // Pin TX en el ESP32
  static const uint32_t SIMBaud = 115200;  // Velocidad de comunicación del SIM

// Pines para la tarjeta SD
SPIClass spiSD(HSPI);  // nueva clase de SPI

// El objeto TinyGPSPlus
TinyGPSPlus gps;

// Usar Serial2 para la conexión con el GPS
HardwareSerial ss(2);  // Usamos UART2 del ESP32 (RX26,TX25)

//Usaremos Serial1 para la conexion con el SIM800L
HardwareSerial sim800(1);  // Usamos UART1 del ESP32 (RX16,TX17)

//Funcion para determinar el area:
  // Definir los vértices del polígono como pares (latitud, longitud)
  const int numVerticesArea1 = 12;  // Número de vértices en el polígono
  float area1_latitudes[] = {-4.928024, -4.928935, -4.929426, -4.92947, -4.928583, -4.928535, -4.928357, -4.928233, -4.928123, -4.927939, -4.927852, -4.927458};  // Coordenadas de latitud
  float area1_longitudes[] = {-80.633414, -80.632668, -80.632724, -80.632165, -80.632069, -80.632619, -80.63242, -80.632527, -80.632376, -80.632516, -80.632392, -80.632712};  // Coordenadas de longitud

  //definir la funcion q dara 0 y 1 depndiendo del area 1 y area 2
  bool isPointInPolygon(float pointLat, float pointLng, float* latitudes, float* longitudes, int numVertices) {
    bool inside = false;
    for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
      if (((latitudes[i] > pointLat) != (latitudes[j] > pointLat)) &&
          (pointLng < (longitudes[j] - longitudes[i]) * (pointLat - latitudes[i]) / (latitudes[j] - latitudes[i]) + longitudes[i])) {
        inside = !inside;
      }
    }
    return inside;
  }

  //Declaramos la variable inside_area1
  bool insideArea1 = false;  // Variable global

//Declaramos el pinvelocidad 
  const int ledvel = 14;
//Contadores
  int attempt_connect_count = 0;
  int attempt_apn_count = 0;
  int max_attempts = 3;

//String dataString para guardar datos en CSV
  String dataString="";

//Configuracion APN:
  const char* APN = "movistar.pe";//chip.entel.pe para entel y claro.pe para claro movistar.pe pa movistar
  const char* USER = "movistar@datos";// "" para entel y claro, movistar@datos pa movistar
  const char* PWD = "movistar";// "" para entel y claro, movistar para movistar

// Configuración del broker MQTT
  const char* mqtt_server = "nanomq.culqui.io";  //mqtts://qd6acf83.ala.us-east-1.emqxsl.com
  const int mqtt_port = 1883;                     //8883
  const char* mqtt_user = "montacargas";                //montacargas, esp32_test
  const char* mqtt_password = "Hp7f#gWsGeXTzP1s";  //Qid8kULggQvJAr:m, esp32_test, Hp7f#gWsGeXTzP1s
  const char* client_id = "CBC_3";  // Client ID
  const char* topic1 = "cbc/gps/datareceived/CBC_3";   //Topico donde se enviarán los datos
  const char* topic2 = "cbc/gps/datasend";       //topico donde se enviaran vel_max
  const char* topic3 = "cbc/gps/historyconsult"; //topico donde se enviara la consulta de datos historicos
  const char* topic4 = "cbc/gps/historydata/CBC_3";    //topico donde se recibiran datos historicos
  const char* topic5 = "cbc/gps/deleteSD";       //topico donde se enviara un comando para borrar la SD
  const char* topic6 = "cbc/gps/stateESP32/CBC_3";     //topico para mandar mensajes del estado del ESP32
  const char* topiclwt = "cbc/gps/off/CBC_3";   //topico para last will
  const char* lwt = "CBC_3 off";        //Mensaje de last will
//Creamos la tarea Task2
  TaskHandle_t Task2;

//Nombre de los archivos a guardar en la SD y la clave de borrado
  String fileName = "/gps_data_"+String(client_id)+".csv";
  String velFile = "/vel_max_"+String(client_id)+".txt";
  String report_fail="/failures_"+String(client_id)+".txt";
  String tempFileName = "/temp.csv"; // Nombre del archivo temporal para la funcion deleteSD()
  String del_SD = "Delete_TGI"; //clave para autorizar el borrado de datos

//variables de recepcion para topic2, topic3 y topic5
  bool new_vel_max= false;
  int vel_in_max= 10 ;
  int new_vel_in_max= 10 ;
  int vel_out_max= 15 ;
  int new_vel_out_max= 15 ;
  bool historic_data = false;
  String start_date = "";
  String start_time = "";
  String end_date = "";
  String end_time = "";
  String d_start_date = "";
  String d_start_time = "";
  String d_end_date = "";
  String d_end_time = "";
  String clave= "";
  bool del_data= false;
  float currentLat = 0;
  float currentLng = 0;
  float vel_sat = 0;

//Tren de comunicacion entre nucleos
  bool pack_ready = false;  //sirve para permitir que el core 0 coge el semaforo
  bool hasData = false;     //sirve para activar un if si el GPS no envio nada
  String jsonPayload;       //Data shared resources
  //Creamos un semaforo
  SemaphoreHandle_t xMutex;

//Configuración del cliente GSM y MQTT
  TinyGsm modem(sim800);
  TinyGsmClient gsmClient(modem);
  PubSubClient mqttClient(gsmClient);

void setup(){  
  xMutex=xSemaphoreCreateMutex();
  
  //Iniciar el puerto UART para la comunicacion con el GPS
  ss.begin(GPSBaud, SERIAL_8N1, GPSRXPin, GPSTXPin);
  
  //Iniciamos la LCD
  initializeLCD();
  
  //Iniciamos la comunicacion con la SIM800L
  initializeSIM();

  //Configuramos y nos conectamos al broker
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  connectToMQTT();

  //Iniciamos la SD
  initializeSD();

  //Declaramos el pin de salida para el rele
  pinMode(ledvel,OUTPUT);
  //timeSet();//configuramos preferences para almacenar tiempo

  //Caracterizamos al core0
  xTaskCreatePinnedToCore(
    loop2,
    "Task_2",
    4096,
    NULL,
    1,
    &Task2,
    0);
  mqttClient.publish(topic6, ("Modulo "+String(client_id)+ " iniciado con exito").c_str());
}

void loop(){
  if (xSemaphoreTake(xMutex, portMAX_DELAY)){//Cojo el semaforo/banderin
    //"Escucho" por 1 segundo lo recibido por el GPS
    for (unsigned long start = millis(); millis() - start < 1000;){
      while (ss.available() > 0) {
        char c = ss.read();
        gps.encode(c);
        hasData=true;
      }
    }
    
    // Verifica si se ha recibido una nueva ubicación
    if (gps.location.isValid() && gps.location.isUpdated()) {
      displayInfo();//muestro en la LCD la velocidad
      saveDataToCSV();//guardo los datos de interes en la SD
      controlVel();//activo/desactivo el rele de control
      armedJson();//armo mi payload a ser enviado
      pack_ready= true;//habilito al loop2 a enviar
    }

    else if (!hasData){//Si el GPS no esta conectado
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("No hay GPS");
      mqttClient.publish(topic6, ("Fallo GPS en "+String(client_id)).c_str());
    }
    
    else {//Si el GPS no esta recibiendo la suficiente informacion
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Captando señal ...");
      saveDataToCSV_off();
      armedJson();
      pack_ready= true;//habilito al loop2 a enviar datos repetidos
    }

    //funciones excepcionales
    if (new_vel_max){//Si se recibio por comando nuevas velocidades maximas
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("  Se cambiaron las  ");
      lcd.setCursor(0, 1);
      lcd.print("velocidades maximas:");
      lcd.setCursor(0, 2);
      lcd.print("Interna: ");
      lcd.print(new_vel_in_max);
      lcd.setCursor(0, 3);
      lcd.print("Externa: ");
      lcd.print(new_vel_out_max);
      //Se reescribe vel_max.txt
      new_config();
      readVel_max();
      new_vel_max= false;
    }

    if(del_data){//Si se recibio el comando de borrar datos
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Borrando data");
      lcd.setCursor(0, 1);
      lcd.print("Historica");
      delete_SD();
      del_data= false;
    }

    //Han pasado 8 dias ?
    /*if (timeRead()) {
      mqttClient.publish(topic6, "¡Han pasado 8 días! Ejecutando acción.");
      File root = SD.open("/");
      // Leer todos los archivos en el directorio raíz
      File file = root.openNextFile();
      while (file) {
        String fileName = file.name(); // Obtener el nombre del archivo
        String Path = "/" + fileName;
        // Ignorar el archivo vel_max.txt
        if (fileName != "vel_max.txt") {

          // Eliminar el archivo
          SD.remove(fileName);
        }
        file = root.openNextFile(); // Siguiente archivo
      }
      root.close(); // Cerrar el directorio raíz
    }*/
    xSemaphoreGive(xMutex); //libero el mutex/semaforo/banderin
  }
}

void loop2(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static bool mqtt_connected = false;
  for (;;) {
    // Intentar reconectar si no estamos conectados
    /*if (!mqtt_connected) {
      mqtt_connected = reconnectToMQTT(); // Intentar Reconectar a MQTT
      if (!mqtt_connected) {
        vTaskDelay(pdMS_TO_TICKS(2000)); // Esperar antes de volver a intentar
        continue; // Saltar if(pack_ready....) y volver a intentar reconnectMQTT()
      }
    }*/
    // Si el paquete está listo y estamos conectados al broker
    if (pack_ready) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) { //el loop2 coje el mutex/semaforo/banderin
        // Enviar datos al broker MQTT
        sendToMQTTJson();
        //Funciones ocasionales
        if (historic_data) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Enviando data");
          lcd.setCursor(0, 1);
          lcd.print("Historica");
          send_historic_data();
          historic_data = false;
        }
        xSemaphoreGive(xMutex); // Liberar el mutex/semaforo/banderin
        pack_ready = false; // Marcar el paquete como procesado
      }
    }
    // Intervalo fijo de espera para evitar saturar la CPU
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
}

//Funcion de inicio de la LCD
void initializeLCD() {
  lcd.init();      // Inicializar la pantalla LCD
  lcd.backlight(); // Encender la retroiluminación
  big.begin();     //Habilitamos la libreria BigFont
  
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");
  delay(1000);
}

//Funcion de inicio del modulo SIM800L
void initializeSIM() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Esperando SIM800");
  delay(1000);
  sim800.begin(SIMBaud, SERIAL_8N1, SIMRXPin, SIMTXPin);//configuramos la comunicacion con el SIM

  // Intentar iniciar el módem en un bucle
  while (!modem.restart()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error al iniciar el módem");
    lcd.setCursor(0, 1);
    lcd.print("reintentando....");
    delay(2000);  // Espera 2 segundos antes de intentar nuevamente
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("SIM800 iniciado");
  delay(1000);

  // Intentar conectar a GPRS en bucle
  while (!modem.gprsConnect(APN, USER, PWD)  && attempt_apn_count < max_attempts) {
    attempt_apn_count++; 

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Fallo configurando");
    lcd.setCursor(0, 1);
    lcd.print("APN");
    lcd.setCursor(0,2);
    lcd.print("reintentando....");
    delay(2000);  // Espera 2 segundos antes de intentar nuevamente
  }
  
  if(modem.gprsConnect(APN, USER, PWD)){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("APN configurado");
    delay(1000);
  }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Fallo APN");
    delay(1000);
  }
}

//Configuracion Callback para recibir mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  // Convertir el payload a String y mostrarlo
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);

  //Si hubo algun problema al deserializar el payload
  if (error) {
    
    mqttClient.publish(topic6, ("Error en "+String(client_id)+" al analizar el payload recibido").c_str());
    
    return;
  }

  //si el ID no se especifico o es no le corresponde
  if (!doc.containsKey("ID") || doc["ID"] != client_id) {
    
    mqttClient.publish(topic6, ("ID no coincide con "+String(client_id)).c_str());
    
    return;
  }
  
  String callback_use = "Mensaje recibido en " + String(client_id) + " desde el tópico: " + String(topic)+"\n";
  callback_use += "Contenido: ";
  mqttClient.publish(topic6, callback_use.c_str());

  // Enviar solo el payload en otro mensaje MQTT
  mqttClient.publish(topic6, message.c_str());
  
  // Procesar según el tópico
  if (strncmp(topic, topic2, strlen(topic2)) == 0) {   // Recibo 2 valores (vel_in_max y vel_out_max)
    if (doc.containsKey("vel_in_max") && doc.containsKey("vel_out_max")) {
      // Verifico que ambos valores sean enteros y positivos
      if (doc["vel_in_max"].is<int>() && doc["vel_in_max"] > 0 && doc["vel_out_max"].is<int>() && doc["vel_out_max"] > 0) {
        new_vel_max = true;
        // Extraigo las velocidades máximas válidas del payload recibido
        new_vel_in_max = doc["vel_in_max"];
        new_vel_out_max = doc["vel_out_max"];
      }
      else {

        mqttClient.publish(topic6, ("Valores inválidos registrados por "+String(client_id)+" en el topic "+String(topic2)).c_str());
     
      }
    }
    else {
     
      mqttClient.publish(topic6, ("Comando incorrecto registrado por "+String(client_id)+" en el topic "+String(topic2)).c_str());
   
    }
  }

  else if (strncmp(topic, topic3, strlen(topic3)) == 0) { // Recibo 2 fechas y 2 horas (YYYY-MM-DD y HH:MM:SS)
    if (doc.containsKey("start_date") && doc.containsKey("start_time") && doc.containsKey("end_date") && doc.containsKey("end_time")) {
      // Extraer los valores y verificar que sean cadenas
      String temp_start_date = doc["start_date"].as<String>();
      String temp_start_time = doc["start_time"].as<String>();
      String temp_end_date = doc["end_date"].as<String>();
      String temp_end_time = doc["end_time"].as<String>();

      // Validar formato de fecha y hora
      std::regex date_regex("^\\d{4}-\\d{2}-\\d{2}$");
      std::regex time_regex("^\\d{2}:\\d{2}:\\d{2}$");

      if (std::regex_match(temp_start_date.c_str(), date_regex) && std::regex_match(temp_start_time.c_str(), time_regex) && std::regex_match(temp_end_date.c_str(), date_regex) && std::regex_match(temp_end_time.c_str(), time_regex)) {

        historic_data = true;

        // Asignar los valores validados
        start_date = temp_start_date;
        start_time = temp_start_time;
        end_date = temp_end_date;
        end_time = temp_end_time;
      }
      else {

        mqttClient.publish(topic6, ("Formatos inválidos registrados por "+String(client_id)+" en el topic "+String(topic3)).c_str());
      
      }
    }
    else {

      mqttClient.publish(topic6, ("Comando incorrecto o rangos incompletos registrado por "+String(client_id)+" en el topic " + String(topic3)).c_str());
    
    }
  }
  
  else if (strncmp(topic, topic5, strlen(topic5)) == 0) { //ACA RECIBO 0 o 1 (1: borrar todos los datos de la SD)
    if(doc.containsKey("Clave")){ //verifico si se envio una clave de borrado
      clave= doc["Clave"].as<String>();
      //verifico que la clave sea del_SD sea valido para activar deleteSD()
      if (clave == del_SD){
        if (doc.containsKey("d_start_date") && doc.containsKey("d_start_time") &&
          doc.containsKey("d_end_date") && doc.containsKey("d_end_time")){
          // Extraer los valores y verificar que sean cadenas
          String temp_start_date = doc["d_start_date"].as<String>();
          String temp_start_time = doc["d_start_time"].as<String>();
          String temp_end_date = doc["d_end_date"].as<String>();
          String temp_end_time = doc["d_end_time"].as<String>();

          // Validar formato de fecha y hora
          std::regex date_regex("^\\d{4}-\\d{2}-\\d{2}$");
          std::regex time_regex("^\\d{2}:\\d{2}:\\d{2}$");

          if (std::regex_match(temp_start_date.c_str(), date_regex) &&
            std::regex_match(temp_start_time.c_str(), time_regex) &&
            std::regex_match(temp_end_date.c_str(), date_regex) &&
            std::regex_match(temp_end_time.c_str(), time_regex)) {

            del_data = true;

            // Asignar los valores validados
            d_start_date = temp_start_date;
            d_start_time = temp_start_time;
            d_end_date = temp_end_date;
            d_end_time = temp_end_time;

            String msg = "Borrando datos en "+String(client_id)+" desde:\n"+ d_start_date +" "+d_start_time +" hasta " +d_end_date+ " "+d_end_time;
          
            mqttClient.publish(topic6, msg.c_str());
          
          }
          else{

            mqttClient.publish(topic6, ("Formatos de fecha u hora inválidos ingresados a "+String(client_id)+" en el topic " + String(topic5)).c_str());

          }
        }
        else{
         
          mqttClient.publish(topic6, ("Rangos de borrado incompletos ingresados a "+String(client_id)+" en el topic "+String(topic5)).c_str());
        
        }
      }
      else{
        mqttClient.publish(topic6, ("Clave de borrado incorrecta ingresada en "+String(client_id)+" en el topic "+String(topic5)).c_str());
      }
    }
    else{
      mqttClient.publish(topic6, ("No se ingreso clave de borrado en "+String(client_id)+" en el topic "+String(topic5)).c_str());
    }
  }
}

// Función para conectar al broker MQTT (solo para setup)
void connectToMQTT() {
  // Configurar Keep Alive antes de conectar
  mqttClient.setKeepAlive(10);  // Configurar Keep Alive en 10s

  while (!mqttClient.connected() && attempt_connect_count < max_attempts) { // Reintentar hasta 3 veces
    attempt_connect_count++; // Incrementar contador de intentos
    
    if (mqttClient.connect(client_id, mqtt_user, mqtt_password, topiclwt, 0, true, lwt)) { // Configura la conexión al broker
      // Me suscribo a los tópicos donde recibiré información
      mqttClient.subscribe(topic2);
      mqttClient.subscribe(topic3);
      mqttClient.subscribe(topic5);

      // Grafico estado de conectado
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Conectado a internet");
      
      mqttClient.publish(topic6, (String(client_id) + " conectado a internet").c_str());

      delay(1000);
      break; // Salir del bucle al conectar
    }
    else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Fallo la conexion");
      lcd.setCursor(0, 1);
      lcd.print("a internet");
      lcd.setCursor(0, 2);
      lcd.print("Reintentando... ");
      delay(1000); // Espera antes de reintentar
    }
  }
}

//Funcion de inicio de SD
void initializeSD() {
  // Inicializar la tarjeta SD con reintento
  spiSD.begin(18, 19, 27, 5);
  while (!SD.begin(5, spiSD)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error en SD");

    mqttClient.publish(topic6, ("Error de SD en "+String(client_id)).c_str());

    delay(1000);  // Esperar un momento antes de intentar de nuevo
  }

  // Confirmar que la tarjeta SD fue inicializada correctamente
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SD inicializada");

  mqttClient.publish(topic6, ("SD Iniciada en "+String(client_id)).c_str());

  delay(1000);

  // Crear o abrir un archivo en la tarjeta SD
  SDFileCreation();
  //Leer la configuracion de velocidades
  readVel_max();
}

//subFuncion de initializeSD para crear o abrir un archivo en la SD
void SDFileCreation() {
  // Intentar crear o abrir el archivo gps_data.csv en un bucle hasta que se logre
  while (!SD.exists(fileName)) {
    File file = SD.open(fileName, FILE_WRITE);
    if (file) {
      // Si es la primera vez, escribir la cabecera en el archivo
      file.println("ID,Fecha,Hora,Latitud,Longitud,Precision,Velocidad,Area_in_out");
      file.close();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Se creo archivo");
      lcd.setCursor(0, 1);
      lcd.print(fileName);
      delay(1000);

      mqttClient.publish(topic6, ("Archivo "+fileName+" creado en "+String(client_id)).c_str());

      break;
    }
    else {
      // Mostrar mensaje de error y esperar antes de intentar nuevamente
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error al crear");
      lcd.setCursor(0, 1);
      lcd.print(fileName);
      lcd.setCursor(0, 2);
      lcd.print("reintentando....");

      mqttClient.publish(topic6, ("No se pudo crear "+fileName+" en "+String(client_id)).c_str());

      delay(1000);
    }
  }

  mqttClient.publish(topic6, ("Archivo "+fileName+" existente en "+String(client_id)).c_str());

  //creo el archivo vel_max.txt por si no existiese
  while(!SD.exists(velFile)){
    File velMaxFile = SD.open(velFile, FILE_WRITE);
    if (velMaxFile) {
      //si es la primera vez escribimos 10,15
      velMaxFile.println("10,15");  // Escribir los valores iniciales
      velMaxFile.close();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Se creo");
      lcd.setCursor(0, 1);
      lcd.print(velFile);

      mqttClient.publish(topic6, ("Archivo "+velFile+" creado en "+String(client_id)).c_str());

      delay(1000);
      break;
    }
    else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("No se creo");
      lcd.setCursor(0, 1);
      lcd.print(velFile);
      lcd.setCursor(0, 2);
      lcd.print("reintentando....");

      mqttClient.publish(topic6, ("No se pudo crear "+velFile+" en "+String(client_id)).c_str());

      delay(1000);
    }
  }

  mqttClient.publish(topic6, ("Archivo "+velFile+" existente en "+String(client_id)).c_str());

  //creo el archivo vel_max.txt por si no existiese
  while(!SD.exists(report_fail)){
    File errorFile = SD.open(report_fail, FILE_WRITE);
    if (errorFile) {
      //si es la primera vez escribimos 10,15
      errorFile.println("Aca se muestra un listado de todos los errores de red");  // Escribir los valores iniciales
      errorFile.close();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Se creo");
      lcd.setCursor(0, 1);
      lcd.print(report_fail);

      mqttClient.publish(topic6, ("Archivo "+report_fail+" creado en "+String(client_id)).c_str());

      delay(1000);
      break;
    }
    else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("No se creo");
      lcd.setCursor(0, 1);
      lcd.print(velFile);
      lcd.setCursor(0, 2);
      lcd.print("reintentando....");

      mqttClient.publish(topic6, ("No se pudo crear "+report_fail+" en "+String(client_id)).c_str());

      delay(1000);
    }
  }

  mqttClient.publish(topic6, ("Archivo "+report_fail+" existente en "+String(client_id)).c_str());

}

//subFuncion de inicializeSD para leer la vel_max
void readVel_max() {
  File archivo = SD.open(velFile); // Abrir archivo
  if (archivo) {
    String contenido = archivo.readStringUntil('\n'); // Leer línea completa (valor1,valor2)
    int separador = contenido.indexOf(','); // Buscar coma
    if (separador != -1) {//si existe una coma
      vel_in_max = contenido.substring(0, separador).toInt(); // Parte antes de la coma
      vel_out_max = contenido.substring(separador + 1).toInt(); // Parte después de la coma
    }
    archivo.close(); // Cerrar archivo
  }
  String read_vel_max_use="En "+String(client_id)+" se definio "+velFile+":\n";
  read_vel_max_use += "vel_in_max=" + String(vel_in_max) + "\n";
  read_vel_max_use += "vel_out_max=" + String(vel_out_max);

  mqttClient.publish(topic6, read_vel_max_use.c_str());

}

//Funcion para "recordar" el tiempo
/*void timeSet() {
  preferences.begin("time-tracker", false); // Inicia el espacio NVS
  // Verifica si ya existe un tiempo guardado
  if (!preferences.isKey("startTime")) {
    unsigned long currentMillis = millis();  // Obtén el tiempo actual
    preferences.putULong("startTime", currentMillis); // Guarda la marca de tiempo
    mqttClient.publish(topic6, "Se establecio una marca de tiempo");
  } else {
    mqttClient.publish(topic6, "Apagado repentino registrado");
  }
  preferences.end(); // Cierra el espacio NVS
}*/

//Funcion para mostrar valores en la LCD
void displayInfo() {
  lcd.clear();
  // Mostrar Velocidad desde la trama NMEA (vel_sat)
  lcd.setCursor(0, 0);
  lcd.print("Velocidad: ");
  if (gps.speed.isValid()) {
    lcd.setCursor(0, 2);
    vel_sat = gps.speed.kmph();  // Velocidad en m/s desde el GPS
    big.writeint(2,7,int(vel_sat),2,true);
    lcd.print("   km/h");
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Velocidad Invalida");  // Indica datos inválidos

    mqttClient.publish(topic6, ("Fallo displayInfo() en "+String(client_id)).c_str());

    vTaskDelay(1000 / portTICK_PERIOD_MS); //pauso el bucle un momento antes de intentar a leer de nuevo
  }
}

//Funcion para guardar los datos en un archivo CSV en la tarjeta SD
void saveDataToCSV() {
  // Intentar abrir el archivo hasta que sea exitoso
  while (true) {
    File file = SD.open(fileName, FILE_APPEND);
    if (file) {
      // Si el archivo se abre correctamente, continuar con el guardado de datos
      dataString = "";  // Limpiar dataString antes de agregar nuevos datos

      // Agregar el ID constante
      dataString += String(client_id) + ",";

      // Fecha y hora
      char buffer[50];
      sprintf(buffer, "%04d-%02d-%02d,%02d:%02d:%02d,", 
      gps.date.year(), gps.date.month(), gps.date.day(), 
      gps.time.hour(), gps.time.minute(), gps.time.second());
      dataString += String(buffer);
        
      // Latitud y longitud
      dataString += String(gps.location.lat(), 6) + ",";
      dataString += String(gps.location.lng(), 6) + ",";
        
      // Precisión (HDOP)
      dataString += String(gps.hdop.hdop()) + ",";
        
      // Velocidad calculada por satelite
      dataString += String(vel_sat,1) + ",";
        
      // Area_in_out (Área dentro o fuera)
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
        // Verificar si está dentro del área 1 (polígono)
        insideArea1 = isPointInPolygon(currentLat, currentLng, 
        area1_latitudes, area1_longitudes, numVerticesArea1);
        dataString += String(insideArea1);

      // Guardar la línea en el archivo CSV
      file.println(dataString); //escribo dataString en fileName
      file.close();  // Cerrar el archivo
      break;  // Salir del bucle si se guarda con éxito

    }
    else {
      // Mostrar mensaje de error en la LCD y reintentar después de un breve retraso
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error");
      lcd.setCursor(0, 1);
      lcd.print("guardando csv");

      mqttClient.publish(topic6, ("Fallo saveDataToCSV() en "+String(client_id)).c_str());

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

//Funcion para guardar los datos cuando el gps perdio señal (datos repetidos)
void saveDataToCSV_off() {
  // Intentar abrir el archivo hasta que sea exitoso
  while (true) {
    File file = SD.open(fileName, FILE_APPEND);
    if (file) {
      // Si el archivo se abre correctamente, continuar con el guardado de datos
      dataString = "";  // Limpiar dataString antes de agregar nuevos datos

      // Agregar el ID constante
      dataString += String(client_id) + ",";

      // Fecha y hora
      char buffer[50];
      sprintf(buffer, "%04d-%02d-%02d,%02d:%02d:%02d,", 
      gps.date.year(), gps.date.month(), gps.date.day(), 
      gps.time.hour(), gps.time.minute(), gps.time.second());
      dataString += String(buffer);
        
      // Latitud y longitud
      dataString += String(currentLat,6) + ",";
      dataString += String(currentLng,6) + ",";
        
      // Precisión (HDOP)
      dataString +="0,";
        
      // Velocidad calculada por satelite
      dataString += "0.0,";
        
      // Area_in_out (Área dentro o fuera)
      dataString += String(insideArea1);

      // Guardar la línea en el archivo CSV
      file.println(dataString); //escribo dataString en fileName
      file.close();  // Cerrar el archivo
      break;  // Salir del bucle si se guarda con éxito

    }
    else {
      // Mostrar mensaje de error en la LCD y reintentar después de un breve retraso
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error");
      lcd.setCursor(0, 1);
      lcd.print("guardando csv");

      mqttClient.publish(topic6, ("Fallo saveDataToCSV_off() en "+String(client_id)).c_str());

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

//Función para controlar la velocidad y activar el relé
void controlVel() {
  // Verificar si la velocidad es válida
  if (gps.speed.isValid() && gps.speed.isUpdated() && vel_sat < 30) {
    // Condicional basado en insideArea1
    if (insideArea1) {
      // Dentro del área: usa vel_in_max como límite
      if (vel_sat > vel_in_max) {
        digitalWrite(ledvel, HIGH); // Velocidad mayor a vel_in_max
        lcd.clear();
        lcd.setCursor(0, 0);
        big.writechar(0,0,'V');
        big.writechar(0,4,'E');
        big.writechar(0,7,'L');
        lcd.setCursor(0, 2);
        big.writechar(2,9,'M');
        big.writechar(2,14,'A');
        big.writechar(2,17,'X');

        mqttClient.publish(topic6, ("Se supero la velocidad maxima interna en "+String(client_id)).c_str());

      }
      else {
        digitalWrite(ledvel, LOW);  // Velocidad menor o igual a vel_in_max
      }
    }
    else {
      // Fuera del área: usa vel_out_max como límite
      if (vel_sat > vel_out_max) {
        digitalWrite(ledvel, HIGH); // Velocidad mayor a vel_out_max
        lcd.clear();
        lcd.setCursor(0, 0);
        big.writechar(0,0,'V');
        big.writechar(0,4,'E');
        big.writechar(0,7,'L');
        lcd.setCursor(0, 2);
        big.writechar(2,9,'M');
        big.writechar(2,14,'A');
        big.writechar(2,17,'X');

        mqttClient.publish(topic6, ("Se supero la velocidad maxima externa en "+String(client_id)).c_str());

      }
      else {
        digitalWrite(ledvel, LOW);  // Velocidad menor o igual a vel_out_max
      }
    }
  }
  else {
    // Si la velocidad no es válida, apagar el LED como medida de seguridad
    digitalWrite(ledvel, LOW);
  }
}

//Funcion para armar el jsonPayload a publicar en el topic1
void armedJson() {
  // Creamos el documento JSON en el heap
  StaticJsonDocument<256> jsonDoc;
  // Dividimos dataString en partes usando comas como delimitador
  int index = 0;
  String partes[8];  // Ajustamos el tamaño según la cantidad de datos
  for (int i = 0; i < dataString.length(); i++) {
    if (dataString[i] == ',') {
      index++;
    }
    else {
      partes[index] += dataString[i];
    }
  }
  // Asignamos cada parte al JSON
  jsonDoc["ID"] = partes[0];                  //ID
  jsonDoc["Fecha"] = partes[1];               // Fecha en formato "YYYY-MM-DD"
  jsonDoc["Hora"] = partes[2];                // Hora en formato "HH:MM:SS"
  jsonDoc["Latitud"] = partes[3].toFloat();   // Latitud en float
  jsonDoc["Longitud"] = partes[4].toFloat();  // Longitud en float
  jsonDoc["Precision"] = partes[5].toFloat();   // Precision en int
  jsonDoc["Velocidad"] = partes[6].toFloat();   // Velocidad en int
  jsonDoc["Area_in_out"] = partes[7].toInt(); // Convertimos de bool a int
  
  // Convertir el documento JSON a un string y almacenarlo en la variable local "jsonPayload"
  serializeJson(jsonDoc, jsonPayload);
}

//Funcion para reconfigurar las velocidades
void new_config(){
  File archivo = SD.open(velFile, FILE_WRITE); // Abrir archivo 
  if (archivo) {
    archivo.seek(0); // Posicionar al inicio del archivo
    archivo.print(String(new_vel_in_max) + "," + String(new_vel_out_max)); // Reescribir valores
    archivo.close(); // Cerrar archivo
  }
  else {

    mqttClient.publish(topic6, ("Fallo new_config()en "+String(client_id)).c_str());

  }
}

//Funcion para borrar todos los archivos de data en el rango de interes
void delete_SD() {
  File inputFile = SD.open(fileName, FILE_READ); // Abrir archivo original en modo lectura
  if (!inputFile) {

    mqttClient.publish(topic6, ("Fallo delete_SD():tipo 1 en "+String(client_id)).c_str());

    return;
  }
  File tempFile = SD.open(tempFileName, FILE_WRITE); // Crear archivo temporal
  if (!tempFile) {

    mqttClient.publish(topic6, ("Fallo delete_SD():tipo 2 en "+String(client_id)).c_str());

    inputFile.close();
    return;
  }

  // Leer línea por línea del archivo original
  while (inputFile.available()) {
    String line = inputFile.readStringUntil('\n'); // Leer una línea completa
    line.trim(); // Eliminar espacios o saltos de línea adicionales

    if (line.length() == 0 || line.startsWith("ID")) { 
      // Conservar encabezados y líneas vacías
      tempFile.println(line);
      continue;
    }

    // Dividir la línea en columnas
    int commaIndex1 = line.indexOf(',');
    int commaIndex2 = line.indexOf(',', commaIndex1 + 1); // Fecha
    int commaIndex3 = line.indexOf(',', commaIndex2 + 1); // Hora

    String date = line.substring(commaIndex1 + 1, commaIndex2); // Fecha
    String time = line.substring(commaIndex2 + 1, commaIndex3); // Hora

    // Condición para borrar la línea
    if ((date > d_start_date || (date == d_start_date && time >= d_start_time)) &&
        (date < d_end_date || (date == d_end_date && time <= d_end_time))) {
      // No escribir esta línea en el archivo temporal
      continue;
    }

    // Escribir la línea en el archivo temporal si no cumple la condición (se conservara)
    tempFile.println(line);
  }

  // Cerrar archivos
  inputFile.close();
  tempFile.close();

  // Reemplazar archivo original con el temporal
  if (SD.remove(fileName)) {
    SD.rename(tempFileName, fileName);

    mqttClient.publish(topic6, ("Datos borrados con exito en "+String(client_id)).c_str());

  } else {

    mqttClient.publish(topic6, ("Fallo delete_SD():tipo 3 en "+String(client_id)).c_str());

  }
  //Reseteamos los valores a los default
  d_start_date = "";
  d_start_time = "";
  d_end_date = "";
  d_end_time = "";
}

//Funcion que verifica si han pasado 8 dias
/*bool timeRead() {
  unsigned long eightDaysMillis = 8UL * 24 * 60 * 60 * 1000; // 8 días en milisegundos
  preferences.begin("time-tracker", false); // Inicia el espacio NVS
  unsigned long startTime = preferences.getULong("startTime", 0); // Recupera la marca de tiempo inicial
  unsigned long elapsedMillis = millis() - startTime; // Calcula el tiempo transcurrido

  if (elapsedMillis >= eightDaysMillis) {
    mqttClient.publish(topic6, "Han pasado 8 días, reiniciando SD");
    // Reinicia el temporizador actualizando `startTime` en NVS
    unsigned long currentMillis = millis();
    preferences.putULong("startTime", currentMillis); // Guarda el nuevo tiempo inicial
    preferences.end(); // Cierra el espacio NVS
    return true; // Indica que han pasado 8 días
  }

  return false; // Aún no han pasado 8 días
}*/

//Funcion para reconectarnos al broker sin interrumpir otras tareas
bool reconnectToMQTT() {
  if (mqttClient.connected()) {
    return true; // Ya estamos conectados
  }

  if (mqttClient.connect(client_id, mqtt_user, mqtt_password)) {
    
    mqttClient.subscribe(topic2);
    mqttClient.subscribe(topic3);
    mqttClient.subscribe(topic5);

    mqttClient.publish(topic6, (String(client_id)+" reconectado a internet").c_str());

    return true; // Conexión exitosa
  } else {
    return false; // Conexión fallida
  }
}

// Función para enviar datos en formato JSON a través de MQTT
void sendToMQTTJson() {
  // Paso 1: Intentar conectar a GPRS si no está conectado
  if (!modem.isGprsConnected()) {
    if (modem.gprsConnect(APN, USER, PWD)) {
    }
    else {
      return; // No continuar si no hay conexión GPRS
    }
  }

  // Paso 2: Verificar y reconectar a MQTT si es necesario
  if (!mqttClient.connected()) {
    mqttClient.setKeepAlive(10);
    
    if (mqttClient.connect(client_id, mqtt_user, mqtt_password, topiclwt, 0, true, lwt)) {
      // Suscribirse a los tópicos necesarios
      mqttClient.subscribe(topic2);
      mqttClient.subscribe(topic3);
      mqttClient.subscribe(topic5);

    }
    else {
      return; // No continuar si no hay conexión MQTT
    }
  }

  // Paso 3: Mantener la conexión MQTT activa
  mqttClient.loop();

  // Paso 4: Publicar el mensaje en el broker MQTT
  if(!mqttClient.publish(topic1, jsonPayload.c_str(), false)){
    saveFailure();
  }
}

//subfuncion que guarda un reporte de error
void saveFailure(){
  //Creo el mensaje de error
  StaticJsonDocument<256> report;
  DeserializationError error = deserializeJson(report, jsonPayload);
    
  int errorCode = mqttClient.state();
  String fecha = report["Fecha"].as<String>();
  String hora = report["Hora"].as<String>();
  String errorMessage = "Fallo tipo: (" + String(errorCode) + ") en " + client_id + " el " + fecha + " a las " + hora;

  //Guardo el mensaje
  File file = SD.open(report_fail, FILE_APPEND);
  if (file){
    file.println(errorMessage);  // Escribe el mensaje en una nueva línea
    file.close();
  }
}

void send_historic_data(){
  File file = SD.open(fileName); // Abrir el archivo

  String line = "";

  while (file.available()) {
    // Crear el documento JSON para almacenar el payload
    StaticJsonDocument<256> payloadObj;

    line = file.readStringUntil('\n'); // Leer línea por línea
    line.trim(); // Quitar espacios en blanco adicionales
    if (line.length() == 0 || line.startsWith("ID")) continue; // Ignorar encabezados y líneas vacías

    // Dividir la línea en columnas
    int commaIndex1 = line.indexOf(','); // ID
    int commaIndex2 = line.indexOf(',', commaIndex1 + 1); // Fecha
    int commaIndex3 = line.indexOf(',', commaIndex2 + 1); // Hora
    int commaIndex4 = line.indexOf(',', commaIndex3 + 1); // Latitud
    int commaIndex5 = line.indexOf(',', commaIndex4 + 1); // Longitud
    int commaIndex6 = line.indexOf(',', commaIndex5 + 1); // Precisión
    int commaIndex7 = line.indexOf(',', commaIndex6 + 1); // Velocidad

    String id = line.substring(0, commaIndex1); // ID
    String date = line.substring(commaIndex1 + 1, commaIndex2); // Fecha
    String time = line.substring(commaIndex2 + 1, commaIndex3); // Hora
    float lat = line.substring(commaIndex3 + 1, commaIndex4).toFloat(); // Latitud
    float lon = line.substring(commaIndex4 + 1, commaIndex5).toFloat(); // Longitud
    float precision = line.substring(commaIndex5 + 1, commaIndex6).toFloat(); // Precisión
    float velocidad = line.substring(commaIndex6 + 1, commaIndex7).toFloat(); // Velocidad
    int area_in_out = line.substring(commaIndex7 + 1).toInt(); // Área_in_out

    // Verificar si está dentro del rango de fecha y hora
    if ((date > start_date || (date == start_date && time >= start_time)) &&
      (date < end_date || (date == end_date && time <= end_time))) {
      // Crear un objeto JSON para esta fila
      payloadObj["ID"] = id;
      payloadObj["Fecha"] = date;
      payloadObj["Hora"] = time;
      payloadObj["Latitud"] = lat;
      payloadObj["Longitud"] = lon;
      payloadObj["Precision"] = precision;
      payloadObj["Velocidad"] = velocidad;
      payloadObj["Area_in_out"] = area_in_out;
    }
    // Serializar y enviar/guardar el bloque
    String output;
    if (payloadObj.size()>0){
    serializeJson(payloadObj, output);
    // Publicar el mensaje en el broker MQTT
    mqttClient.publish(topic4, output.c_str());
    }
  }
  file.close(); // Cerrar el archivo

  mqttClient.publish(topic6, ("Datos historicos enviados desde "+String(client_id)).c_str());

  //Regresamos los valores por default
  start_date = "";
  start_time = "";
  end_date = "";
  end_time = "";
}