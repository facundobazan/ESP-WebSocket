#include <DHT.h>
//#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <LiquidCrystal_I2C.h>
#define puertoDNS 53
#define puertoHTTP 80
#define puertoWebSocket 1337
#define LED_BLINK 2
#define DHTPIN 27
#define switchMode 25
#define Rele1 33
#define Rele2 32
#define Rele3 18
#define Rele4 19
#define Touch T0
#define DHTTYPE DHT11

//fauxmoESP Alexa;

bool estBlink;
bool estRele1;
bool estRele2;
bool estRele3;
bool estRele4;
bool modoAP = false;
bool lcdON = true;
unsigned long tiempo;
unsigned long checkpoint = 0;
unsigned long checkpoint2 = 0;
int bandera = 0;
String temperatura;
String humedad;
/*String nombreRele1 = "tx1Garage";
String nombreRele2 = "tx2Living";
String nombreRele3 = "tx3Living #2";
String nombreRele4 = "tx4Comedor";*/

//CONFIGURACION WIFI
char *red = "iMocho";
char *clave =  "Gamuza Bazan";

//String WIFI_SSID = red;
//String WIFI_PASS = clave;

char *redAP = "iMochoAP";
char *claveAP =  "Pomelo Rosado";

const char *msg_toggle_led = "toggleLED";
const char *msg_get_led = "getLEDState";

//CONFIGURACION WEBSOCKET/ASYNCWEBSERVER
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);

//CONFIGURACION LCD 16X2
//LCD SDA 21
//LCD SCL 22
LiquidCrystal_I2C lcd(0x27, 16, 2);

char msg_buf[10];
char pin[1] = "";
String mensajeLeido;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  pinMode(LED_BLINK, OUTPUT);
  digitalWrite(LED_BLINK, LOW);

  pinMode(switchMode, INPUT); // presiono = high
  attachInterrupt (switchMode, isrSwitchMode, FALLING);
  modoAP = digitalRead(switchMode);

  pinMode(Rele1, OUTPUT);
  digitalWrite(Rele1, HIGH);
  pinMode(Rele2, OUTPUT);
  digitalWrite(Rele2, HIGH);
  pinMode(Rele3, OUTPUT);
  digitalWrite(Rele3, HIGH);
  pinMode(Rele4, OUTPUT);
  digitalWrite(Rele4, HIGH);

  //PULLDOWN PUSH -> HIGH
  pinMode(Touch, INPUT_PULLDOWN);
  //TOUCH SENSIBILIDAD 40
  attachInterrupt (Touch, apagarLCD, 40);

  //Serial.begin(115200);
  if ( !SPIFFS.begin()) {
    //Serial.println("ERROR AL MONTAR SPIFFS");
    while (1);
  }

  lcd.init();
  lcd.backlight();
  lcd.clear();
  conectarWifi();
  server.begin();



  //INICIAR COMO AP
  //WiFi.softAP(red, clave);
  //mostrarIP();
  //delay(5000);

  server.on("/", HTTP_GET, enviarIndex);
  server.on("/estilo.css", HTTP_GET, enviarCSS);
  server.on("/aplicacion.js", HTTP_GET, enviarJS);
  server.on("/favicon.png", HTTP_GET, enviarPNG);
  server.on("/temperatura.png", HTTP_GET, enviarTemperatura);
  server.on("/humedad.png", HTTP_GET, enviarHumedad);

  //MANEJAR REQUEST DE PAGINAS INEXISTENTES
  server.onNotFound(enviarError404);
  //INICIA WEBSERVER
  server.begin();
  //INICIA WEBSOCKET Y ASIGNAR EL CALLBACK
  webSocket.begin();
  webSocket.onEvent(eventoWebSocket);

  dht.begin();

  leerDHT();

  estBlink = digitalRead(LED_BLINK);
  estRele1 = digitalRead(Rele1);
  estRele2 = digitalRead(Rele2);
  estRele3 = digitalRead(Rele3);
  estRele4 = digitalRead(Rele4);

  /*Alexa.createServer(true);
  Alexa.setPort(80);
  Alexa.enable(true);
  Alexa.addDevice("Garage");
  Alexa.addDevice("Living");
  Alexa.addDevice("Living 2");
  Alexa.addDevice("Comedor");
  Alexa.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {

    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

    if (strcmp(device_name, "Garage") == 0) {
      digitalWrite(Rele1, state ? HIGH : LOW);
    } else if (strcmp(device_name, "Living") == 0) {
      digitalWrite(Rele2, state ? HIGH : LOW);
    } else if (strcmp(device_name, "Living 2") == 0) {
      digitalWrite(Rele3, state ? HIGH : LOW);
    } else if (strcmp(device_name, "Comedor") == 0) {
      digitalWrite(Rele4, state ? HIGH : LOW);
    }

  });*/
}

void loop() {
  tiempo = millis();
  if (tiempo - checkpoint >= 3000 && lcdON) actualizarLCD();
  if (tiempo - checkpoint2 >= 60000) leerDHT();
  //Alexa.handle();
  webSocket.loop();
}

void eventoWebSocket(uint8_t numeroCliente, WStype_t type, uint8_t * payload, size_t length)
{
  switch (type)
  {
    //CLIENTE DESCONECTADO
    case WStype_DISCONNECTED:
      //Serial.printf("[%u] DESCONECTADO!\n", numeroCliente);
      break;

    //CLIENTE CONECTADO
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(numeroCliente);
        //Serial.printf("[%u] CONEXION DESDE ", numeroCliente);
        //Serial.println(ip.toString());
      }
      break;

    //MANEJAR MENSAJES DEL CLIENTE
    case WStype_TEXT:

      //MUESTRA EL MENSAJE SIN FORMATO
      //Serial.printf("[%u] TEXTO RECIBIDO: %s\n", numeroCliente, payload);
      if ( strcmp((char *)payload, "blink") == 0 )
      {
        //CAMBIA ESTADO DEL LED
        digitalWrite(LED_BLINK, !estBlink);
        estBlink = !estBlink;
      }
      else if ( strcmp((char *)payload, "est_blink") == 0 )
      {
        //REPORTA A TODOS EL ESTADO DEL LED
        //pin[0] = '0' + estBlink;
        //sprintf(msg_buf, "%d", estBlink);
        sprintf(msg_buf, "%i%i", 0, estBlink);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "btn1") == 0 )
      {
        digitalWrite(Rele1, !estRele1);
        estRele1 = !estRele1;
      }
      else if ( strcmp((char *)payload, "est_btn1") == 0 )
      {
        sprintf(msg_buf, "%i%i", 1, !estRele1);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "btn2") == 0 )
      {
        digitalWrite(Rele2, !estRele2);
        estRele2 = !estRele2;
      }
      else if ( strcmp((char *)payload, "est_btn2") == 0 )
      {
        sprintf(msg_buf, "%i%i", 2, !estRele2);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "btn3") == 0 )
      {
        //CAMBIA ESTADO DEL LED
        digitalWrite(Rele3, !estRele3);
        estRele3 = !estRele3;
      }
      else if ( strcmp((char *)payload, "est_btn3") == 0 )
      {
        sprintf(msg_buf, "%i%i", 3, !estRele3);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "btn4") == 0 )
      {
        digitalWrite(Rele4, !estRele4);
        estRele4 = !estRele4;
      }
      else if ( strcmp((char *)payload, "est_btn4") == 0 )
      {
        sprintf(msg_buf, "%i%i", 4, !estRele4);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "est_temperatura") == 0 )
      {
        sprintf(msg_buf, "%s", temperatura);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "est_humedad") == 0 )
      {
        sprintf(msg_buf, "%s", humedad);
        webSocket.broadcastTXT(msg_buf);
      }/*

      if ( strcmp((char *)payload, "ntx1") == 0 )
      {
        sprintf(msg_buf, "%s", nombreRele1);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "ntx2") == 0 )
      {
        sprintf(msg_buf, "%s", nombreRele2);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "ntx3") == 0 )
      {
        sprintf(msg_buf, "%s", nombreRele3);
        webSocket.broadcastTXT(msg_buf);
      }

      if ( strcmp((char *)payload, "ntx4") == 0 )
      {
        sprintf(msg_buf, "%s", nombreRele4);
        webSocket.broadcastTXT(msg_buf);
      }*/
      else
      {
        //Serial.println("[%u] MENSAJE NO RECONOCIDO");
      }

      break;

    //CASOS NO IMPLEMENTADOS
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void enviarIndex(AsyncWebServerRequest *request)
{
  //IPAddress remote_ip = request->client()->remoteIP();
  request->send(SPIFFS, "/index.html", "text/html");
}

void enviarCSS(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/estilo.css", "text/css");
}

void enviarJS(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/aplicacion.js", "text/javascript");
}

void enviarPNG(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/favicon.png", "image/png");
}

void enviarTemperatura(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/temperatura.png", "image/png");
}

void enviarHumedad(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/humedad.png", "image/png");
}

void enviarError404(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Pagina no encontrada");
}

void conectarWifi()
{
  mensajeLCD("", "");
  //lcd.clear();
  //lcd.setCursor(0, 0);
  if (modoAP)
  {
    mensajeLCD("INICIANDO AP...", redAP);
    /*lcd.print("INICIANDO AP...");
      lcd.setCursor(0, 1);
      lcd.print(redAP);*/
    WiFi.softAP(redAP, claveAP);

    while (WiFi.status() != WL_CONNECTED)
    {
      for (int i = 14; i <= 16; i++ )
      {
        lcd.setCursor(i, 1);
        lcd.print(".");
        delay(250);
      }
      delay(300);
      lcd.setCursor(14, 1);
      lcd.print("   ");
    }
    lcd.clear();
    /*lcd.setCursor(0, 0);
      lcd.print(redAP);
      lcd.setCursor(0, 1);
      lcd.print(claveAP);*/
  }
  else
  {
    mensajeLCD("CONECTANDO A", red);
    /*lcd.print("CONECTANDO A");
      lcd.setCursor(0, 1);
      lcd.print(red);
    */
    WiFi.begin(red, clave);

    while (WiFi.status() != WL_CONNECTED)
    {
      for (int i = 14; i <= 16; i++ )
      {
        lcd.setCursor(i, 1);
        lcd.print(".");
        delay(250);
      }
      delay(300);
      lcd.setCursor(14, 1);
      lcd.print("   ");
    }
  }
}

/*void mostrarRed()
  {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CONECTADO A RED");
  lcd.setCursor(0, 1);
  lcd.print(red);
  }*/

/*void mostrarIP()
  {
  mensajeLCD("CONECTADO", WiFi.localIP());
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CONECTADO");
  lcd.setCursor(0, 1);
  lcd.print("IP ");
  lcd.print(WiFi.localIP());
  }*/

void isrSwitchMode()
{
  mensajeLCD("REINCIANDO...", "");
  /*lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("REINCIANDO...");*/
  delay(3000);
  ESP.restart();
}

void apagarLCD()
{
  lcdON = !lcdON;
  //mensajeLCD("     TOUCH     ", "   PRESIONADO   ");
  lcd.setBacklight(lcdON);
}

/*void leerArchivo(archivo)
  {
  if (!SPIFFS.begin(true))
  {
    //Serial.println("ERROR AL INICIAR SPIFFS");
    return;
  }

  File file = SPIFFS.open(archivo);

  if (!file)
  {
    //Serial.println("ERROR AL ABRIR ARCHIVO");
    return;
  }

  while (file.available())
  {
    mensajeLeido = file.read();
    //Serial.write(mensajeLeido);
  }

  file.close();
  delay(1000);
  return mensajeLeido;
  }

  void escribirArchivo(archivo, mensaje)
  {
  if (!SPIFFS.begin(true))
  {
    //Serial.println("ERROR AL INICIAR SPIFFS");
    return;
  }

  File file = SPIFFS.open(archivo, FILE_WRITE);

  if (!file)
  {
    //Serial.println("ERROR AL ABRIR ARCHIVO");
    return;
  }

  if (file.print(mensaje))
  {
    //Serial.println("ARCHIVO GUARDADO");
  }
  else
  {
    //Serial.println("ERROR AL GUARDAR ARCHIVO");
  }

  file.close();
  }*/

void actualizarLCD()
{
  checkpoint = tiempo;

  if (bandera == 0 )
  {
    if (modoAP)
    {
      mensajeLCD("(MODO AP) RED:", redAP);
    }
    else
    {
      mensajeLCD("CONECTADO A RED:", red);
    }
    bandera = 1;
  }
  else
  {
    if (bandera == 1 && modoAP)
    {
      mensajeLCD("CLAVE WIFI:", claveAP);
      bandera = 2;
    }
    else
    {
      mensajeLCD("DIRECCION IP:", "");
      lcd.print(WiFi.localIP());
      bandera = 0;
    }
  }
}

void mensajeLCD(String linea1, String linea2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(linea1);
  lcd.setCursor(0, 1);
  lcd.print(linea2);
}

void leerDHT()
{
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t))
  {
    temperatura = "t--";
  }
  else
  {
    temperatura = "t" + (String)t;
  }
  if (isnan(h))
  {
    humedad = "h--";
  }
  else
  {
    humedad = "h" + (String)h;
  }
  checkpoint2 = tiempo;

  sprintf(msg_buf, "%s", temperatura);
  webSocket.broadcastTXT(msg_buf);
  sprintf(msg_buf, "%s", humedad);
  webSocket.broadcastTXT(msg_buf);
}
