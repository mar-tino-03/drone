#ifndef _WIFILUCA_H_
#define _WIFILUCA_H_

const char* ssid = "Pfizer";//FASTWEB-BPDB15-Plus
const char* password = "ciaociao";//UYJRHSG365

JSONVar parsed;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void initWiFi();
void initFS();
void initWebSocket();
void hostSite();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void inviaDatiUtenti(JSONVar parsed);

void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void readFile(fs::FS &fs, const char * path);

//_________________________________________________________________wifi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  
  Serial.println(WiFi.localIP());
}

//_________________________________________________________________file sistem
void initFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}

//_________________________________________________________________webSocket
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

//________________________________________________________________host
void hostSite(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  //server.on("/debug.txt", HTTP_GET, [](AsyncWebServerRequest *request) {
    //request->send(LittleFS, "/debug.txt", "text/plain");
  //});

  server.serveStatic("/", LittleFS, "/");

  // Start server
  server.begin();
}

//========== WebSocket Event Handler ==========
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        String message = String((char*)data).substring(0, len);

        parsed = JSON.parse(message);

        // Validate JSON parsing
        if (JSON.typeof(parsed) == "undefined") {
          Serial.println("JSON parsing failed!");
          return;
        }

        // Check and update joystick data
        if (parsed.hasOwnProperty("j1X")) {
          yaw_dot_input_desired_angle = - String((const char*)parsed["j1X"]).toFloat();
        }
        if(parsed.hasOwnProperty("j1Y")){
          throttle_desired = (String((const char*)parsed["j1Y"]).toFloat()) * 4;
        }
        if (parsed.hasOwnProperty("j2X")) {
          roll_desired_angle = String((const char*)parsed["j2X"]).toFloat();  //+-10
        }
        if(parsed.hasOwnProperty("j2Y")){
          pitch_desired_angle = String((const char*)parsed["j2Y"]).toFloat();  //+-10
        }

        if(parsed.hasOwnProperty("input1")){
          Serial.println(parsed["input1"]);
          yaw_kp = String((const char*)parsed["input1"]).toFloat();
        }else if(parsed.hasOwnProperty("input2")){
          Serial.println(parsed["input2"]);
          yaw_dot_kp = String((const char*)parsed["input2"]).toFloat();
        }else if(parsed.hasOwnProperty("input3")){
          Serial.println(parsed["input3"]);
          yaw_kp = String((const char*)parsed["input3"]).toFloat();
        }
        if(parsed.hasOwnProperty("blockAlt")){
          modalita = (bool)parsed["blockAlt"];
        }
        if(parsed.hasOwnProperty("clearFile")){
          writeFile(LittleFS, "/debug.txt", "");
        }
        if(parsed.hasOwnProperty("writeFile")){
          writeInRam = (bool)parsed["writeFile"];
          startTime = millis();
          if(writeInRam == false) writeInFile = true;
        }

        //Print joystick values for debugging
        //Serial.printf("Joystick1: (X: ?, Y: %.2f), Joystick2: (X: %.2f, Y: %.2f)\n",
        //              input_THROTTLE, roll_desired_angle, pitch_desired_angle);
        Serial.println("Joystick1: (X: ?, Y: "+String(throttle_desired)+"), Joystick2: (X: "+String(roll_desired_angle)+", Y: "+String(pitch_desired_angle)+")");
        
        //inviaDatiUtenti();//invio dati all'utente
      }
      break;
    }
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      throttle_desired = 0;
      roll_desired_angle = 0;
      pitch_desired_angle = 0;
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void inviaDatiUtenti(JSONVar parsed){
  String jsonString = JSON.stringify(parsed);
  //Serial.print(jsonString+"\n");
  //jsonString = "{";
  
  ws.textAll(jsonString);
  //ws.binaryAll((uint8_t*)jsonString.c_str(), jsonString.length());
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

#endif