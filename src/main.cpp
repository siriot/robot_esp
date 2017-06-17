
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include "WiFiServer.h"
#include <ESP8266WiFiScan.h>
#include <FS.h>
#include <ESP.h>
#include "ArduinoJson.h"
#include <ESP8266HTTPUpdateServer.h>


// UART_ COMMANDS
#define COMMUNICATION_RESET 0x00
#define SET_SPEED		0x10
#define START_RADAR		0x30
#define WRITE_SONAR_DATA 0x20

// Logging function
#define Log(x) Log_2_Buffer(x,100)
// Debug messages
#define db_msg(x) Serial1.println(x)

// Data structures
const char *SSID = "TP-LINK_C9FBE8";
const char *PASSW = "";

const char* NotFoundHTML="<html> <title> Not found </title> \
<body> \
The requested site doesn't exists.</body></html>'";

// UART data
#define UART_RX_BUFFER_SIZE 2+255
unsigned char UART_rx_buffer[UART_RX_BUFFER_SIZE] __attribute__ ((aligned (16)));
#define UART_TX_BUFFER_SIZE 2+255
unsigned char UART_tx_buffer[UART_TX_BUFFER_SIZE];
unsigned char *rx_buffer_write;
#define RX_STATE_HEAD 0
#define RX_STATE_DATA 1
char rx_state;
int bytes_left;

#define SERVER_ANS_BUFFER_SIZE 2048
#define LOG_BUFFER_SIZE 2048  // max ser.ans.buff.size


// TX buffer for webserver answer
unsigned char SERVER_ANS_BUFFER[SERVER_ANS_BUFFER_SIZE+1];

// Sonar data in ASCII
char SONAR_data[1024]="100,110,120,130,140,130,120,110,100,150,130,110";

// Log buffer
unsigned char Log_Buffer[LOG_BUFFER_SIZE];
unsigned int Log_Buffer_Index = 0;

ESP8266WebServer server(80);

// OTA updater
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";
ESP8266HTTPUpdateServer httpUpdater;

// Socket
WiFiServer server_socket(2000);
WiFiClient client_socket;

// Side handlers
void handleRoot();
void handleFSread();
void handleUI();
void handleCommand();
void handleFSupload();
void handleOK();
void handleFSdelete();
void handleInfo(); // old version not used
void handleSonarDataRequest();
void redirect();

void manageKeys();
void getKeyJson();
void getInfoJson();
void getNetworksJson();
void saveKeyJson();
void getLog();

void handleReset();
void handleReconnect();

void ProcessData(uint8_t *buffer);
IPAddress initWifi();

void Log_2_Buffer(const String& msg,unsigned int max_len);
void Log_2_Buffer(const char* msg, unsigned int len);

/********************************************** MAIN ************************************************/
ADC_MODE(ADC_VCC);
IPAddress my_ip;
bool isAP = false;
void setup()
{
  // Swtich ADC to VCC, used in info site
  // Starting the serial port. This step supresses the debug messages of the ESP core.
  Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1); // Enable serial1 on GPIO2 pin for log messages.

  SPIFFS.begin();
  initWifi();
  // Setup server
  server.onNotFound(handleFSread);
  server.on("/UI", handleUI);
  server.on("/",handleRoot);
  server.on("/command",handleCommand);
  server.on("/fs_upload",HTTP_POST, redirect,handleFSupload);
  server.on("/fs_delete",HTTP_ANY,handleFSdelete);
  server.on("/info",HTTP_GET,handleInfo);
  server.on("/sonar_data.txt",HTTP_GET,handleSonarDataRequest);
// JSON methods
  server.on("/getInfoJson",HTTP_GET,getInfoJson);
// WiFi manager with JSON
  server.on("/manageKeys",manageKeys);
  server.on("/getKeyJson",HTTP_GET,getKeyJson);
  server.on("/getNetworksJson",HTTP_GET,getNetworksJson);
  server.on("/saveKeyJson",HTTP_POST,saveKeyJson);
  server.on("/Log",HTTP_GET,getLog);

  server.on("/reset_ic",HTTP_GET,handleReset);
  server.on("/reconnect",HTTP_GET,handleReconnect);

//  server.on("/radarData",handleRadarData);
  /*
 httpUpdater.setup(&server, update_path, update_username, update_password);
 Log("OTA updater initialized!");
 Log("Update username:");
 Log(update_username);
 Log("passwd:");
 Log(update_password);
 Log("");*/
server.begin();
server_socket.begin();

rx_state = RX_STATE_HEAD;
bytes_left = 2;
rx_buffer_write = UART_rx_buffer;
}


void loop()
{
	if(!isAP && (WiFi.status()!=WL_CONNECTED))
	{
		Log("Connection lost, rescanning.");
		initWifi();
	}

  if(Serial.available())
  {
    db_msg("Bytes received:"+String(Serial.available()));
    uint8_t byte;
    Serial.readBytes(&byte,1);
	if(client_socket.connected())
    {
    	//send the packet to the client
    	client_socket.write((uint8_t*)&byte,1);
    }
      switch(rx_state)
      {
        case RX_STATE_HEAD:
            *rx_buffer_write++ = byte;
            bytes_left--;
            if(bytes_left == 0)
            {
              // Check for command code, if invalid, restart the flush the buffer ad restart the receiving.
              if(UART_rx_buffer[0] == COMMUNICATION_RESET)
              {
                Log("CR");
                rx_buffer_write = UART_rx_buffer;
                bytes_left = 2;
                break;
              }
              // Accept data bytes
              else if(UART_rx_buffer[1]==0)
              {
                // No data bytes, ready for processing
                ProcessData(UART_rx_buffer);
                rx_buffer_write = UART_rx_buffer;
                bytes_left = 2;
              }
              else
              {
                // Start data receiving
                rx_state = RX_STATE_DATA;
                bytes_left = UART_rx_buffer[1];
                db_msg("Start receiving data section. Bytes left: " + String(bytes_left)) ;
              }
            }
            break;
       case RX_STATE_DATA:
             *rx_buffer_write++ = byte;
             bytes_left--;
             if(bytes_left == 0)
             {
               db_msg("Data section received");
               ProcessData(UART_rx_buffer);
               rx_state = RX_STATE_HEAD;
               bytes_left = 2;
               rx_buffer_write = UART_rx_buffer;
             }
             break;
      }
      return;
  }

  if(!client_socket.connected())
  {
	  //try to get new client
	  client_socket = server_socket.available();
	  if(client_socket.connected())
	  {
		  Log("Client connected to the socket.");
	  	  Log("Remote IP Address:"+client_socket.remoteIP().toString());
	  	  Log("Remote port:"+String(client_socket.remotePort()));
	  	  Log("Local port:"+String(client_socket.localPort()));
	  }


  }
  else if(client_socket.available())
  {
	  // send the incoming bytes
	  unsigned char buffer[256];
	  int n;
	  do
	  {
		  n = client_socket.read(buffer,256);
		  if (n>0) Serial.write(buffer,n);
	  }while(n>0);

  }

  server.handleClient();
}

/**************************************** Connecter ************************************/
IPAddress initWifi()
{
  // Reset all current connections
  Log("Resetting connections.");
  WiFi.disconnect();
  WiFi.softAPdisconnect(true);
  // Scan networks
  int found_networks = WiFi.scanNetworks();
  // /networks file contains SSID-password pairs. If exists, the ESP tries to connect one of them.
  if(SPIFFS.exists("wifi_keys.json"))
  {
    Log("wifi_keys.json found.");
    File f = SPIFFS.open("wifi_keys.json", "r");
    StaticJsonBuffer<1024> jsonBuffer;
    char buffer[1024];
    int buf_size;

    buf_size = f.read((uint8_t*)buffer,1024);
    f.close();
    JsonArray& root = jsonBuffer.parseArray(buffer,3);

    for(int i=0; i< root.size() ;i++)
    {
      String ssid = root[i]["SSID"];
      String passwd  = root[i]["PASSWD"];
      db_msg("SSID list element:");
      db_msg(String(ssid));
      for(int j=0;j<found_networks;j++)
        if(ssid == WiFi.SSID(j))
        {
          WiFi.begin(ssid.c_str(),passwd.c_str());
          WiFi.waitForConnectResult();
          if(WiFi.status() == WL_CONNECTED)
          {
            Log("Connected to Wifi hotspot. SSID: "+ssid);
            Log("IP address: "+WiFi.localIP().toString());
            isAP = false;
            return WiFi.localIP();
          }
        }
     }
   } else Log("wifi_keys.json not foud.");

  // No known access point, create a new one.
  WiFi.softAP("ESP_SOFT_AP","blueheaven",7);
  Log("Own hotspot created.");
  Log("IP adress:"+WiFi.softAPIP().toString());
  isAP = true;
  return WiFi.softAPIP();
}
/**************************************** WEB_HANDLERS ***********************************/
void handleNotFound()
{
  server.send(200,"text/html",NotFoundHTML);
}

void handleRoot()
{
  if(SPIFFS.exists("index.html"))
  {
    File f = SPIFFS.open("index.html","r");
    server.streamFile(f,"text/html");
    f.close();
  }
  else server.send(404,"text/plain","index.html file not found.");
}


void handleUI()
{
  if(SPIFFS.exists("robot_ui.html"))
  {
    File f = SPIFFS.open("robot_ui.html","r");
    server.streamFile(f,"text/html");
    f.close();
  }
  else server.send(404,"text/plain","robot_ui.html file not found.");
}
void handleCommand()
{
  uint8_t i;

  if(server.args() <= 0) return;

  UART_tx_buffer[0] = (int8_t)(server.arg(0).toInt());
  UART_tx_buffer[1] = server.args()-1;

  for(i=1;i<server.args();i++)
  {
    int8_t val = (int8_t)(server.arg(i).toInt());
    UART_tx_buffer[1+i] = val;
  }
  Serial.write(UART_tx_buffer, server.args()+1);
  server.send(204,"text/plain","");
}

void handleFSdelete() {
  if(server.hasArg("delete"))
  {
    // Do Deleting
    String filename = server.arg("delete");
    Log("Deleting file: "+filename);
    if(SPIFFS.exists(filename))
    {
      SPIFFS.remove(filename);
    }
    else
    {
      return server.send(404,"text/plain","File not found");
    }
  }
  // In case of success send the info page back
  redirect();
}

File uploadingFile;
void handleFSupload()
{
  Log("Upload process started.");
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    Log("New file name: "+filename);
    if(SPIFFS.exists(filename)) SPIFFS.remove(filename);
    uploadingFile = SPIFFS.open(filename, "w");
    //filename=String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(uploadingFile)
      uploadingFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(uploadingFile)
      uploadingFile.close();
  }
}

void handleFSread()
{
  String path=server.uri();
  if(!path.startsWith("/fs/")) return server.send(404,"text/plain","File not found");
  String filename = path.substring(4);
  Log("Looking up file: "+filename);
  if(SPIFFS.exists(filename))
  {
    // Get content type from file name
    String contentType;
    if(filename.endsWith("html") || filename.endsWith("htm") )contentType = "text/html";
    else contentType = "text/plain";
    // Open and send file
    File f = SPIFFS.open(filename,"r");
    server.streamFile(f, contentType);
    f.close();
  }
  else
  {
    server.send(404,"text/plain","File not found");
  }
}

void handleOK()
{
  server.send(200,"text/plain","OK");
}

void redirect()
{
  String page="  \
  <html> \
  <head> \
    <title>Success</title>  \
    <meta http-equiv='refresh' content=\"0;URL='http://" + WiFi.localIP().toString() + "/info"+  "'\" /> \
  </head> \
  <body> \
    <p> Operation succeeded. Redirecting to the info site.</p> \
  </body> \
</html>" ;

server.send(200,"text/html",page);

}

void handleInfo()
{
  FSInfo info;
  SPIFFS.info(info);
  String buffer;
  buffer+="<!DOCTYPE html><meta charset='utf-8'><html><head><title>ESP file system info</title></head><body> Info:<br>";
  buffer+="File system full size: " + String(info.totalBytes) +"Bytes<br>";
  buffer+="File system used size: " + String(info.usedBytes) + "Bytes<br>";
  buffer+= "<br>File list (can be read under /fs/<filename>):<br>";
  Dir d = SPIFFS.openDir("");
  while (d.next()) {
    String fileName = d.fileName();
    size_t fileSize = d.fileSize();
    buffer+= "* " + fileName + "       " + String(fileSize) + "Bytes   <a href=/fs_delete?delete="+fileName+">Delete</a><br>";
  }
  // File uploader
  buffer+="<br>Upload new file: <br>";
  buffer+="<form method='POST' action='/fs_upload' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

  // System info
  buffer+="<br>System info:<br>";
  buffer+="Free Heap: " + String(ESP.getFreeHeap()) +" Bytes<br>";
  buffer+="Chip ID: " + String(ESP.getChipId()) +" <br>";
  buffer+="Flash size: " + String(ESP.getFlashChipSize()) +" Bytes<br>";
  buffer+="Supply voltage: " + String(ESP.getVcc()) +" mV<br>";
  buffer+= "</body></html>";
  server.send(200,"text/html",buffer);
}

void handleSonarDataRequest()
{
  server.send(200,"text/plain", SONAR_data);
}

uint8_t buffer_1k[1024];
int buffer_1k_len = 0;
void getKeyJson()
{
  // Open wifi_keys.json file
  if(!SPIFFS.exists("wifi_keys.json")) { server.send(404,"text/plain","File: wifi_keys.json not found"); return;}
  File f = SPIFFS.open("wifi_keys.json", "r");
  memset(buffer_1k,0,buffer_1k_len);
  buffer_1k_len = f.read(buffer_1k,1024);
  server.send(200,"application/json",String((char*)buffer_1k));
  f.close();
}

void saveKeyJson()
{
	if(!server.hasArg("json"))
	{
		server.send(404,"text/plain","No parameters given.");
		return;
	}
	String json_text = server.arg("json");
	File f = SPIFFS.open("wifi_keys.json","w");
	f.write((const unsigned char*)json_text.c_str(),json_text.length());
	f.close();
	server.send(200,"text/plain","OK");

}
void manageKeys()
{
  StaticJsonBuffer<1024> jsonBuffer;
  if(!SPIFFS.exists("wifi_keys.json")) { server.send(404,"text/plain","File: wifi_keys.json.json not found"); return;}
  File f = SPIFFS.open("wifi_keys.json", "r");
  memset(buffer_1k,0,buffer_1k_len);
  buffer_1k_len = f.read(buffer_1k,1024);
  f.close();
  JsonArray& root = jsonBuffer.parseArray((char*)buffer_1k,3);

  if(server.hasArg("delete"))
  {
    // Delete the SSID-PASSWD pair with the given SSID
    db_msg("Delete command received. Parsing list.");

    char removed = 0;
    for(int i=0; i< root.size() && !removed ;i++)
    {
      const char *ssid = root[i]["SSID"];
      db_msg("SSID list element:");
      db_msg(String(ssid));
      if(!strcmp(ssid,server.arg("delete").c_str()))
        {
          db_msg("SSID found and deleted.");
          root.removeAt(i);
          server.send(200,"text/plain","ok");
          removed = 1;
          break;
        }
    }
    if(!removed) server.send(404,"text/plain","Given SSID not found");
  }
  else if(server.hasArg("new_SSID") && server.hasArg("new_password"))
  {
    db_msg("Adding new key pair:");
    // Add new SSID-PASSWD key pair
    // ? server.arg változhat a kiírás előtt?, és mi pointerrel mutatunk rá.
    JsonObject& pair = root.createNestedObject();
    pair["PASSWD"] = server.arg("new_password");
    pair["SSID"] = server.arg("new_SSID");
    server.send(200,"text/plain","ok");
    db_msg(server.arg("new_SSID"));
    db_msg(server.arg("new_password"));
  } else server.send(404,"text/plain","No command arguments");

  //buffer_1k_len = root.printTo((char*)buffer_1k,1024);
  File fout = SPIFFS.open("wifi_keys.json","w");
  //f.write(buffer_1k,buffer_1k_len);
  root.printTo(fout);
  fout.close();
}

void getNetworksJson()
{
	String json_text;

	json_text += "{";
  // Actual WiFi networks:
	json_text+= "\"networks\":[";

	int found_networks = WiFi.scanNetworks();

	String g;
	for(int i=0;i<found_networks;i++)
	{
	  if(i!=0) json_text+=",";
	  json_text+="{";
	  json_text+= "\"SSID\":\"" + WiFi.SSID(i)+ "\",";
	  json_text+= "\"RSSI\":\"" + String(WiFi.RSSI(i)) + "dB\",";
	  json_text+= "\"CHANNEL\":\"" + String(WiFi.channel(i)) + "\",";
	  json_text+="\"ENC_TYPE\":\"";
	  switch(WiFi.encryptionType(i))
	  {
	  case ENC_TYPE_WEP: json_text+= String("WEP"); break;
	  case ENC_TYPE_TKIP: json_text+= String("WPA/PSK"); break;
	  case ENC_TYPE_CCMP:  json_text+= String("WPA2/PSK"); break;
	  case ENC_TYPE_NONE: json_text+= String("OPEN"); break;
	  case ENC_TYPE_AUTO: json_text+= String("WPA/WPA2/PSK"); break;
	  default: break;
	  }
	  json_text+="\"}";
	}
  json_text+="]}";

  server.send(200,"application/json",json_text);
}
void getInfoJson()
{
  String json_text;

  FSInfo info;
  SPIFFS.info(info);

  json_text+= "{";
  json_text+="\"fs_info\":[";
  json_text+="{\"name\":\"Full size\",\"value\":\""+String(info.totalBytes)+" Bytes\"},";
  json_text+="{\"name\":\"Used size\",\"value\":\""+String(info.usedBytes)+" Bytes\"}";

  json_text += "],";

	json_text+="\"files\":[";

  Dir d = SPIFFS.openDir("");
  int start=1;
  while (d.next()) {
    if(start) start=0;
    else json_text+= ",";

    String fileName = d.fileName();
    size_t fileSize = d.fileSize();
    json_text+="{\"name\":\""+fileName+"\", \"size\":\""+String(fileSize)+"Bytes\"}";
  }
  json_text+="],";

  json_text+="\"sys_info\":[";

	json_text+= "{\"name\":\"Data RAM\",\"value\":\"96 kBytes\" },";
	json_text+= "{\"name\":\"Free heap\",\"value\":\""+String(ESP.getFreeHeap())+" Bytes\" },";
  json_text+= "{\"name\":\"Chip ID\",\"value\":\""+String(ESP.getChipId())+"\" },";
  json_text+= "{\"name\":\"Flash size\",\"value\":\""+String(ESP.getFlashChipSize())+" Bytes\" },";
  json_text+= "{\"name\":\"Supply voltage\",\"value\":\""+String(ESP.getVcc())+" mV\" }";

  json_text+= "]";

	json_text+="}";

  server.send(200,"application/json",json_text);
}

void getLog()
{
	unsigned int i;
	for(i=0;i<LOG_BUFFER_SIZE;i++)
		SERVER_ANS_BUFFER[i] = Log_Buffer[(Log_Buffer_Index+i)%LOG_BUFFER_SIZE];

	SERVER_ANS_BUFFER[SERVER_ANS_BUFFER_SIZE] = '\0';

	uint8_t* tmp = SERVER_ANS_BUFFER;
	while(!(*tmp)) tmp++;
	server.send(200,"text/plain",String((char*)tmp));
}

/******************************************** UART handler ***********************************************/
void ProcessData(uint8_t *buffer)
{
  db_msg("Processing incoming data.");
    if(buffer[0] == WRITE_SONAR_DATA)// Add the known command id to the serial handler too!!!
    {
      // Iterate over the buffer and convert the values to ASCII
      uint8_t i;
      uint8_t len = buffer[1]/2;
      uint16_t *data16 = (uint16_t*)(buffer+2);
      char *wp = SONAR_data;
      for(i=0;i<len;i++)
      {
        db_msg("Processing number: "+String(*data16));
        if(*data16<=9999)
          itoa(*data16, wp, 10);
        else
          itoa(9999,wp,10);

        // search string end
        while(*wp) wp++;
        *wp++ =',';
        // step data16
        data16++;
      }
      // Clear last ',', and terminate the string
      wp--;
      *wp = 0;
    }
}


void handleReset()
{
  Log("Reset request arrived.");
  if(server.hasArg("reset") && server.arg("reset")==String("true"))
  {
    Log("Resetting due to online request.");
    server.send(200,"text/plain","ok");
    ESP.restart();
  }
}

void handleReconnect()
{
  Log("Reconnect request received.");
  if(server.hasArg("reconnect") && server.arg("reconnect")==String("true"))
  {
    Log("Restarting WiFi interface.");
    server.send(200,"text/plain","ok");
    initWifi();
  }
}

/********************************************* LOGGING *********************************************/
void Log_2_Buffer(const String& msg,unsigned int max_len)
{
	Log_2_Buffer(msg.c_str(),max_len);
}
void Log_2_Buffer(const char* msg,unsigned int max_len)
{
	unsigned int i;
	for(i=0;msg[i]!=0 && i<max_len;i++)
	{
		Log_Buffer[Log_Buffer_Index++]=msg[i];
		if(Log_Buffer_Index>=LOG_BUFFER_SIZE) Log_Buffer_Index=0;
	}
	Log_Buffer[Log_Buffer_Index++]='\n';
	if(Log_Buffer_Index>=LOG_BUFFER_SIZE) Log_Buffer_Index=0;

	db_msg(msg);
}
