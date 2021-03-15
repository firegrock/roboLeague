// загружаем библиотеку для WiFi:
#include <WiFi.h>

// вставляем ниже SSID и пароль для своей WiFi-сети:
const char* ssid = "Rulon";
const char* password = "US1NGnameSPACE";

// создаем объект сервера и задаем ему порт «80»:
WiFiServer server(80);

// мотор 1:
int motor1Pin1 = 27; 
int motor1Pin2 = 14; 
int enable1Pin = 15; 

// мотор 2:
int motor2Pin1 = 12; 
int motor2Pin2 = 13; 
int enable2Pin = 32;

// переменные для свойств широтно-импульсной модуляции (ШИМ):
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 0;

int val;


void setup() {
  Serial.begin(115200);
  
  // переключаем контакты моторов в режим «OUTPUT»:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // задаем настройки ШИМ-канала:
  ledcSetup(pwmChannel, freq, resolution);
  
  // подключаем ШИМ-канал 0 к контактам ENA и ENB,
  // т.е. к GPIO-контактам для управления скоростью вращения моторов:
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel);

  // подаем на контакты ENA и ENB 
  // ШИМ-сигнал с коэффициентом заполнения «0»:
  ledcWrite(pwmChannel, dutyCycle);
  
  // подключаемся к WiFi-сети при помощи заданных выше SSID и пароля:
  Serial.print("Connecting to ");  //  "Подключаемся к "
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // печатаем в мониторе порта
  // локальный IP-адрес и запускаем веб-сервер:
  Serial.println("");
  Serial.println("WiFi connected.");  //  "Подключились к WiFi-сети."
  Serial.println("IP address: ");  //  "IP-адрес: "
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();  // Запускаем прослушку 
                                           // входящих клиентов.

  if (!client) {
    return;
  }

  String req = client.readStringUntil('\r');
  client.flush();
  //  Serial.print("\nMessage recieved");

  if (req.indexOf("=") != -1) //Ищем в принятом "=" - это и есть ключ. Извлекаем цифру после "=" и подставляем в ШИМ
              {
                int val = (int)req[req.indexOf("=") + 1] - 48; //Особая уличная магия ардуинщиков
                Serial.println(val);
                delay(500);
              }
          
          // Это конец HTTP-запроса клиента, поэтому отправляем ответ:
          if (req.length() == 0) 
          {
            // HTTP-заголовки всегда начинаются с кода ответа
            // (например, с «HTTP/1.1 200 OK»),
            // а также с информации о типе контента,
            // чтобы клиент знал, что получает.
            // После этого пишем пустую строчку:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
                       //  "Соединение: отключено"
            client.println();
          }
            // Этот код отвечает за управление контактами моторов
            // согласно тому, в соответствии с положением джойстика:

            
  
            if (req.indexOf("=5") != -1) {
              Serial.println("Idle");  //  
              digitalWrite(motor1Pin1, LOW);
              digitalWrite(motor1Pin2, LOW); 
              digitalWrite(motor2Pin1, LOW);
              digitalWrite(motor2Pin2, LOW);
            }  
            if (req.indexOf("=1") != -1) {
              Serial.println("Forward");  //  "Вперед"
              digitalWrite(motor1Pin1, LOW);
              digitalWrite(motor1Pin2, HIGH); 
              digitalWrite(motor2Pin1, LOW);
              digitalWrite(motor2Pin2, HIGH);
            }  
            else if (req.indexOf("=3") != -1) {
              Serial.println("Left");  //  "Влево"
              digitalWrite(motor1Pin1, LOW); 
              digitalWrite(motor1Pin2, LOW); 
              digitalWrite(motor2Pin1, LOW);
              digitalWrite(motor2Pin2, HIGH);
            }  
               
            else if (req.indexOf("=4") != -1) {
              Serial.println("Right");  //  "Вправо"
              digitalWrite(motor1Pin1, LOW); 
              digitalWrite(motor1Pin2, HIGH); 
              digitalWrite(motor2Pin1, LOW);
              digitalWrite(motor2Pin2, LOW);    
            } 
            else if (req.indexOf("=2") != -1) {
              Serial.println("Reverse");  //  "Назад"
              digitalWrite(motor1Pin1, HIGH);
              digitalWrite(motor1Pin2, LOW); 
              digitalWrite(motor2Pin1, HIGH);
              digitalWrite(motor2Pin2, LOW);          
            }
      }
