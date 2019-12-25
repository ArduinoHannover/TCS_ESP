#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
// PubSubClient muss für längere Payloads verändert werden!

volatile uint32_t lastInt;
volatile uint32_t code;
volatile bool     length;
volatile uint32_t _code;
volatile uint8_t  _pos;
volatile bool     _crc;
volatile bool     _length;
volatile bool     _valid;
const uint8_t     _OUT = 15;

uint32_t last_door;
uint32_t last_ring;
uint32_t last_light;
uint16_t light_info;

const uint32_t UID       = 0x12345; // Seriennummer (in 16 Byte Nachrichten xSSSSSxxl
const uint16_t LIGHT_DUR =  130000; // Licht aktiv für x ms

const char* ssid        = "ssid";
const char* password    = "password";
IPAddress      mqtt_server(10, 0, 0, 1);
//const char* mqtt_server = "mqtt.local"

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
	Serial.begin(115200);
	Serial.println("WiFi Init");
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		Serial.write('.');
		delay(100);
	}
	Serial.println("Interrupt init");
	attachInterrupt(4, inputChange, CHANGE);
	Serial.println("OTA");
	ArduinoOTA.setPort(8266);
	ArduinoOTA.setPassword((const char *)"totalcontrolsystem");
	ArduinoOTA.setHostname("TCS");
	ArduinoOTA.begin();
	client.setServer(mqtt_server, 1883);
	client.setCallback(callback);
	Serial.println("MQTT connecting");
	mqttCon(true);
	pinMode(_OUT, OUTPUT);
	Serial.println("Ready.");
}

void mqttCon(boolean block) {
	while (!client.connected()) {
		yield();
		if (client.connect("TCS", "tcs/available", 0, (boolean)true, "offline")) {
			client.subscribe("tcs/do_open");
			client.subscribe("tcs/light/set");
			client.subscribe("tcs/cmd/send");
			client.publish("homeassistant/light/tcs/config", "{\"name\":\"TCS Light\",\"stat_t\":\"tcs/light\",\"json_attr_t\":\"tcs/light/info\",\"avty_t\":\"tcs/available\",\"cmd_t\":\"tcs/light/set\",\"qos\":0}", (bool)true);
			client.publish("homeassistant/binary_sensor/tcs/config", "{\"name\":\"TCS Ring\",\"stat_t\":\"tcs/ring\",\"avty_t\":\"tcs/available\",\"json_attr_t\":\"tcs/ring/info\",\"qos\":0,\"dev_cla\":\"occupancy\"}", (bool)true);
			client.publish("homeassistant/switch/tcs/config", "{\"name\":\"TCS Opener\",\"stat_t\":\"tcs/opener\",\"avty_t\":\"tcs/available\",\"cmd_t\":\"tcs/do_open\",\"qos\":0,\"opt\":false,\"ic\":\"mdi:lock-open-outline\"}", (bool)true);
			client.publish("tcs/available", "online", (boolean)true);
		}
		if (!block) break;
	}
}

void callback(char* topic, byte* payload, unsigned int length) {
	if (String("tcs/do_open").equals(topic)) {
		if (String((char*)payload).substring(0,length).equals("ON")) {
			sendCMD((uint16_t)0x1100);
		} else {
			Serial.println("Content mismatch");
		}
	} else if (String("tcs/light/set").equals(topic)) {
		if (String((char*)payload).substring(0,length).equals("ON")) {
			sendCMD((uint16_t)0x1200);
		} else {
			Serial.println("Content mismatch");
		}
	} else if (String("tcs/cmd/send").equals(topic)) {
		if (length == 4) {
			sendCMD((uint16_t)strtol(String((char*)payload).substring(0,length).c_str(), NULL, 16));
		} else if (length == 8) {
			sendCMD((uint32_t)strtol(String((char*)payload).substring(0,length).c_str(), NULL, 16));
		}
	}
}

void sendCMD(uint32_t cmd, uint8_t length) {
	if (length != 16 && length != 32) return;
	bool crc = 1;
	digitalWrite(_OUT, 1);
	delay(6);
	digitalWrite(_OUT, 0);
	delay(length >> 3);
	for (uint8_t i = 0; i < length; i++) {
		digitalWrite(_OUT, !(i & 1));
		bool b = cmd >> (length - 1 - i) & 1;
		crc ^= b;
		delay(b ? 4 : 2);
	}
	digitalWrite(_OUT, 1);
	delay(crc ? 4 : 2);
	digitalWrite(_OUT, 0);
}

void sendCMD(uint16_t cmd) {
	sendCMD(cmd, 16);
}

void sendCMD(uint32_t cmd) {
	sendCMD(cmd, 32);
}

ICACHE_RAM_ATTR void inputChange() {
	uint32_t curInt = micros();
	uint32_t t = curInt - lastInt;
	if (lastInt > curInt) {
		t = 0xFFFFFFFF - lastInt + curInt;
	}
	if (t < 1000) {
		Serial.write('I');
	} else if (t < 5000) {
		bool b = 1;
		if (del < 3000) {
			b = 0;
		}
		
		if (!_pos) {
			_length = b;
			Serial.print(b ? "32b " : "16b ");
		} else if (_pos <= (_length ? 32 : 16)) {{
				_code <<= 1;
				_code |= b;
			}
			_crc ^= b;
			Serial.write(b + '0');
		} else if (_pos == (_length ? 33 : 17)) {
			Serial.print("CRC ");
			if (b == _crc) {
				Serial.println("OK");
				code = _code;
				length = _length;
			} else {
				Serial.println("NOK");
			}
		} else {
			Serial.println("W");
		}
		_pos++;
	} else if (t < 7000) {
		Serial.println();
		Serial.println(_pos);
		Serial.write('B');
		_pos = 0;
		_crc = 1;
		_code = 0;
	} else if (t < 24000) {
		Serial.write('R');
	} else {
		Serial.println("TOUT");
	}
	lastInt = curInt;
}

void loop() {
	if (code != 0) {
		Serial.println("CODE:");
		Serial.println(code, HEX);
		char h[10];
		if (length) {
			switch (code >> 28) {
				case 0x0:
					switch ((code >> 8) & 0xFFFFF) {
						case UID:
							last_ring = millis();
							client.publish("tcs/ring/info", "{\"Source\":\"external\"}", (bool)true);
							client.publish("tcs/ring", "ON", (bool)true);
							Serial.println("Klingel unten");
							break;
						default:
							// Andere Klingel
							break;
					}
					break;
				case 0x1:
					Serial.println("Klingelt");
					switch ((code >> 8) & 0xFFFFF) {
						case UID:
							last_ring = millis();
							client.publish("tcs/ring/info", "{\"Source\":\"internal\"}", (bool)true);
							client.publish("tcs/ring", "ON", (bool)true);
							Serial.println("Eigene Etagenklingel");
							break;
						default:
							// Andere Klingel
							break;
					}
					break;
				case 0x3:
					// Eventuell muss man auf die 2 Byte ACK Response achten, da bei NACK kein 0x30XX geschickt wird beim Auflegen
					client.publish("tcs/call", "ON", (bool)true);
					switch ((code >> 8) & 0xFFFFF) {
						case UID:
							//Eigenes Telefon
						default:
							break;
					}
					break;
				default:
					Serial.println("UNKOWN");
			}
			sprintf(h, " %08X", code);
		} else {
			switch (code >> 8) {
				case 0x11:
					client.publish("tcs/opener", "ON", (bool)true);
					last_door = millis();
					Serial.println("Tueroffner");
					break;
				case 0x12:
					last_light = millis();
					client.publish("tcs/light", "ON", (bool)true);
					Serial.println("Licht an");
					break;
				case 0x22:
					client.publish("tcs/ring", "OFF", (bool)true);
					Serial.println("Klingeln beendet");
					// Nur gesendet, wenn von einem TCS Gerät geklingelt wurde
					break;
				case 0x30:
					client.publish("tcs/call", "OFF", (bool)true);
					Serial.println("Ruf beendet");
					break;
			}
			sprintf(h, " %04X", code);
		}
		client.publish("tcs/cmd", h);
		code = 0;
	}
	if (last_door && last_door + 7000 <= millis()) {
		last_door = 0;
		client.publish("tcs/opener", "OFF", (bool)true);
	}
	if (last_ring && last_ring + 5000 <= millis()) {
		// Timout, wenn nicht über offizielle Wege geklingel wurde
		last_ring = 0;
		client.publish("tcs/ring", "OFF", (bool)true);
	}
	if (last_light) {
		uint16_t cl = max((int16_t)0, (int16_t)((last_light + LIGHT_DUR - millis()) / 1000));
		if (cl != light_info) {
			client.publish("tcs/light/info", (String("{\"Remaining\":") + cl + "}").c_str(), (bool)true);
			if (!cl) {
				last_light = 0;
				client.publish("tcs/light", "OFF", (bool)true);
			}
			light_info = cl;
		}
	}
	ArduinoOTA.handle();
	mqttCon(false);
	client.loop();
}
