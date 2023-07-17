//  FARM DATA RELAY SYSTEM
//
//  GATEWAY CONFIGURATION

//Addresses
#define UNIT_MAC           0x00  // The address of this gateway

//#define ESPNOW_NEIGHBOR_1  0x99  // Address of ESP-NOW neighbor #1
//#define ESPNOW_NEIGHBOR_2  0x99  // Address of ESP-NOW neighbor #2
#define LORA_NEIGHBOR_1    0x01  // Address of LoRa neighbor #1
#define LORA_NEIGHBOR_2    0x99  // Address of LoRa neighbor #2

// Interfaces
//#define USE_ESPNOW
#define USE_LORA
//#define USE_MQTT


// Routing
// Options: sendESPNowNbr(1 or 2); sendESPNow(0xNN); sendESPNowPeers(); sendLoRaNbr(1 or 2); broadcastLoRa(); sendSerial(); sendMQTT();
#define ESPNOWG_ACT    
#define LORAG_ACT                                       
#define SERIAL_ACT      
#define MQTT_ACT        sendLoRaNbr(1);
#define INTERNAL_ACT    sendLoRaNbr(1);
#define ESPNOW1_ACT
#define ESPNOW2_ACT    
#define LORA1_ACT      
#define LORA2_ACT      


// LoRa Configuration
//#define REZODO
#ifdef REZODO
// LoRa Configuration for REZODO purple board
#define RADIOLIB_MODULE RFM95
#define LORA_SS 27
#define LORA_RST 14
#define LORA_DIO 12
#define LORA_BUSY -1
//#define USE_SX126X


#define CUSTOM_SPI
#define LORA_SPI_SCK 26
#define LORA_SPI_MISO 35
#define LORA_SPI_MOSI 25

#else
// LoRa Configuration for Hallard's board
#define RADIOLIB_MODULE RFM95
#define LORA_SS    16
#define LORA_RST   0
#define LORA_DIO   23
#define LORA_BUSY  -1
//#define USE_SX126X


#define CUSTOM_SPI
#define LORA_SPI_SCK  17
#define LORA_SPI_MISO 5
#define LORA_SPI_MOSI 18
#endif //REZODO

#define FDRS_DEBUG     // Enable USB-Serial debugging
#define DEBUG_CONFIG   //should display config at boot

// OLED -- Displays console debugging messages on an SSD1306 I²C OLED
///#define USE_OLED
#define OLED_HEADER "FDRS"
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

// UART data interface pins (if available)
#define RXD2 14
#define TXD2 15

//#define USE_LR  // Use ESP-NOW LR mode (ESP32 only)

// MQTT Credentials  -- These will override the global settings

#define MQTT_ADDR   "192.168.0.48" //homeassistant.local
#define MQTT_PORT   1883 // Default MQTT port is 1883
#define MQTT_AUTH   //Enable MQTT authentication
#define MQTT_USER   "MQTT_access"
#define MQTT_PASS   "JP"
