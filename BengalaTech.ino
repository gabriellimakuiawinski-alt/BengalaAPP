/*
 * ACCESS — Firmware da Bengala Tech (ESP32-CAM AI-Thinker)
 * ---------------------------------------------------------------------------------------------
 * Escrito do zero para o aplicativo ACCESS v3.0. Fala o MESMO protocolo das versões anteriores:
 *
 *   celular -> bengala :  "FOTO\n"    captura e envia uma foto
 *                         "PING\n"    responde "PONG\n"
 *                         "STATUS\n"  responde "ESP32_OK\n"
 *   bengala -> celular :  "FOTO:<tamanho>\n" seguido de EXATAMENTE <tamanho> bytes JPEG
 *                         "ERRO_CAMERA\n"    se não conseguiu capturar
 *
 * Botão físico (opcional): ao apertar, a bengala captura e envia a foto sozinha, usando o mesmo
 * "FOTO:<tamanho>\n" + JPEG. O aplicativo já aceita esse envio sem ter pedido, então não existe
 * protocolo novo.
 *
 * REGRA DE OURO: nunca escreva texto de depuração em SerialBT. Qualquer byte extra no meio do
 * canal Bluetooth atrapalha o aplicativo. Os logs vão só para o Serial (USB).
 *
 * COMO COMPILAR (Arduino IDE)
 *   Placa ............ "AI Thinker ESP32-CAM"  (pacote "esp32" da Espressif)
 *   Partition Scheme . "Huge APP (3MB No OTA/1MB SPIFFS)"   <- obrigatório: Bluetooth + câmera é grande
 *   PSRAM ............ Enabled
 *   Nenhuma biblioteca extra: esp_camera e BluetoothSerial já vêm com o pacote da placa.
 *   Para gravar: ligue o GPIO0 ao GND, aperte RESET, grave, desligue o GPIO0 e aperte RESET de novo.
 *
 * LIGAÇÕES
 *   Botão físico ..... entre o GPIO13 e o GND (usa pull-up interno, não precisa de resistor)
 *   LED vermelho ..... é o LED da própria placa (GPIO33): aceso = celular conectado
 *   Alimentação ...... fonte de 5 V com pelo menos 1 A. Fonte fraca causa reinicializações,
 *                      imagem cortada e o erro "FB-OVF". Se puder, ponha um capacitor de
 *                      470 µF a 1000 µF entre 5V e GND, perto da placa.
 *
 * Este código foi escrito com base na documentação do pacote esp32 e na API esp_camera, mas ainda
 * NÃO foi testado em uma placa real. Teste primeiro com PING, depois com FOTO.
 */

#include "esp_camera.h"
#include "BluetoothSerial.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic nao esta habilitado. Use uma placa ESP32 (nao S2/S3/C3) e o pacote esp32 da Espressif."
#endif

// =============================== CONFIGURAÇÃO ================================================

#define DEVICE_NAME      "BengalaTech"   // O app procura exatamente este nome entre os pareados.

// Imagem. SVGA (800x600) é um bom equilíbrio: a IA enxerga bem e o envio leva ~1 s.
#define FRAME_SIZE       FRAMESIZE_SVGA
#define JPEG_QUALITY     12              // 10 = melhor qualidade, 63 = mais comprimido. Use 10 a 20.
#define XCLK_HZ          10000000        // 10 MHz é mais estável que 20 MHz (evita "FB-OVF").
#define DISCARD_FRAMES   1               // quadros descartados antes da foto (evita imagem velha)

// Orientação da câmera na bengala (troque para 1 se a imagem sair de cabeça para baixo/espelhada).
#define CAMERA_VFLIP     0
#define CAMERA_HMIRROR   0

// Lanterna (LED branco forte da placa, GPIO4). Ajuda no escuro, mas ofusca. 0 = desligada.
#define USE_FLASH        0

// Botão físico da bengala.
#define USE_BUTTON       1
#define BUTTON_PIN       13
#define BUTTON_COOLDOWN_MS 2500          // tempo mínimo entre fotos disparadas pelo botão

// Deixe 1 apenas se a placa reiniciar por "brownout" e você já tiver tentado melhorar a fonte.
#define DISABLE_BROWNOUT 0

// Envio Bluetooth.
#define CHUNK_SIZE       512             // bytes por escrita
#define SEND_STALL_MS    4000            // desiste se o celular parar de receber por tanto tempo

// ============================ PINOS DA AI-THINKER ============================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_PIN          4
#define LED_PIN           33             // LED vermelho da placa (acende com LOW)

// =============================== ESTADO ======================================================

BluetoothSerial SerialBT;

static String   rxLine;
static bool     cameraReady = false;
static uint8_t  consecutiveFailures = 0;
static bool     ledOn = false;

#if USE_BUTTON
static bool     buttonDown = false;
static uint32_t buttonChangedAt = 0;
static uint32_t lastButtonPhotoAt = 0;
#endif

// =============================== CÂMERA ======================================================

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;

  if (psramFound()) {
    // Dois quadros em PSRAM: a câmera continua capturando enquanto o anterior é enviado,
    // e GRAB_LATEST sempre entrega o quadro mais recente. Isso evita o "FB-OVF".
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("[camera] PSRAM nao encontrada: usando QVGA (ative PSRAM nas opcoes da placa).");
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[camera] falha ao iniciar: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != nullptr) {
    s->set_vflip(s, CAMERA_VFLIP);
    s->set_hmirror(s, CAMERA_HMIRROR);
    s->set_gain_ctrl(s, 1);       // ganho automático
    s->set_exposure_ctrl(s, 1);   // exposição automática
    s->set_awb_gain(s, 1);
    s->set_aec2(s, 1);            // melhora a exposição com pouca luz
    s->set_lenc(s, 1);            // corrige o escurecimento nas bordas da lente
  }
  Serial.println("[camera] pronta");
  return true;
}

static bool restartCamera() {
  esp_camera_deinit();
  delay(150);
  cameraReady = initCamera();
  return cameraReady;
}

static void setQuality(int quality) {
  sensor_t *s = esp_camera_sensor_get();
  if (s != nullptr) s->set_quality(s, quality);
}

static camera_fb_t *grabFrame() {
  for (uint8_t i = 0; i < DISCARD_FRAMES; i++) {
    camera_fb_t *old = esp_camera_fb_get();
    if (old != nullptr) esp_camera_fb_return(old);
  }
  return esp_camera_fb_get();
}

// Devolve o tamanho útil do JPEG (até o marcador final FF D9) ou 0 se o quadro não for um JPEG válido.
static size_t jpegLength(const camera_fb_t *fb) {
  if (fb == nullptr || fb->format != PIXFORMAT_JPEG || fb->len < 4) return 0;
  if (fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) return 0;
  size_t from = fb->len > 16 ? fb->len - 16 : 2;
  for (size_t i = fb->len - 2; i >= from; i--) {
    if (fb->buf[i] == 0xFF && fb->buf[i + 1] == 0xD9) return i + 2;
  }
  return 0;
}

// =============================== BLUETOOTH ===================================================

static void sendLine(const char *text) {
  SerialBT.print(text);
  SerialBT.print('\n');
}

// Escreve tudo, em blocos, respeitando o que o Bluetooth aceitou de fato.
static bool writeAll(const uint8_t *data, size_t len) {
  size_t sent = 0;
  uint32_t lastProgress = millis();
  while (sent < len) {
    if (!SerialBT.hasClient()) return false;
    size_t remaining = len - sent;
    size_t chunk = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
    size_t written = SerialBT.write(data + sent, chunk);
    if (written > 0) {
      sent += written;
      lastProgress = millis();
    } else {
      if (millis() - lastProgress > SEND_STALL_MS) return false;
      delay(2);
    }
    yield();
  }
  return true;
}

// Captura uma foto e a envia no protocolo FOTO:<tamanho>\n + JPEG.
static void sendPhoto() {
  if (!SerialBT.hasClient()) return;

  camera_fb_t *fb = nullptr;
  size_t length = 0;

  if (!cameraReady && !restartCamera()) {
    sendLine("ERRO_CAMERA");
    return;
  }
  setQuality(JPEG_QUALITY);

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
#if USE_FLASH
    digitalWrite(FLASH_PIN, HIGH);
    delay(80);
#endif
    fb = grabFrame();
#if USE_FLASH
    digitalWrite(FLASH_PIN, LOW);
#endif
    length = jpegLength(fb);
    if (length > 0) break;

    Serial.printf("[foto] tentativa %u falhou\n", attempt + 1);
    if (fb != nullptr) {
      esp_camera_fb_return(fb);
      fb = nullptr;
    }
    if (attempt == 0) {
      setQuality(JPEG_QUALITY + 6);   // comprime mais: quadros menores não estouram o buffer
    } else {
      restartCamera();                // última cartada: reinicia o driver da câmera
    }
    delay(60);
  }

  if (length == 0 || fb == nullptr) {
    sendLine("ERRO_CAMERA");
    consecutiveFailures++;
    Serial.printf("[foto] falhas seguidas: %u\n", consecutiveFailures);
    if (consecutiveFailures >= 3) {
      Serial.println("[foto] reiniciando a placa para recuperar a camera");
      delay(300);
      ESP.restart();
    }
    return;
  }

  char header[24];
  int headerLen = snprintf(header, sizeof(header), "FOTO:%u\n", (unsigned)length);
  bool ok = writeAll((const uint8_t *)header, (size_t)headerLen) && writeAll(fb->buf, length);
  esp_camera_fb_return(fb);

  if (ok) {
    SerialBT.flush();
    consecutiveFailures = 0;
    Serial.printf("[foto] enviada: %u bytes\n", (unsigned)length);
  } else {
    // O celular perdeu o fim da imagem: encerra o link para ele reconectar e sincronizar de novo.
    Serial.println("[foto] envio interrompido; desconectando para ressincronizar");
    SerialBT.disconnect();
  }
}

static void handleCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command.length() == 0) return;
  Serial.printf("[cmd] %s\n", command.c_str());

  if (command == "FOTO") {
    sendPhoto();
  } else if (command == "PING") {
    sendLine("PONG");
  } else if (command == "STATUS") {
    sendLine("ESP32_OK");
  }
  // Qualquer outro texto é ignorado de propósito.
}

static void readCommands() {
  while (SerialBT.available()) {
    int value = SerialBT.read();
    if (value < 0) break;
    char c = (char)value;
    if (c == '\n') {
      handleCommand(rxLine);
      rxLine = "";
    } else if (c != '\r' && rxLine.length() < 32) {
      rxLine += c;
    }
  }
}

// ============================== BOTÃO E LED ==================================================

static void handleButton() {
#if USE_BUTTON
  bool down = digitalRead(BUTTON_PIN) == LOW;
  uint32_t now = millis();
  if (down != buttonDown && now - buttonChangedAt > 40) {   // 40 ms de antirruído
    buttonDown = down;
    buttonChangedAt = now;
    if (down && SerialBT.hasClient() && now - lastButtonPhotoAt > BUTTON_COOLDOWN_MS) {
      Serial.println("[botao] foto");
      sendPhoto();
      lastButtonPhotoAt = millis();
    }
  }
#endif
}

static void updateLed() {
  bool connected = SerialBT.hasClient();
  if (connected != ledOn) {
    ledOn = connected;
    digitalWrite(LED_PIN, connected ? LOW : HIGH);   // LED da placa acende com LOW
  }
}

// ================================ ARDUINO ====================================================

void setup() {
#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
  Serial.begin(115200);
  Serial.println();
  Serial.println("[bengala] iniciando");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
#if USE_FLASH
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);
#endif
#if USE_BUTTON
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#endif

  rxLine.reserve(32);
  cameraReady = initCamera();   // se falhar, o app recebe ERRO_CAMERA e a placa tenta de novo no próximo pedido

  SerialBT.begin(DEVICE_NAME);
  Serial.printf("[bluetooth] visivel como \"%s\"\n", DEVICE_NAME);
}

void loop() {
  readCommands();
  handleButton();
  updateLed();
  delay(5);
}
