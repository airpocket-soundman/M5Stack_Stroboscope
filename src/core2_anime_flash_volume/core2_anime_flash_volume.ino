#include <M5Unified.h>
#include <SD.h>
#include <esp_timer.h>

// ===== 再生設定 =====
static int  FRAME_COUNT      = 199;   // 再生枚数（初期199）
static int  FRAME_DELAY_MS   = 0;     // フレーム間待ち(0=できるだけ速く)
static const int START_Y     = 0;     // 縦位置は固定（必要なら変えてOK）
static const int LCD_W       = 320;   // Core2: 320x240

// ===== ストロボ（フレームとは非同期・独立） =====
// 初期値（参照コード準拠）
static volatile uint32_t g_on_us  = 100;     // 点灯時間(µs)
static volatile uint32_t g_off_us = 30000;   // 消灯時間(µs) → 周期 = on + off

// つまみ（Core2のADC入力）
static const int ADC_OFF_PIN = 36;  // サイクル(=off)用
static const int ADC_ON_PIN  = 35;  // 点灯(on)用

// マッピング範囲（必要に応じて調整）
static const uint32_t OFF_MIN_US = 1000;     // 1ms
static const uint32_t OFF_MAX_US = 100000;   // 100ms
static const uint32_t ON_MIN_US  = 10;       // 10µs
static const uint32_t ON_MAX_US  = 10000;    // 10ms

static const float ADC_EMA_ALPHA = 0.20f;    // 平滑

// ストロボ出力
static const int STROBE_PIN = 19;

// タイマ
static esp_timer_handle_t g_periodic_timer     = nullptr;
static esp_timer_handle_t g_one_shot_off_timer = nullptr;

// ===== 画像用：PSRAMバッファ使い回し（毎回malloc/freeしない） =====
static uint8_t* g_buf = nullptr;
static size_t   g_cap = 0;

// ===== ユーティリティ =====
static inline uint32_t umap(uint32_t x, uint32_t in_min, uint32_t in_max,
                            uint32_t out_min, uint32_t out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  uint64_t num = (uint64_t)(x - in_min) * (out_max - out_min);
  return out_min + (uint32_t)(num / (in_max - in_min));
}

// BMP幅をファイルヘッダから取得（リトルエンディアン / 18バイト目から4バイト）
static int getBmpWidth(File &f) {
  uint8_t hdr[26];
  if (!f.seek(0)) return -1;
  size_t r = f.read(hdr, sizeof(hdr));
  if (r < 22) return -1;
  // 幅（int32 LE）。負値はトップダウンBMPだがここでは絶対値扱い
  int32_t w = (int32_t)( (uint32_t)hdr[18]
                       | ((uint32_t)hdr[19] << 8)
                       | ((uint32_t)hdr[20] << 16)
                       | ((uint32_t)hdr[21] << 24) );
  if (w < 0) w = -w;
  // ファイルポインタはこの後読み直すので先頭へ戻すのを忘れずに
  f.seek(0);
  return w;
}

// ===== ストロボ割り込み =====
static void IRAM_ATTR one_shot_off_cb(void*) {
  gpio_set_level((gpio_num_t)STROBE_PIN, 0);
}

static void IRAM_ATTR periodic_cb(void*) {
  gpio_set_level((gpio_num_t)STROBE_PIN, 1);
  uint32_t on_us = g_on_us;
  if (g_one_shot_off_timer) {
    esp_timer_stop(g_one_shot_off_timer);
    esp_timer_start_once(g_one_shot_off_timer, on_us);
  }
}

static void update_periodic_timer(uint32_t on_us, uint32_t off_us) {
  // 範囲ガード
  if (on_us  < ON_MIN_US)  on_us  = ON_MIN_US;
  if (on_us  > ON_MAX_US)  on_us  = ON_MAX_US;
  if (off_us < OFF_MIN_US) off_us = OFF_MIN_US;
  if (off_us > OFF_MAX_US) off_us = OFF_MAX_US;

  g_on_us  = on_us;
  g_off_us = off_us;

  uint32_t period_us = on_us + off_us;
  if (g_periodic_timer) {
    esp_timer_stop(g_periodic_timer);
    esp_timer_start_periodic(g_periodic_timer, period_us); // µs周期
  }
  Serial.printf("Strobe set: on=%u us, off=%u us (period=%u us)\n", on_us, off_us, period_us);
}

// ===== セットアップ =====
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== Setup ===");

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(0);
  M5.Display.fillScreen(TFT_BLACK);

  // SD（Core2のTFスロット）
  if (!SD.begin(GPIO_NUM_4)) {
    Serial.println("SD init failed");
    while (true) delay(1000);
  }
  Serial.println("SD OK");

  // ストロボ出力
  pinMode(STROBE_PIN, OUTPUT);
  digitalWrite(STROBE_PIN, LOW);

  // ADC
  analogReadResolution(12);

  // タイマ作成
  esp_timer_create_args_t periodic_args = {};
  periodic_args.callback = &periodic_cb;
  periodic_args.name     = "strobe_periodic";
  ESP_ERROR_CHECK(esp_timer_create(&periodic_args, &g_periodic_timer));

  esp_timer_create_args_t off_args = {};
  off_args.callback = &one_shot_off_cb;
  off_args.name     = "strobe_off";
  ESP_ERROR_CHECK(esp_timer_create(&off_args, &g_one_shot_off_timer));

  // 初期ストロボ設定（参照コード準拠）
  update_periodic_timer(g_on_us, g_off_us);

  Serial.println("=== Ready ===");
}

// ===== メインループ =====
void loop() {
  // 1) アナログ入力から on/off を更新（独立保持）
  static float ema_off = 2000.0f, ema_on = 500.0f;
  int raw_off = analogRead(ADC_OFF_PIN);   // 0..4095
  int raw_on  = analogRead(ADC_ON_PIN);    // 0..4095
  ema_off = (1.0f - ADC_EMA_ALPHA) * ema_off + ADC_EMA_ALPHA * raw_off;
  ema_on  = (1.0f - ADC_EMA_ALPHA) * ema_on  + ADC_EMA_ALPHA * raw_on;

  uint32_t new_off_us = umap((uint32_t)ema_off, 0, 4095, OFF_MIN_US, OFF_MAX_US);
  uint32_t new_on_us  = umap((uint32_t)ema_on,  0, 4095, ON_MIN_US,  ON_MAX_US);

  static uint32_t last_off_us = 0, last_on_us = 0;
  if (abs((int)new_off_us - (int)last_off_us) > 50 ||
      abs((int)new_on_us  - (int)last_on_us)  > 10) {
    update_periodic_timer(new_on_us, new_off_us);
    last_off_us = new_off_us;
    last_on_us  = new_on_us;
  }

  // 2) 画像を連番で強制再生（中心寄せ：X = (320 - 画像幅)/2）
  for (int i = 1; i <= FRAME_COUNT; ++i) {
    char filename[32];
    sprintf(filename, "/%04d_240.bmp", i);

    File f = SD.open(filename, FILE_READ);
    if (!f) { delay(1); continue; }

    // 幅取得 → 中央X算出
    int imgW = getBmpWidth(f);
    int x = 0;
    if (imgW > 0 && imgW < LCD_W) {
      x = (LCD_W - imgW) / 2;
    } else if (imgW >= LCD_W) {
      x = 0; // 画面幅超えは左寄せで描画
    }

    size_t len = f.size();
    if (len > g_cap) {
      if (g_buf) { free(g_buf); g_buf = nullptr; g_cap = 0; }
      g_buf = (uint8_t*)ps_malloc(len);   // PSRAMに確保
      if (!g_buf) {
        f.close();
        Serial.println("ps_malloc failed");
        delay(1);
        continue;
      }
      g_cap = len;
    }

    // 読み込み → 描画（使い回しバッファで高速）
    f.read(g_buf, len);
    f.close();
    M5.Display.drawBmp(g_buf, len, 0,40); //x, START_Y);

    if (FRAME_DELAY_MS > 0) delay(FRAME_DELAY_MS);
    else delay(1); // 他処理に呼吸
  }

  // そのままループして繰り返し再生
}

