#include <M5Unified.h>
#include <SD.h>

// ======================== 再生設定 ========================
static int  FRAME_COUNT      = 199;   // 再生枚数
static int  FRAME_DELAY_MS   = 0;     // フレーム間待ち(0=できるだけ速く)
static const int START_Y     = 0;     // 縦位置（今回は固定）
static const int LCD_W       = 320;   // Core2: 320x240

// ======================== ストロボ（LEDC） ========================
// 初期値
static volatile uint32_t g_on_us  = 100;     // 点灯時間(µs)
static volatile uint32_t g_off_us = 30000;   // 消灯時間(µs) → 周期 = on + off

// つまみ（ADC1系：Wi-Fiと非競合）
static const int ADC_OFF_PIN = 36;  // 周期(=off時間)用
static const int ADC_ON_PIN  = 35;  // on時間用

// マッピング範囲
static const uint32_t OFF_MIN_US = 1000;     // 1ms
static const uint32_t OFF_MAX_US = 100000;   // 100ms
static const uint32_t ON_MIN_US  = 10;       // 10µs
static const uint32_t ON_MAX_US  = 10000;    // 10ms

static const float ADC_EMA_ALPHA = 0.20f;    // 平滑(0<alpha<=1)

// ストロボ出力（SDと非競合ピン）
static const int STROBE_PIN = 26;

// LEDC設定
static const int LEDC_CHANNEL  = 0;
static const int LEDC_TIMER    = 0;          // （ESP32はタイマIDを意識しない運用でOK）
static const int LEDC_RES_BITS = 12;         // 12bit分解能
static const int LEDC_BASE     = (1 << LEDC_RES_BITS) - 1;

// ======================== 画像用バッファ ========================
static uint8_t* g_buf = nullptr;
static size_t   g_cap = 0;

// ======================== ユーティリティ ========================
static inline uint32_t umap(uint32_t x, uint32_t in_min, uint32_t in_max,
                            uint32_t out_min, uint32_t out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  uint64_t num = (uint64_t)(x - in_min) * (out_max - out_min);
  return out_min + (uint32_t)(num / (in_max - in_min));
}

// LEDCに周期・デューティを反映（動的変更OK）
static void update_strobe_pwm(uint32_t on_us, uint32_t off_us) {
  // 範囲ガード
  if (on_us  < ON_MIN_US)  on_us  = ON_MIN_US;
  if (on_us  > ON_MAX_US)  on_us  = ON_MAX_US;
  if (off_us < OFF_MIN_US) off_us = OFF_MIN_US;
  if (off_us > OFF_MAX_US) off_us = OFF_MAX_US;

  g_on_us  = on_us;
  g_off_us = off_us;

  uint32_t period_us = on_us + off_us;
  if (period_us == 0) period_us = 1;

  double freq = 1000000.0 / (double)period_us;   // 周波数[Hz]

  // 周波数変更（動的OK）
  ledcSetup(LEDC_CHANNEL, freq, LEDC_RES_BITS);

  // Duty（0..LEDC_BASE）
  uint32_t duty = (uint32_t)((double)on_us * (double)LEDC_BASE / (double)period_us);
  if (duty > LEDC_BASE) duty = LEDC_BASE;

  // 反映
  ledcWrite(LEDC_CHANNEL, duty);

  // 基本ログ
  Serial.printf("LEDC set: on=%u us, off=%u us, period=%u us, freq=%.3f Hz, duty=%u/%d\n",
                on_us, off_us, period_us, freq, duty, LEDC_BASE);
}

// ======================== セットアップ ========================
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

  // ストロボ出力（LEDC）
  pinMode(STROBE_PIN, OUTPUT);
  ledcAttachPin(STROBE_PIN, LEDC_CHANNEL);
  ledcSetup(LEDC_CHANNEL, 1000, LEDC_RES_BITS);  // 仮設定
  ledcWrite(LEDC_CHANNEL, 0);                    // 消灯スタート

  // ADC初期化
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_OFF_PIN, ADC_11db);
  analogSetPinAttenuation(ADC_ON_PIN,  ADC_11db);

  // 初期ストロボ設定
  update_strobe_pwm(g_on_us, g_off_us);

  Serial.println("=== Ready ===");
}

// ======================== メインループ ========================
void loop() {
  // --- 1) ADC読み取り & 平滑 ---
  static float ema_off = 2000.0f, ema_on = 500.0f;
  int raw_off = analogRead(ADC_OFF_PIN);   // 0..4095
  int raw_on  = analogRead(ADC_ON_PIN);    // 0..4095
  ema_off = (1.0f - ADC_EMA_ALPHA) * ema_off + ADC_EMA_ALPHA * raw_off;
  ema_on  = (1.0f - ADC_EMA_ALPHA) * ema_on  + ADC_EMA_ALPHA * raw_on;

  uint32_t new_off_us = umap((uint32_t)ema_off, 0, 4095, OFF_MIN_US, OFF_MAX_US);
  uint32_t new_on_us  = umap((uint32_t)ema_on,  0, 4095, ON_MIN_US,  ON_MAX_US);

  // --- 2) ある程度変化したときだけLEDC更新＆ログ ---
  static uint32_t last_off_us = 0, last_on_us = 0;
  if (abs((int)new_off_us - (int)last_off_us) > 500 ||   // 例：500µs以上変化
      abs((int)new_on_us  - (int)last_on_us)  > 100) {   // 例：100µs以上変化

    update_strobe_pwm(new_on_us, new_off_us);

    uint32_t period_us = new_on_us + new_off_us;
    float    freq_hz   = period_us ? 1000000.0f / (float)period_us : 0.0f;
    float    duty_pct  = period_us ? 100.0f * (float)new_on_us / (float)period_us : 0.0f;

    Serial.printf(
      "[STROBE] raw(off,on)=(%4d,%4d)  EMA(off,on)=(%6.1f,%6.1f)  "
      "on=%u us  off=%u us  period=%u us  freq=%.2f Hz  duty=%.2f%%  (GPIO=%d)\n",
      raw_off, raw_on, ema_off, ema_on,
      new_on_us, new_off_us, period_us, freq_hz, duty_pct, STROBE_PIN
    );

    last_off_us = new_off_us;
    last_on_us  = new_on_us;
  }

  // --- 3) 画像を1コマだけ描画（非ブロッキング） ---
  static int frame_i = 1;
  char filename[32];
  sprintf(filename, "/%04d_240.bmp", frame_i);

  File f = SD.open(filename, FILE_READ);
  if (f) {
    size_t len = f.size();
    if (len > g_cap) {
      if (g_buf) { free(g_buf); g_buf = nullptr; g_cap = 0; }
      g_buf = (uint8_t*)ps_malloc(len);   // PSRAM確保
      if (!g_buf) {
        f.close();
        Serial.println("ps_malloc failed");
        delay(1);
        return;
      }
      g_cap = len;
    }

    f.read(g_buf, len);
    f.close();

    // 座標は要求どおり固定（x=0, y=40）
    M5.Display.drawBmp(g_buf, len, 0, 40);
  }

  // 次フレームへ
  frame_i++;
  if (frame_i > FRAME_COUNT) frame_i = 1;

  // フレーム間ディレイ
  if (FRAME_DELAY_MS > 0) delay(FRAME_DELAY_MS);
  else delay(1);  // 他タスクへ譲る
}
