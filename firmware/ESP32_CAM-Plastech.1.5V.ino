/* Edge Impulse Arduino examples
 * ESP32-CAM Plastic Detection + Servo + CoinSlot
 * Version: 2.1.0
 */

#include <Juzzoooo-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// Camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_AI_THINKER)
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
#else
#error "Camera model not selected"
#endif

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

#define PLASTIC_THRESHOLD 0.65f

// Servo configuration
#define SERVO_PIN               14
#define SERVO_CHANNEL           6
#define SERVO_PWM_FREQ          50
#define SERVO_PWM_RESOLUTION    16
#define SERVO_NEUTRAL_US        1500
#define SERVO_ROTATE_90_US      2000
#define SERVO_HOLD_TIME_MS      1500

// Flash
#define FLASH_LED_PIN 4

// Coin slot
#define COINSLOT_PIN 13
#define COINSLOT_PULSE_MS 500

static bool debug_nn = false;
static bool is_initialised = false;
static bool servo_active = false;
static uint32_t servo_start_time = 0;
static bool coin_active = false;
static uint32_t coin_start_time = 0;
static uint8_t plastic_streak = 0;
uint8_t *snapshot_buf;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static uint32_t servo_us_to_duty(uint32_t us) {
    return (us * ((1 << SERVO_PWM_RESOLUTION) - 1)) / 20000;
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("ESP32-CAM Plastic Detection + Servo + CoinSlot");

    if(!ei_camera_init()) {
        Serial.println("Failed to initialize camera!");
        while(1);
    }

    // Flash always on
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, HIGH);

    // Servo init
    ledcSetup(SERVO_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
    ledcWrite(SERVO_CHANNEL, servo_us_to_duty(SERVO_NEUTRAL_US));

    // Coin slot init
    pinMode(COINSLOT_PIN, OUTPUT);
    digitalWrite(COINSLOT_PIN, LOW);

    Serial.println("Camera initialized. System ready.");
}

void loop() {
    uint32_t now_ms = millis();

    // --- Coin pulse handling ---
    if(coin_active) {
        if(now_ms - coin_start_time >= COINSLOT_PULSE_MS) {
            digitalWrite(COINSLOT_PIN, LOW);
            coin_active = false;
        }
    }

    // --- Servo handling ---
    if(servo_active) {
        if(now_ms - servo_start_time >= SERVO_HOLD_TIME_MS) {
            ledcWrite(SERVO_CHANNEL, servo_us_to_duty(SERVO_NEUTRAL_US));
            servo_active = false;
        }
        delay(50);
        return; // skip scanning while servo is active
    }

    if(ei_sleep(5) != EI_IMPULSE_OK) return;

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if(!snapshot_buf) {
        Serial.println("ERR: Failed to allocate snapshot buffer!");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if(!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        Serial.println("Failed to capture image");
        free(snapshot_buf);
        return;
    }

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if(err != EI_IMPULSE_OK) {
        Serial.printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // --- PLASTIC / NON-PLASTIC DECISION ---
    float plastic_confidence = 0.0f;
    for(uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if(strcmp(ei_classifier_inferencing_categories[i], "Plastic") == 0 ||
           strcmp(ei_classifier_inferencing_categories[i], "plastic") == 0) {
            plastic_confidence = result.classification[i].value;
            break;
        }
    }

    if(plastic_confidence >= PLASTIC_THRESHOLD) {
        Serial.println("🟢 PLASTIC DETECTED");
        plastic_streak++;
        if(plastic_streak >= 5 && !servo_active) {
            Serial.println("🟢 PLASTIC CONFIRMED → SERVO ROTATE 90° + COINSLOT PULSE");

            // Rotate servo
            ledcWrite(SERVO_CHANNEL, servo_us_to_duty(SERVO_ROTATE_90_US));
            servo_active = true;
            servo_start_time = millis();
            plastic_streak = 0;

            // Coin slot pulse
            digitalWrite(COINSLOT_PIN, HIGH);
            coin_active = true;
            coin_start_time = millis();
        }
    } else {
        Serial.println("🔴 NON-PLASTIC DETECTED");
        plastic_streak = 0;
    }

    free(snapshot_buf);
}

// --- Camera functions ---
bool ei_camera_init(void) {
    if(is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if(err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if(s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if(err != ESP_OK) Serial.println("Camera deinit failed");
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if(!is_initialised) return false;

    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb) return false;

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if(!converted) return false;

    if(img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS || img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height
        );
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t out_ptr_ix = 0;
    while(length--) {
        out_ptr[out_ptr_ix++] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        pixel_ix += 3;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif