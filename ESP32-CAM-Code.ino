#include <Car_Detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// Camera model definition
#define CAMERA_MODEL_AI_THINKER

// Camera pin definitions for AI-Thinker ESP32-CAM
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

// Image buffer dimensions
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240
#define EI_CAMERA_FRAME_BYTE_SIZE        3

// Pin to signal Arduino when car is detected
#define CAR_SIGNAL_PIN                   12

// HC-SR04 Ultrasonic sensor pins
#define TRIG_PIN  13
#define ECHO_PIN  15

// Node identification
#define NODE_ID "ESP32_CAM_01"

// Power management settings
#define SLEEP_DURATION_SEC 10        // Deep sleep duration in seconds
#define LOW_POWER_THRESHOLD 30       // Distance threshold for low power mode (cm)
#define INFERENCE_INTERVAL_MS 5      // Normal inference interval
#define LOW_POWER_INTERVAL_MS 1000   // Low power mode inference interval

// Private variables
static bool is_initialised = false;
uint8_t *snapshot_buf;  // Points to the output of the capture
unsigned long system_uptime = 0;  // System uptime in milliseconds
bool low_power_mode = false;      // Low power mode flag

// Camera configuration
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
    .xclk_freq_hz = 20000000,  // 20MHz clock
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,  // JPEG format for efficient transfer
    .frame_size = FRAMESIZE_QVGA,    // QVGA resolution (320x240)
    .jpeg_quality = 12,              // 0-63, lower means higher quality
    .fb_count = 1,                   // Single frame buffer
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Function declarations
bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
float getDistance();

/**
 * Measure distance using HC-SR04 ultrasonic sensor
 * 
 * @return Distance in centimeters, or -1 if measurement failed
 */
float getDistance() {
    // Send trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read echo pulse duration
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
    
    // Calculate distance (speed of sound = 343 m/s or 0.0343 cm/µs)
    // Distance = (duration * 0.0343) / 2
    if (duration == 0) {
        return -1;  // Timeout or no echo
    }
    
    float distance = duration * 0.0343 / 2;
    return distance;
}

/**
 * Arduino setup function - runs once at startup
 */
void setup() {
    // Serial for debugging (USB)
    Serial.begin(115200);
    
    // Serial1 for communication with Arduino (TX/RX pins)
    // TX = GPIO 1, RX = GPIO 3
    Serial1.begin(115200, SERIAL_8N1, 3, 1);  // RX=3, TX=1
    
    while (!Serial);  // Wait for serial connection
    
    Serial.println("Edge Impulse Car Detection with Ultrasonic Sensor");
    Serial.println("Sending data to Arduino via TX pin (GPIO 1)");
    
    // Initialize camera
    if (!ei_camera_init()) {
        ei_printf("Failed to initialize Camera!\r\n");
    } else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continuous inference in 2 seconds...\n");
    ei_sleep(2000);

    // Setup output pin for car detection signal
    pinMode(CAR_SIGNAL_PIN, OUTPUT);
    digitalWrite(CAR_SIGNAL_PIN, LOW);
    
    // Setup ultrasonic sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

/**
 * Arduino main loop - runs continuously
 */
void loop() {
    // Adjust sleep interval based on power mode
    int sleep_interval = low_power_mode ? LOW_POWER_INTERVAL_MS : INFERENCE_INTERVAL_MS;
    
    if (ei_sleep(sleep_interval) != EI_IMPULSE_OK) {
        return;
    }
    
    // Update system uptime
    system_uptime = millis();
    
    // Get distance from ultrasonic sensor first (low power operation)
    float distance = getDistance();
    
    // Check if we should enter/exit low power mode
    if (distance > LOW_POWER_THRESHOLD && distance != -1) {
        if (!low_power_mode) {
            low_power_mode = true;
            Serial.println("{\"status\":\"Entering low power mode - no objects nearby\"}");
        }
        // In low power mode, skip heavy AI inference
        // Just send basic sensor data
        sendBasicJSON(distance, false);
        return;
    } else {
        if (low_power_mode) {
            low_power_mode = false;
            Serial.println("{\"status\":\"Exiting low power mode - object detected nearby\"}");
        }
    }

    // Allocate memory for image snapshot
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * 
                                    EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 
                                    EI_CAMERA_FRAME_BYTE_SIZE);

    // Check if allocation was successful
    if (snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    // Setup signal structure for Edge Impulse
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Capture image from camera
    if (!ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, 
                           (size_t)EI_CLASSIFIER_INPUT_HEIGHT, 
                           snapshot_buf)) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // Get distance from ultrasonic sensor
    // (We already got it earlier, but keeping this for compatibility)
    
    // Check system health
    bool camera_ok = is_initialised;
    bool memory_ok = (snapshot_buf != nullptr);
    bool ultrasonic_ok = (distance >= 0);
    size_t free_heap = ESP.getFreeHeap();
    
    // Check for car detections in bounding boxes
    bool car_detected = false;
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        
        // Check if this detection is a car
        if (strcmp(bb.label, "car") == 0) {
            car_detected = true;
        }
    }
    
    // Control signal pin based on detection
    if (car_detected) {
        digitalWrite(CAR_SIGNAL_PIN, HIGH);  // Send HIGH signal to Arduino
    } else {
        digitalWrite(CAR_SIGNAL_PIN, LOW);   // No car detected
    }
    
    // Send full JSON with inference results
    sendFullJSON(distance, car_detected, camera_ok, memory_ok, ultrasonic_ok, free_heap);

    // Free allocated memory
    free(snapshot_buf);
}

/**
 * Send basic JSON message (low power mode - no AI inference)
 */
void sendBasicJSON(float distance, bool car_detected) {
    // Send to Arduino via TX pin (Serial1)
    Serial1.print("{");
    Serial1.print("\"node_id\":\"");
    Serial1.print(NODE_ID);
    Serial1.print("\",");
    Serial1.print("\"timestamp\":");
    Serial1.print(system_uptime);
    Serial1.print(",");
    Serial1.print("\"sensors\":{");
    Serial1.print("\"distance_cm\":");
    Serial1.print(distance, 2);
    Serial1.print(",\"car_detected\":");
    Serial1.print(car_detected ? "true" : "false");
    Serial1.print(",\"signal_pin\":\"LOW\"");
    Serial1.print("},");
    Serial1.print("\"health\":{");
    Serial1.print("\"low_power_mode\":true,");
    Serial1.print("\"free_heap_bytes\":");
    Serial1.print(ESP.getFreeHeap());
    Serial1.print(",\"uptime_ms\":");
    Serial1.print(system_uptime);
    Serial1.print("}");
    Serial1.println("}");
    
    // Also print to debug Serial (USB)
    Serial.print("{");
    Serial.print("\"node_id\":\"");
    Serial.print(NODE_ID);
    Serial.print("\",");
    Serial.print("\"timestamp\":");
    Serial.print(system_uptime);
    Serial.print(",");
    Serial.print("\"sensors\":{");
    Serial.print("\"distance_cm\":");
    Serial.print(distance, 2);
    Serial.print(",\"car_detected\":");
    Serial.print(car_detected ? "true" : "false");
    Serial.print(",\"signal_pin\":\"LOW\"");
    Serial.print("},");
    Serial.print("\"health\":{");
    Serial.print("\"low_power_mode\":true,");
    Serial.print("\"free_heap_bytes\":");
    Serial.print(ESP.getFreeHeap());
    Serial.print(",\"uptime_ms\":");
    Serial.print(system_uptime);
    Serial.print("}");
    Serial.println("}");
}

/**
 * Send full JSON message with all sensor data and health info
 */
void sendFullJSON(float distance, bool car_detected, bool camera_ok, 
                  bool memory_ok, bool ultrasonic_ok, size_t free_heap) {
    // Send to Arduino via TX pin (Serial1)
    Serial1.print("{");
    Serial1.print("\"node_id\":\"");
    Serial1.print(NODE_ID);
    Serial1.print("\",");
    Serial1.print("\"timestamp\":");
    Serial1.print(system_uptime);
    Serial1.print(",");
    Serial1.print("\"sensors\":{");
    Serial1.print("\"distance_cm\":");
    Serial1.print(distance, 2);
    Serial1.print(",\"car_detected\":");
    Serial1.print(car_detected ? "true" : "false");
    Serial1.print(",\"signal_pin\":\"");
    Serial1.print(car_detected ? "HIGH" : "LOW");
    Serial1.print("\"},");
    Serial1.print("\"health\":{");
    Serial1.print("\"low_power_mode\":false,");
    Serial1.print("\"camera_initialized\":");
    Serial1.print(camera_ok ? "true" : "false");
    Serial1.print(",\"memory_allocated\":");
    Serial1.print(memory_ok ? "true" : "false");
    Serial1.print(",\"ultrasonic_ok\":");
    Serial1.print(ultrasonic_ok ? "true" : "false");
    Serial1.print(",\"free_heap_bytes\":");
    Serial1.print(free_heap);
    Serial1.print(",\"uptime_ms\":");
    Serial1.print(system_uptime);
    Serial1.print("}");
    Serial1.println("}");
    
    // Also print to debug Serial (USB)
    Serial.print("{");
    Serial.print("\"node_id\":\"");
    Serial.print(NODE_ID);
    Serial.print("\",");
    Serial.print("\"timestamp\":");
    Serial.print(system_uptime);
    Serial.print(",");
    Serial.print("\"sensors\":{");
    Serial.print("\"distance_cm\":");
    Serial.print(distance, 2);
    Serial.print(",\"car_detected\":");
    Serial.print(car_detected ? "true" : "false");
    Serial.print(",\"signal_pin\":\"");
    Serial.print(car_detected ? "HIGH" : "LOW");
    Serial.print("\"},");
    Serial.print("\"health\":{");
    Serial.print("\"low_power_mode\":false,");
    Serial.print("\"camera_initialized\":");
    Serial.print(camera_ok ? "true" : "false");
    Serial.print(",\"memory_allocated\":");
    Serial.print(memory_ok ? "true" : "false");
    Serial.print(",\"ultrasonic_ok\":");
    Serial.print(ultrasonic_ok ? "true" : "false");
    Serial.print(",\"free_heap_bytes\":");
    Serial.print(free_heap);
    Serial.print(",\"uptime_ms\":");
    Serial.print(system_uptime);
    Serial.print("}");
    Serial.println("}");
}

/**
 * Initialize camera sensor and start streaming
 * 
 * @return true if initialization successful, false otherwise
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    // Initialize the camera with config
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    // Get sensor handle for adjustments
    sensor_t *s = esp_camera_sensor_get();
    
    // OV3660 sensors are flipped and need adjustment
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);        // Flip it back
        s->set_brightness(s, 1);   // Increase brightness slightly
        s->set_saturation(s, 0);   // Lower saturation
    }

    is_initialised = true;
    return true;
}

/**
 * Capture, rescale and crop image from camera
 * 
 * @param img_width   Width of output image
 * @param img_height  Height of output image
 * @param out_buf     Pointer to store output image
 * @return true if capture successful, false otherwise
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    // Get frame buffer from camera
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    // Convert JPEG to RGB888 format
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);  // Return frame buffer to driver

    if (!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }

    // Resize if dimensions don't match
    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || 
        (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

/**
 * Get data callback for Edge Impulse signal
 * Converts RGB888 buffer to float format expected by classifier
 * 
 * @param offset   Pixel offset to start reading from
 * @param length   Number of pixels to read
 * @param out_ptr  Output array for pixel data
 * @return 0 on success
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    // Calculate byte offset (3 bytes per pixel in RGB888)
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB and pack into single float
        // (ESP32 camera returns BGR, we need RGB)
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + 
                              (snapshot_buf[pixel_ix + 1] << 8) + 
                              snapshot_buf[pixel_ix];
        
        // Move to next pixel
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    
    return 0;
}