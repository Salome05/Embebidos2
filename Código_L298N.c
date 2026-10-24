/*
 * LABORATORIO 2 — PARTES 1 A 5 — ESP-IDF (ESP32 / ESP32-S3)
 * Motor L298N + encoder en cuadratura (PCNT x4) + 2 botones + OLED SSD1306.
 *
 * VERSION CORREGIDA:
 *  - ENCODER_PPR = 500  (2000 cuentas/vuelta con x4)
 *  - PID con FEEDFORWARD calculado a partir del setpoint (zona muerta + SP/K),
 *    asi cada cambio de SP mueve el PWM de inmediato.
 *  - Kp y Ki mas altos, antiwindup tambien durante la rampa de subida.
 *  - PWM del L298N a 5 kHz (a 20 kHz el L298N pierde mucho torque).
 *  - SP inicial 4 rad/s, paso +2 rad/s (coincide con la pantalla).
 *
 * OLED 128x64 I2C SIN dependencias de terceros. Requiere ESP-IDF 5.3 o superior.
 * Cuatro modos: MONITOR -> CONFIG PWM -> CONFIG DIR -> VELOCIDAD (PID).
 *
 * AJUSTAR SEGUN TU MOTOR (ver bloque "CALIBRACION DEL PID"):
 *   PLANT_GAIN_RAD_S_PER_PCT y PWM_DEADBAND_PCT.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"


static const char *TAG = "LAB2_PID";


// ============================================================
// PINES Y CONSTANTES
// ============================================================

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define OLED_SCL_GPIO       GPIO_NUM_20
#define OLED_SDA_GPIO       GPIO_NUM_45
#define ENCODER_PIN_A       GPIO_NUM_18
#define ENCODER_PIN_B       GPIO_NUM_21
#define BTN_PWM             GPIO_NUM_6
#define BTN_MODE            GPIO_NUM_5
#elif defined(CONFIG_IDF_TARGET_ESP32)
#define OLED_SCL_GPIO       GPIO_NUM_20
#define OLED_SDA_GPIO       GPIO_NUM_45
#define ENCODER_PIN_A       GPIO_NUM_21
#define ENCODER_PIN_B       GPIO_NUM_18
#define BTN_PWM             GPIO_NUM_4
#define BTN_MODE            GPIO_NUM_5
#else
#error "Este proyecto esta preparado para ESP32 clasico o ESP32-S3"
#endif

// ---- OLED SSD1306 (I2C) ----
#define OLED_I2C_PORT       I2C_NUM_0
#define OLED_I2C_ADDR_1     0x3C
#define OLED_I2C_ADDR_2     0x3D

// ---- Motor (L298N): PWM en ENA, sentido con IN1/IN2 ----
#define MOTOR_PWM_GPIO      GPIO_NUM_16     // -> ENA
#define MOTOR_DIR_GPIO      GPIO_NUM_17     // -> IN1
#define MOTOR_DIR2_GPIO     GPIO_NUM_15     // -> IN2
#define MOTOR_PWM_FREQ_HZ   5000            // L298N: mejor a 1-5 kHz (a 20 kHz pierde torque)
#define MOTOR_DUTY_MAX      1023            // 10 bits: 0..1023

#define DIR_HOLD_US              1000000LL
#define DIR_MIN_OFF_US            800000LL
#define DIR_ZERO_STABLE_US        500000LL
#define DIR_ZERO_THRESHOLD_RAD_S    0.2f
#define DIR_ENCODER_FRESH_US      350000LL
#define DIR_RELEASE_DEBOUNCE_US    40000LL
#define MOTOR_RAMP_STEP_PERCENT       5
#define MOTOR_RAMP_PERIOD_US     100000LL

// Nivel de IN1 cuando el sentido es "CW". Si gira al reves, cambia a 0.
#define MOTOR_CW_IN1_LEVEL  1

// ---- Encoder ----
// 500 PPR por canal => 2000 cuentas/vuelta con decodificacion x4.
// Si el fabricante da 500 cuentas TOTALES por vuelta, usa 125.0f aqui.
#define ENCODER_PPR             500.0f
#define TOTAL_COUNTS_PER_REV    (ENCODER_PPR * 4.0f)
#define TWO_PI                  (2.0f * M_PI)
#define RPM_PER_RAD_S           (60.0f / TWO_PI)

// Si en CW ves rad/s negativos, pon -1 (o intercambia los cables A y B).
#define ENCODER_SIGN            1
#define ENCODER_USE_PULLUP      1

#define SAMPLE_PERIOD_US        100000
#define OLED_REFRESH_MS         200
#define OLED_RETRY_US           3000000LL

#define PCNT_HIGH_LIMIT         30000
#define PCNT_LOW_LIMIT         -30000

// ---- Botones ----
#define BTN_DEBOUNCE_US     35000LL
#define PID_HOLD_US         1000000LL      // en VELOCIDAD: mantener 1 s = ON/OFF

// ============================================================
// CALIBRACION DEL PID  (AJUSTA ESTOS DOS VALORES CON TU MOTOR)
// ============================================================
// Como medirlos: en CONFIG PWM pon 50 % y espera a que se estabilice.
// Lee los rad/s en pantalla (MONITOR). K = rad/s / (50 - DEADBAND).
//   Ej: 50 % da 10 rad/s y la zona muerta es 20 % -> K = 10 / 30 = 0.33
// PWM_DEADBAND_PCT: el PWM minimo al que el eje empieza a girar (sube
//   el PWM de a 10 % en CONFIG PWM y anota cuando arranca).
#define PLANT_GAIN_RAD_S_PER_PCT  0.30f
#define PWM_DEADBAND_PCT          20.0f

#define PID_KP              3.00f
#define PID_KI              3.00f
#define PID_KD              0.02f
#define PID_I_LIMIT        40.00f
#define PID_SP_MIN          2.00f
#define PID_SP_MAX         20.00f
#define PID_SP_STEP         2.00f
#define PID_SP_INITIAL      4.00f
#define PID_ENCODER_TIMEOUT_US 350000LL
#define PID_STALL_DUTY     70.00f
#define PID_STALL_SPEED     0.10f
#define PID_STALL_US      2500000LL
#define PID_OVERSPEED_US   400000LL
#define PID_MAX_PWM       100.00f


#define SPEED_QUEUE_LEN     10
#define BUTTON_QUEUE_LEN    10


// ============================================================
// TIPOS
// ============================================================

typedef struct
{
    float    speed_rad_s;
    int32_t  pulses;
    uint64_t timestamp_us;
} speed_msg_t;

typedef struct
{
    int      gpio;
    uint64_t t_us;
} button_event_t;

typedef enum
{
    UI_MONITOR = 0,
    UI_CFG_PWM,
    UI_CFG_DIR,
    UI_VELOCIDAD,
    UI_MODE_COUNT
} ui_mode_t;

typedef enum {
    PID_STOP = 0,
    PID_ON,
    PID_NO_ENCODER,
    PID_STALL,
    PID_OVERSPEED
} pid_status_t;

typedef enum {
    MOTOR_RUN = 0,
    MOTOR_RAMP,
    MOTOR_WAIT_STOP,
    MOTOR_CHANGED,
    MOTOR_HALTED
} motor_status_t;

typedef struct
{
    int       pwm_ref;
    float     pwm_applied;
    float     auto_target;
    float     setpoint;
    float     pid_error;
    bool      auto_running;
    pid_status_t pid_status;
    int       motor_dir;
    ui_mode_t mode;
    motor_status_t motor_status;
} ui_state_t;


// ============================================================
// VARIABLES GLOBALES
// ============================================================

static SemaphoreHandle_t g_state_mutex = NULL;
static ui_state_t g_ui = {
    .pwm_ref = 0, .pwm_applied = 0, .auto_target = 0,
    .setpoint = PID_SP_INITIAL, .auto_running = false,
    .pid_status = PID_STOP, .motor_dir = 0,
    .mode = UI_MONITOR, .motor_status = MOTOR_RUN
};
static speed_msg_t g_latest_speed = {0};

static pcnt_unit_handle_t g_pcnt_unit = NULL;
static int32_t g_last_pcnt_count = 0;
static int64_t g_last_sample_time_us = 0;
static esp_timer_handle_t g_speed_timer_handle = NULL;

static QueueHandle_t g_speed_queue  = NULL;
static QueueHandle_t g_button_queue = NULL;

static TaskHandle_t g_ui_task_handle      = NULL;
static TaskHandle_t g_display_task_handle = NULL;
static TaskHandle_t g_telemetry_task_handle = NULL;

typedef struct {
    gpio_num_t gpio;
    bool raw_down;
    bool stable_down;
    bool long_done;
    int64_t raw_since_us;
    int64_t pressed_since_us;
    ui_mode_t press_mode;
} btn_state_t;
static btn_state_t g_btns[2] = { {.gpio = BTN_MODE}, {.gpio = BTN_PWM} };

// Estado PID interno: solamente lo actualiza ui_task.
typedef struct {
    float integral;
    float last_pv;
    float deriv_filtered;
    uint64_t last_sample_us;
    int64_t stalled_since_us;
    int64_t overspeed_since_us;
} pid_rt_t;
static pid_rt_t g_pid = {0};


// ============================================================
// ESTADO COMPARTIDO (todo acceso pasa por el mutex)
// ============================================================

static ui_mode_t ui_mode_get(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    ui_mode_t m = g_ui.mode;
    xSemaphoreGive(g_state_mutex);
    return m;
}

static void ui_mode_set(ui_mode_t m)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.mode = m;
    xSemaphoreGive(g_state_mutex);
}

static int pwm_ref_step(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.pwm_ref += 10;
    if (g_ui.pwm_ref > 100)
        g_ui.pwm_ref = 0;
    int v = g_ui.pwm_ref;
    xSemaphoreGive(g_state_mutex);
    return v;
}

static void motor_dir_set(int d)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.motor_dir = d ? 1 : 0;
    xSemaphoreGive(g_state_mutex);
}

static void snapshot_get(ui_state_t *ui, speed_msg_t *speed)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    *ui = g_ui;
    *speed = g_latest_speed;
    xSemaphoreGive(g_state_mutex);
}

static void motor_status_set(motor_status_t status)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.motor_status = status;
    xSemaphoreGive(g_state_mutex);
}

static void oled_notify_change(void)
{
    if (g_display_task_handle != NULL) {
        xTaskNotifyGive(g_display_task_handle);
    }
}

// ============================================================
// PARTE 1 — MOTOR (LEDC + L298N)
// ============================================================

static void motor_apply_direction(int dir)
{
    int in1 = dir ? MOTOR_CW_IN1_LEVEL : !MOTOR_CW_IN1_LEVEL;
    gpio_set_level(MOTOR_DIR_GPIO,  in1);
    gpio_set_level(MOTOR_DIR2_GPIO, !in1);
}

static void motor_set_duty(float percent)
{
    if (!isfinite(percent)) percent = 0.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    uint32_t pwm = (uint32_t)lroundf(percent * MOTOR_DUTY_MAX / 100.0f);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num   = MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_DIR_GPIO) | (1ULL << MOTOR_DIR2_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    motor_set_duty(0);
    motor_apply_direction(g_ui.motor_dir);
}

static void motor_apply_pwm(float percent)
{
    if (!isfinite(percent)) percent = 0.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    motor_set_duty(percent);
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.pwm_applied = percent;
    xSemaphoreGive(g_state_mutex);
}

static void pid_stop(pid_status_t reason)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.auto_running = false;
    g_ui.auto_target = 0.0f;
    g_ui.pwm_ref = 0;
    g_ui.pid_status = reason;
    xSemaphoreGive(g_state_mutex);
    memset(&g_pid, 0, sizeof(g_pid));
    motor_apply_pwm(0.0f);
    motor_status_set(MOTOR_RUN);
    oled_notify_change();
}

static const char *pid_status_name(pid_status_t s)
{
    switch (s) {
    case PID_STOP: return "PID OFF";
    case PID_ON: return "PID ON";
    case PID_NO_ENCODER: return "SIN ENCODER";
    case PID_STALL: return "SIN GIRO - OFF";
    case PID_OVERSPEED: return "SOBREVEL. - OFF";
    default: return "?";
    }
}

// Estimacion de PWM necesario para un setpoint dado (feedforward).
static float pid_feedforward(float setpoint)
{
    float ff = PWM_DEADBAND_PCT + setpoint / PLANT_GAIN_RAD_S_PER_PCT;
    if (ff > PID_MAX_PWM) ff = PID_MAX_PWM;
    if (ff < 0.0f) ff = 0.0f;
    return ff;
}

static void pid_start(int64_t now)
{
    ui_state_t state;
    speed_msg_t speed;
    snapshot_get(&state, &speed);
    if (state.motor_status == MOTOR_WAIT_STOP || state.motor_status == MOTOR_CHANGED ||
        state.motor_status == MOTOR_HALTED) {
        ESP_LOGW(TAG, "PID no puede iniciar mientras hay cambio/bloqueo de direccion");
        return;
    }
    if (speed.timestamp_us == 0 || now - (int64_t)speed.timestamp_us > PID_ENCODER_TIMEOUT_US) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_ui.pid_status = PID_NO_ENCODER;
        xSemaphoreGive(g_state_mutex);
        ESP_LOGW(TAG, "PID no inicia: no hay muestras frescas del encoder");
        oled_notify_change();
        return;
    }
    memset(&g_pid, 0, sizeof(g_pid));
    g_pid.last_pv = fabsf(speed.speed_rad_s);
    g_pid.last_sample_us = speed.timestamp_us;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.auto_running = true;
    g_ui.pid_status = PID_ON;
    g_ui.auto_target = pid_feedforward(state.setpoint);  // arranca ya cerca del valor correcto
    g_ui.pid_error = state.setpoint - g_pid.last_pv;
    xSemaphoreGive(g_state_mutex);
    // Fuerza que la rampa empiece a subir hacia el objetivo
    motor_status_set(MOTOR_RAMP);
    ESP_LOGI(TAG, "PID ON SP=%.2f rad/s FF=%.1f%% Kp=%.2f Ki=%.2f Kd=%.3f",
             state.setpoint, pid_feedforward(state.setpoint), PID_KP, PID_KI, PID_KD);
    oled_notify_change();
}

// Un calculo PID por muestra nueva de encoder (~100 ms).
// Salida = feedforward(SP) + P + I + D. D sobre la medicion.
static void pid_tick(int64_t now)
{
    ui_state_t state;
    speed_msg_t speed;
    snapshot_get(&state, &speed);
    if (!state.auto_running) return;
    if (speed.timestamp_us == 0 || now < (int64_t)speed.timestamp_us ||
        now - (int64_t)speed.timestamp_us > PID_ENCODER_TIMEOUT_US) {
        ESP_LOGE(TAG, "PID OFF: encoder no entrega datos recientes");
        pid_stop(PID_NO_ENCODER);
        return;
    }
    if (speed.timestamp_us == g_pid.last_sample_us) return;
    float dt = ((float)((int64_t)speed.timestamp_us - (int64_t)g_pid.last_sample_us)) * 1e-6f;
    if (dt <= 0.0f || dt > 0.35f) {
        pid_stop(PID_NO_ENCODER);
        return;
    }
    g_pid.last_sample_us = speed.timestamp_us;
    float pv = fabsf(speed.speed_rad_s);
    float error = state.setpoint - pv;
    float deriv = (pv - g_pid.last_pv) / dt;
    g_pid.last_pv = pv;
    g_pid.deriv_filtered = 0.75f * g_pid.deriv_filtered + 0.25f * deriv;

    float ff = pid_feedforward(state.setpoint);
    float p = PID_KP * error;
    float d = -PID_KD * g_pid.deriv_filtered;
    float without_int = ff + p + d;

    float proposed = g_pid.integral + PID_KI * dt * error;
    if (proposed > PID_I_LIMIT) proposed = PID_I_LIMIT;
    if (proposed < -PID_I_LIMIT) proposed = -PID_I_LIMIT;

    // Antiwindup: no integrar si la salida esta saturada, ni mientras la
    // rampa de subida todavia no alcanza el objetivo anterior.
    bool sat_hi  = (without_int + proposed > PID_MAX_PWM) && error > 0.0f;
    bool sat_lo  = (without_int + proposed < 0.0f) && error < 0.0f;
    bool ramping = (state.pwm_applied < state.auto_target - 2.0f) && error > 0.0f;
    if (!sat_hi && !sat_lo && !ramping) {
        g_pid.integral = proposed;
    }

    float target = without_int + g_pid.integral;
    if (target > PID_MAX_PWM) target = PID_MAX_PWM;
    if (target < 0.0f) target = 0.0f;
    if (!isfinite(target)) {
        pid_stop(PID_NO_ENCODER);
        return;
    }

    if (state.pwm_applied >= PID_STALL_DUTY && pv < PID_STALL_SPEED) {
        if (g_pid.stalled_since_us == 0) g_pid.stalled_since_us = now;
        if (now - g_pid.stalled_since_us >= PID_STALL_US) {
            ESP_LOGE(TAG, "PID OFF: PWM alto con motor sin giro medible");
            pid_stop(PID_STALL);
            return;
        }
    } else g_pid.stalled_since_us = 0;

    if (pv > fmaxf(state.setpoint * 1.7f, state.setpoint + 6.0f)) {
        if (g_pid.overspeed_since_us == 0) g_pid.overspeed_since_us = now;
        if (now - g_pid.overspeed_since_us >= PID_OVERSPEED_US) {
            ESP_LOGE(TAG, "PID OFF: sobrevelocidad, SP=%.2f PV=%.2f", state.setpoint, pv);
            pid_stop(PID_OVERSPEED);
            return;
        }
    } else g_pid.overspeed_since_us = 0;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_ui.auto_target = target;
    g_ui.pid_error = error;
    // Si la rampa ya habia terminado y el nuevo objetivo es mayor, vuelve a rampa.
    if (g_ui.motor_status == MOTOR_RUN) g_ui.motor_status = MOTOR_RAMP;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGD(TAG, "PID SP=%.2f PV=%.2f e=%.2f FF=%.1f I=%.1f PWM=%.2f",
             state.setpoint, pv, error, ff, g_pid.integral, target);
}

typedef struct {
    bool holding;
    bool reversed;
    int64_t started_us;
    int64_t stopped_since_us;
    int64_t released_since_us;
    int64_t last_ramp_us;
} motor_control_t;

static motor_control_t g_motor_control = {0};

static void direction_button_pressed(int64_t now)
{
    if (ui_mode_get() != UI_CFG_DIR) return;
    motor_apply_pwm(0);
    g_motor_control.holding = true;
    g_motor_control.reversed = false;
    g_motor_control.started_us = now;
    g_motor_control.stopped_since_us = 0;
    g_motor_control.released_since_us = 0;
    motor_status_set(MOTOR_WAIT_STOP);
    ESP_LOGI(TAG, "CAMBIO DIR: PWM=0, mantenga boton >=1 s; esperando paro");
    oled_notify_change();
}

static void motor_control_tick(int64_t now)
{
    if (g_motor_control.holding) {
        bool pressed = (gpio_get_level(BTN_PWM) == 0);
        if (!pressed) {
            if (g_motor_control.released_since_us == 0)
                g_motor_control.released_since_us = now;
            if (now - g_motor_control.released_since_us >= DIR_RELEASE_DEBOUNCE_US) {
                bool was_reversed = g_motor_control.reversed;
                bool was_short = (now - g_motor_control.started_us < DIR_HOLD_US + DIR_RELEASE_DEBOUNCE_US);
                g_motor_control.holding = false;
                g_motor_control.released_since_us = 0;
                if (was_reversed || was_short) {
                    motor_status_set(MOTOR_RAMP);
                    g_motor_control.last_ramp_us = now - MOTOR_RAMP_PERIOD_US;
                    ESP_LOGI(TAG, "BTN DIR soltado: %s. Rampa de arranque",
                             was_reversed ? "cambio confirmado" : "toque corto, sin cambio");
                } else {
                    motor_status_set(MOTOR_HALTED);
                    ESP_LOGW(TAG, "CAMBIO DIR CANCELADO: no se confirmo velocidad cercana a cero. Motor detenido; configurar PWM para reiniciar");
                }
                oled_notify_change();
            }
        } else {
            g_motor_control.released_since_us = 0;
            if (!g_motor_control.reversed) {
                speed_msg_t speed;
                ui_state_t state;
                snapshot_get(&state, &speed);
                bool recent = speed.timestamp_us != 0 &&
                              now >= (int64_t)speed.timestamp_us &&
                              now - (int64_t)speed.timestamp_us <= DIR_ENCODER_FRESH_US;
                bool still = recent && fabsf(speed.speed_rad_s) <= DIR_ZERO_THRESHOLD_RAD_S;
                if (still) {
                    if (g_motor_control.stopped_since_us == 0)
                        g_motor_control.stopped_since_us = now;
                } else {
                    g_motor_control.stopped_since_us = 0;
                }
                if (now - g_motor_control.started_us >= DIR_HOLD_US &&
                    now - g_motor_control.started_us >= DIR_MIN_OFF_US &&
                    g_motor_control.stopped_since_us != 0 &&
                    now - g_motor_control.stopped_since_us >= DIR_ZERO_STABLE_US) {
                    int next_dir = !state.motor_dir;
                    motor_apply_direction(next_dir);
                    motor_dir_set(next_dir);
                    g_motor_control.reversed = true;
                    motor_status_set(MOTOR_CHANGED);
                    ESP_LOGI(TAG, "CAMBIO DIR CONFIRMADO: %s. Soltar para acelerar con rampa",
                             next_dir ? "CW" : "CCW");
                    oled_notify_change();
                }
            }
        }
        return;
    }

    ui_state_t state;
    speed_msg_t speed;
    snapshot_get(&state, &speed);
    if (state.motor_status == MOTOR_RAMP || state.motor_status == MOTOR_RUN) {
        float target = state.auto_running ? state.auto_target : (float)state.pwm_ref;
        if (target <= 0.0f) {
            if (state.pwm_applied > 0.01f) motor_apply_pwm(0);
            motor_status_set(MOTOR_RUN);
            return;
        }
        if (fabsf(state.pwm_applied - target) >= 0.05f &&
            now - g_motor_control.last_ramp_us >= MOTOR_RAMP_PERIOD_US) {
            float next = state.pwm_applied;
            if (next < target) {
                next += MOTOR_RAMP_STEP_PERCENT;
                if (next > target) next = target;
            } else {
                next = target;
            }
            motor_apply_pwm(next);
            g_motor_control.last_ramp_us = now;
            // Con PID activo se queda en RAMP para seguir al objetivo; en manual vuelve a RUN.
            if (!state.auto_running && fabsf(next - target) < 0.05f)
                motor_status_set(MOTOR_RUN);
        }
    }
}

// ============================================================
// PARTE 2 — ENCODER (PCNT x4 en hardware) + TIMER DE 100 ms
// ============================================================

static void timer_callback(void *arg)
{
    (void)arg;
    int count = 0;
    if (pcnt_unit_get_count(g_pcnt_unit, &count) != ESP_OK) {
        return;
    }

    int64_t now = esp_timer_get_time();
    int64_t delta_counts = (int64_t)count - (int64_t)g_last_pcnt_count;
    int64_t dt_us = now - g_last_sample_time_us;
    g_last_pcnt_count = count;
    g_last_sample_time_us = now;

    if (dt_us <= 0) return;
    int32_t pulses = (int32_t)(delta_counts * ENCODER_SIGN);
    float speed = ((float)pulses / TOTAL_COUNTS_PER_REV) * TWO_PI *
                  (1000000.0f / (float)dt_us);

    speed_msg_t msg = {
        .pulses = pulses,
        .speed_rad_s = speed,
        .timestamp_us = (uint64_t)now
    };
    (void)xQueueSend(g_speed_queue, &msg, 0);
}

static void init_encoder_pcnt(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_PIN_A) | (1ULL << ENCODER_PIN_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = ENCODER_USE_PULLUP ? GPIO_PULLUP_ENABLE    : GPIO_PULLUP_DISABLE,
        .pull_down_en = ENCODER_USE_PULLUP ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &g_pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num  = ENCODER_PIN_A,
        .level_gpio_num = ENCODER_PIN_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num  = ENCODER_PIN_B,
        .level_gpio_num = ENCODER_PIN_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(g_pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(g_pcnt_unit, PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_unit));
    g_last_pcnt_count = 0;
    g_last_sample_time_us = esp_timer_get_time();
}

static void init_speed_timer(void)
{
    g_speed_queue = xQueueCreate(SPEED_QUEUE_LEN, sizeof(speed_msg_t));
    if (g_speed_queue == NULL)
    {
        ESP_LOGE(TAG, "No se pudo crear la cola de velocidad");
        abort();
    }

    const esp_timer_create_args_t timer_args = {
        .callback        = &timer_callback,
        .arg             = NULL,
        .name            = "encoder_speed_timer",
        .dispatch_method = ESP_TIMER_TASK
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &g_speed_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_speed_timer_handle, SAMPLE_PERIOD_US));
}


// ============================================================
// PARTE 3 — BOTONES (ISR minima + debounce fuera de la ISR)
// ============================================================

static void IRAM_ATTR button_isr(void *arg)
{
    button_event_t ev = {
        .gpio = (int)(intptr_t)arg,
        .t_us = esp_timer_get_time()
    };

    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(g_button_queue, &ev, &high_task_wakeup);

    if (high_task_wakeup == pdTRUE)
        portYIELD_FROM_ISR();
}

static void init_uibtn_gpios(void)
{
    g_button_queue = xQueueCreate(BUTTON_QUEUE_LEN, sizeof(button_event_t));
    if (g_button_queue == NULL)
    {
        ESP_LOGE(TAG, "No se pudo crear la cola de botones");
        abort();
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_MODE) | (1ULL << BTN_PWM),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_MODE, button_isr, (void *)(intptr_t)BTN_MODE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_PWM,  button_isr, (void *)(intptr_t)BTN_PWM));
}


// ============================================================
// PARTE 4 — ui_task: maquina de estados + debounce
// ============================================================

static void telemetry_task(void *arg)
{
    (void)arg;
    speed_msg_t msg;
    int64_t last_log_us = 0;
    while (1) {
        if (xQueueReceive(g_speed_queue, &msg, portMAX_DELAY) != pdTRUE) continue;
        speed_msg_t newer;
        while (xQueueReceive(g_speed_queue, &newer, 0) == pdTRUE) msg = newer;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_latest_speed = msg;
        ui_state_t state = g_ui;
        xSemaphoreGive(g_state_mutex);
        if ((int64_t)msg.timestamp_us - last_log_us >= 500000LL) {
            ESP_LOGI(TAG, "SERIE: pulsos=%ld rad/s=%.3f RPM=%.1f PWMref=%d%% PWMreal=%.1f%% DIR=%s modo=%d PID=%d SP=%.1f",
                     (long)msg.pulses, msg.speed_rad_s,
                     msg.speed_rad_s * RPM_PER_RAD_S,
                     state.pwm_ref, state.pwm_applied,
                     state.motor_dir ? "CW" : "CCW", (int)state.mode,
                     (int)state.auto_running, state.setpoint);
            last_log_us = (int64_t)msg.timestamp_us;
        }
    }
}

static void button_press(btn_state_t *b, int64_t now)
{
    b->pressed_since_us = now;
    b->long_done = false;
    b->press_mode = ui_mode_get();
    if (b->gpio == BTN_MODE) {
        if (g_motor_control.holding) return;
        ui_mode_t next = (ui_mode_t)((b->press_mode + 1) % UI_MODE_COUNT);
        if (next == UI_CFG_PWM || next == UI_CFG_DIR) {
            ui_state_t state;
            speed_msg_t speed;
            snapshot_get(&state, &speed);
            if (state.auto_running) {
                pid_stop(PID_STOP);
                ESP_LOGI(TAG, "Cambio a modo manual: PID apagado y PWM=0");
            }
        }
        ui_mode_set(next);
        ESP_LOGI(TAG, "MODO -> %d", (int)next);
        oled_notify_change();
    } else if (b->gpio == BTN_PWM && b->press_mode == UI_CFG_DIR) {
        direction_button_pressed(now);
    }
}

static void button_release(btn_state_t *b, int64_t now)
{
    (void)now;
    if (b->gpio != BTN_PWM || b->long_done || b->press_mode != ui_mode_get()) return;
    if (b->press_mode == UI_CFG_PWM) {
        int ref = pwm_ref_step();
        motor_status_set(ref == 0 ? MOTOR_RUN : MOTOR_RAMP);
        if (ref == 0) motor_apply_pwm(0);
        ESP_LOGI(TAG, "PWM MANUAL -> %d%%", ref);
        oled_notify_change();
    } else if (b->press_mode == UI_VELOCIDAD) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        float sp = g_ui.setpoint + PID_SP_STEP;
        if (sp > PID_SP_MAX + 0.01f) sp = PID_SP_MIN;
        g_ui.setpoint = sp;
        xSemaphoreGive(g_state_mutex);
        ESP_LOGI(TAG, "SETPOINT -> %.1f rad/s", sp);
        oled_notify_change();
    }
}

static void buttons_tick(int64_t now)
{
    for (size_t i=0; i<2; i++) {
        btn_state_t *b = &g_btns[i];
        bool down = gpio_get_level(b->gpio) == 0;
        if (down != b->raw_down) {
            b->raw_down = down;
            b->raw_since_us = now;
        }
        if (down != b->stable_down && now - b->raw_since_us >= BTN_DEBOUNCE_US) {
            b->stable_down = down;
            if (down) button_press(b, now);
            else button_release(b, now);
        }
        if (b->gpio == BTN_PWM && b->stable_down && down && !b->long_done &&
            b->press_mode == UI_VELOCIDAD && ui_mode_get() == UI_VELOCIDAD &&
            now - b->pressed_since_us >= PID_HOLD_US) {
            b->long_done = true;
            ui_state_t state;
            speed_msg_t speed;
            snapshot_get(&state, &speed);
            if (state.auto_running) {
                pid_stop(PID_STOP);
                ESP_LOGI(TAG, "PID detenido por pulsacion larga");
            } else pid_start(now);
        }
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    for (size_t i=0;i<2;i++) {
        g_btns[i].raw_down = (gpio_get_level(g_btns[i].gpio) == 0);
        g_btns[i].stable_down = g_btns[i].raw_down;
        g_btns[i].raw_since_us = esp_timer_get_time();
        g_btns[i].pressed_since_us = esp_timer_get_time();
        g_btns[i].long_done = true;
    }
    button_event_t ev;
    while (1) {
        (void)xQueueReceive(g_button_queue, &ev, pdMS_TO_TICKS(10));
        int64_t now = esp_timer_get_time();
        buttons_tick(now);
        pid_tick(now);
        motor_control_tick(now);
    }
}

// ============================================================
// PARTE 5 — OLED SSD1306 (I2C) + display_task
// ============================================================

#define OLED_W              128
#define OLED_H              64
#define OLED_PAGES          (OLED_H / 8)
#define OLED_ROTATE_180     0
#define OLED_IS_SH1106       0

static i2c_master_bus_handle_t g_oled_bus = NULL;
static i2c_master_dev_handle_t g_oled_dev = NULL;
static uint8_t g_oled_addr = 0;
static bool g_oled_ready = false;
static uint8_t g_fb[OLED_W * OLED_PAGES];

static const uint8_t FONT5X7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x14,0x08,0x3E,0x08,0x14}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x41,0x22,0x14,0x08}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x32,0x49,0x79,0x41,0x3E}, // @
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x7F,0x41,0x41,0x00}, // [
    {0x02,0x04,0x08,0x10,0x20}, // backslash
    {0x00,0x41,0x41,0x7F,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04}, // ^
    {0x40,0x40,0x40,0x40,0x40}, // _
    {0x00,0x01,0x02,0x04,0x00}, // `
    {0x20,0x54,0x54,0x54,0x78}, // a
    {0x7F,0x48,0x44,0x44,0x38}, // b
    {0x38,0x44,0x44,0x44,0x20}, // c
    {0x38,0x44,0x44,0x48,0x7F}, // d
    {0x38,0x54,0x54,0x54,0x18}, // e
    {0x08,0x7E,0x09,0x01,0x02}, // f
    {0x0C,0x52,0x52,0x52,0x3E}, // g
    {0x7F,0x08,0x04,0x04,0x78}, // h
    {0x00,0x44,0x7D,0x40,0x00}, // i
    {0x20,0x40,0x44,0x3D,0x00}, // j
    {0x7F,0x10,0x28,0x44,0x00}, // k
    {0x00,0x41,0x7F,0x40,0x00}, // l
    {0x7C,0x04,0x18,0x04,0x78}, // m
    {0x7C,0x08,0x04,0x04,0x78}, // n
    {0x38,0x44,0x44,0x44,0x38}, // o
    {0x7C,0x14,0x14,0x14,0x08}, // p
    {0x08,0x14,0x14,0x18,0x7C}, // q
    {0x7C,0x08,0x04,0x04,0x08}, // r
    {0x48,0x54,0x54,0x54,0x20}, // s
    {0x04,0x3F,0x44,0x40,0x20}, // t
    {0x3C,0x40,0x40,0x20,0x7C}, // u
    {0x1C,0x20,0x40,0x20,0x1C}, // v
    {0x3C,0x40,0x30,0x40,0x3C}, // w
    {0x44,0x28,0x10,0x28,0x44}, // x
    {0x0C,0x50,0x50,0x50,0x3C}, // y
    {0x44,0x64,0x54,0x4C,0x44}, // z
    {0x00,0x08,0x36,0x41,0x00}, // {
    {0x00,0x00,0x7F,0x00,0x00}, // |
    {0x00,0x41,0x36,0x08,0x00}, // }
    {0x08,0x04,0x08,0x10,0x08}, // ~
};

static esp_err_t oled_send(uint8_t control, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + OLED_W];
    if (g_oled_dev == NULL || len > OLED_W) return ESP_ERR_INVALID_STATE;
    buf[0] = control;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(g_oled_dev, buf, len + 1, 150);
}

static esp_err_t oled_hw_init(void)
{
#if OLED_IS_SH1106
    const uint8_t init[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0xAD, 0x8B,
        OLED_ROTATE_180 ? 0xA0 : 0xA1,
        OLED_ROTATE_180 ? 0xC0 : 0xC8,
        0xDA, 0x12, 0x81, 0x80, 0xD9, 0x22, 0xDB, 0x35,
        0xA4, 0xA6, 0xAF
    };
#else
    const uint8_t init[] = {
        0xAE, 0xD5, 0x80, 0xA8, OLED_H - 1, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x02,
        OLED_ROTATE_180 ? 0xA0 : 0xA1,
        OLED_ROTATE_180 ? 0xC0 : 0xC8,
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
#endif
    return oled_send(0x00, init, sizeof init);
}

static void oled_clear(void)
{
    memset(g_fb, 0, sizeof g_fb);
}

static esp_err_t oled_display(void)
{
    if (!g_oled_ready) return ESP_ERR_INVALID_STATE;
    for (int page = 0; page < OLED_PAGES; page++) {
#if OLED_IS_SH1106
        const uint8_t set_page[] = { (uint8_t)(0xB0 + page), 0x02, 0x10 };
#else
        const uint8_t set_page[] = { (uint8_t)(0xB0 + page), 0x00, 0x10 };
#endif
        esp_err_t err = oled_send(0x00, set_page, sizeof set_page);
        if (err != ESP_OK) return err;
        err = oled_send(0x40, &g_fb[page * OLED_W], OLED_W);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void oled_pixel(int x, int y)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H)
        return;
    g_fb[(y / 8) * OLED_W + x] |= (uint8_t)(1u << (y % 8));
}

static void oled_draw_char(int x, int y, char c, int scale)
{
    if (c < 32 || c > 126)
        c = '?';
    const uint8_t *glyph = FONT5X7[c - 32];

    for (int col = 0; col < 5; col++)
    {
        for (int row = 0; row < 7; row++)
        {
            if (glyph[col] & (1u << row))
            {
                for (int dx = 0; dx < scale; dx++)
                    for (int dy = 0; dy < scale; dy++)
                        oled_pixel(x + col * scale + dx, y + row * scale + dy);
            }
        }
    }
}

static void oled_draw_text_scaled(int x, int y, const char *s, int scale)
{
    for (; *s; s++)
    {
        oled_draw_char(x, y, *s, scale);
        x += 6 * scale;
    }
}

static void oled_draw_text(int x, int y, const char *s)
{
    oled_draw_text_scaled(x, y, s, 1);
}

static void oled_draw_rect(int x, int y, int w, int h, bool fill)
{
    for (int i = 0; i < w; i++)
        for (int j = 0; j < h; j++)
            if (fill || i == 0 || j == 0 || i == w - 1 || j == h - 1)
                oled_pixel(x + i, y + j);
}

static bool oled_try_connect(void)
{
    if (g_oled_bus == NULL) return false;
    uint8_t addr = 0;
    if (i2c_master_probe(g_oled_bus, OLED_I2C_ADDR_1, 100) == ESP_OK)
        addr = OLED_I2C_ADDR_1;
    else if (i2c_master_probe(g_oled_bus, OLED_I2C_ADDR_2, 100) == ESP_OK)
        addr = OLED_I2C_ADDR_2;
    if (addr == 0) return false;

    if (g_oled_dev != NULL && addr != g_oled_addr) {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(g_oled_dev));
        g_oled_dev = NULL;
    }
    if (g_oled_dev == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(g_oled_bus, &dev_cfg, &g_oled_dev) != ESP_OK)
            return false;
    }
    g_oled_addr = addr;
    if (oled_hw_init() != ESP_OK) return false;
    g_oled_ready = true;
    oled_clear();
    oled_draw_text(0, 0, "OLED CONECTADA");
    oled_draw_text(0, 14, "Actualizacion activa");
    if (oled_display() != ESP_OK) {
        g_oled_ready = false;
        return false;
    }
    ESP_LOGI(TAG, "OLED conectada 0x%02X (%s)", addr,
             OLED_IS_SH1106 ? "SH1106" : "SSD1306");
    return true;
}

static void oled_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = OLED_I2C_PORT,
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &g_oled_bus));
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!oled_try_connect())
        ESP_LOGW(TAG, "OLED sin respuesta; se intentara reconectar cada 3 s");
}

static const char *motor_status_name(motor_status_t status)
{
    switch (status) {
    case MOTOR_RUN:       return "MARCHA";
    case MOTOR_RAMP:      return "ACELERANDO";
    case MOTOR_WAIT_STOP: return "ESPERANDO PARO";
    case MOTOR_CHANGED:   return "DIR CAMBIADA";
    case MOTOR_HALTED:    return "PARADO SEGURIDAD";
    default:              return "?";
    }
}

static void draw_monitor(const speed_msg_t *m, const ui_state_t *s, uint32_t frame, bool stale)
{
    char line[40];
    snprintf(line, sizeof line, "MONITOR     F:%04lu", (unsigned long)(frame % 10000));
    oled_draw_text(0, 0, line);
    snprintf(line, sizeof line, "%.2f", m->speed_rad_s);
    int n = (int)strlen(line);
    if (n <= 7) {
        oled_draw_text_scaled(0, 12, line, 2);
        oled_draw_text(88, 22, "rad/s");
    } else {
        oled_draw_text(0, 16, line);
        oled_draw_text(0, 25, "rad/s");
    }
    if (stale) snprintf(line, sizeof line, "ENCODER SIN DATOS");
    else snprintf(line, sizeof line, "RPM: %.1f", m->speed_rad_s * RPM_PER_RAD_S);
    oled_draw_text(0, 35, line);
    snprintf(line, sizeof line, "REF:%d%% ACT:%.0f%%", s->pwm_ref, s->pwm_applied);
    oled_draw_text(0, 45, line);
    snprintf(line, sizeof line, "%s  %s", s->motor_dir ? "CW" : "CCW",
             s->auto_running ? "PID ACTIVO" : motor_status_name(s->motor_status));
    oled_draw_text(0, 55, line);
}

static void draw_cfg_pwm(const speed_msg_t *m, const ui_state_t *s, uint32_t frame, bool stale)
{
    char line[40];
    snprintf(line, sizeof line, "CONFIG PWM  F:%04lu", (unsigned long)(frame % 10000));
    oled_draw_text(0, 0, line);
    snprintf(line, sizeof line, "%d%%", s->pwm_ref);
    oled_draw_text_scaled(0, 11, line, 2);
    snprintf(line, sizeof line, "REAL:%.0f%%", s->pwm_applied);
    oled_draw_text(65, 22, line);
    oled_draw_rect(0, 32, 128, 8, false);
    int width = (126 * s->pwm_ref) / 100;
    if (width > 0) oled_draw_rect(1, 33, width, 6, true);
    if (stale) snprintf(line, sizeof line, "ENCODER SIN DATOS");
    else snprintf(line, sizeof line, "v=%.2f rad/s", m->speed_rad_s);
    oled_draw_text(0, 43, line);
    oled_draw_text(0, 55, "PWM:+10  MODE:SIG");
}

static void draw_cfg_dir(const speed_msg_t *m, const ui_state_t *s, uint32_t frame, bool stale)
{
    char line[40];
    snprintf(line, sizeof line, "CONFIG DIR  F:%04lu", (unsigned long)(frame % 10000));
    oled_draw_text(0, 0, line);
    oled_draw_text_scaled(0, 11, s->motor_dir ? "CW" : "CCW", 2);
    if (s->motor_status == MOTOR_WAIT_STOP)
        oled_draw_text(0, 31, "MANTEN - ESPERA PARO");
    else if (s->motor_status == MOTOR_CHANGED)
        oled_draw_text(0, 31, "CAMBIO OK - SUELTA");
    else if (s->motor_status == MOTOR_HALTED)
        oled_draw_text(0, 31, "SIN PARO: MOTOR OFF");
    else
        oled_draw_text(0, 31, "MANTEN PWM 1 SEG");
    if (stale) snprintf(line, sizeof line, "ENCODER SIN DATOS");
    else snprintf(line, sizeof line, "v=%.2f rad/s", m->speed_rad_s);
    oled_draw_text(0, 43, line);
    snprintf(line, sizeof line, "PWM real:%.0f%%", s->pwm_applied);
    oled_draw_text(0, 55, line);
}

static void draw_velocidad(const speed_msg_t *m, const ui_state_t *s, uint32_t frame, bool stale)
{
    char line[40];
    snprintf(line, sizeof line, "VELOCIDAD  F:%04lu", (unsigned long)(frame % 10000));
    oled_draw_text(0, 0, line);
    snprintf(line, sizeof line, "SP:%.1f rad/s", s->setpoint);
    oled_draw_text(0, 12, line);
    if (stale) oled_draw_text(0, 23, "PV: SIN DATOS");
    else {
        snprintf(line, sizeof line, "PV:%.2f rad/s", fabsf(m->speed_rad_s));
        oled_draw_text(0, 23, line);
    }
    snprintf(line, sizeof line, "PWM:%.0f%% e:%+.1f", s->pwm_applied, s->pid_error);
    oled_draw_text(0, 34, line);
    oled_draw_text(0, 45, pid_status_name(s->pid_status));
    oled_draw_text(0, 55, s->auto_running ? "TOQUE:+2 MANTEN:OFF" : "TOQUE:+2 MANTEN:ON");
}

static void display_task(void *arg)
{
    (void)arg;
    uint32_t frame = 0;
    int64_t last_retry_us = 0;
    int64_t last_report_us = 0;
    while (1) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(OLED_REFRESH_MS));
        int64_t now = esp_timer_get_time();
        if (!g_oled_ready) {
            if (now - last_retry_us >= OLED_RETRY_US) {
                last_retry_us = now;
                if (!oled_try_connect())
                    ESP_LOGW(TAG, "OLED: reconexion fallida (0x3C/0x3D)");
            }
            continue;
        }
        ui_state_t state;
        speed_msg_t speed;
        snapshot_get(&state, &speed);
        bool speed_stale = speed.timestamp_us == 0 ||
                           now < (int64_t)speed.timestamp_us ||
                           now - (int64_t)speed.timestamp_us > 700000LL;
        if (speed_stale) speed.speed_rad_s = 0;
        oled_clear();
        switch (state.mode) {
        case UI_MONITOR: draw_monitor(&speed, &state, frame, speed_stale); break;
        case UI_CFG_PWM: draw_cfg_pwm(&speed, &state, frame, speed_stale); break;
        case UI_CFG_DIR: draw_cfg_dir(&speed, &state, frame, speed_stale); break;
        case UI_VELOCIDAD: draw_velocidad(&speed, &state, frame, speed_stale); break;
        default: break;
        }
        esp_err_t err = oled_display();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OLED I2C: fallo al refrescar: %s. Reconectando...",
                     esp_err_to_name(err));
            g_oled_ready = false;
            last_retry_us = now;
        } else {
            frame++;
            if (now - last_report_us >= 2000000LL) {
                ESP_LOGI(TAG, "OLED frame=%lu modo=%d v=%.2f pwm=%.1f/%d PID=%d",
                         (unsigned long)frame, (int)state.mode,
                         speed.speed_rad_s, state.pwm_applied, state.pwm_ref,
                         (int)state.auto_running);
                last_report_us = now;
            }
        }
    }
}


// ============================================================
// APP MAIN
// ============================================================

void app_main(void)
{
    ESP_LOGI(TAG, "LAB2: 3 modos originales + velocidad PID con feedforward");
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) abort();

    motor_init();
    init_encoder_pcnt();
    oled_init();
    init_speed_timer();

    if (xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 3,
                    &g_telemetry_task_handle) != pdPASS) abort();
    if (xTaskCreate(display_task, "display", 5120, NULL, 2,
                    &g_display_task_handle) != pdPASS) abort();
    init_uibtn_gpios();
    if (xTaskCreate(ui_task, "ui", 4096, NULL, 4,
                    &g_ui_task_handle) != pdPASS) abort();

    ESP_LOGI(TAG, "BTN MODE: 4 pantallas. PWM: corto=+10%% manual o +2 rad/s, largo=DIR/PID");
    ESP_LOGW(TAG, "IMPORTANTE: comprobar conexion y cuenta encoder (500 PPR => 2000 cuentas/vuelta)");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Stack libre [bytes]: ui=%u telemetry=%u oled=%u",
                 (unsigned)uxTaskGetStackHighWaterMark(g_ui_task_handle),
                 (unsigned)uxTaskGetStackHighWaterMark(g_telemetry_task_handle),
                 (unsigned)uxTaskGetStackHighWaterMark(g_display_task_handle));
    }
}