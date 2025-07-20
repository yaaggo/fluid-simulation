#include "mpu6050.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>

// Funções estáticas para comunicação I2C
static void mpu6050_i2c_init();
static bool mpu6050_write_register(uint8_t reg, uint8_t value);
static bool mpu6050_read_register(uint8_t reg, uint8_t *value);
static bool mpu6050_read_registers(uint8_t reg, uint8_t *buffer, size_t len);

// Inicializa a comunicação I2C para o MPU6050
static void mpu6050_i2c_init() {
    // Usa 400kHz para comunicação rápida (MPU6050 suporta até 400kHz)
    i2c_init(MPU_I2C_PORT, 400 * 1000);
    gpio_set_function(MPU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU_SCL_PIN, GPIO_FUNC_I2C);
    
    // Ativa pull-ups internos (importante para I2C)
    gpio_pull_up(MPU_SDA_PIN);
    gpio_pull_up(MPU_SCL_PIN);
}

// Escreve um valor em um registrador do MPU6050
static bool mpu6050_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    int result = i2c_write_blocking(MPU_I2C_PORT, MPU6050_ADDRESS, data, 2, false);
    return result == 2;
}

// Lê um valor de um registrador do MPU6050
static bool mpu6050_read_register(uint8_t reg, uint8_t *value) {
    // Primeiro envia o endereço do registrador
    int result = i2c_write_blocking(MPU_I2C_PORT, MPU6050_ADDRESS, &reg, 1, true);
    if (result != 1) return false;
    
    // Depois lê o valor
    result = i2c_read_blocking(MPU_I2C_PORT, MPU6050_ADDRESS, value, 1, false);
    return result == 1;
}

// Lê múltiplos registradores sequenciais
static bool mpu6050_read_registers(uint8_t reg, uint8_t *buffer, size_t len) {
    // Primeiro envia o endereço do registrador inicial
    int result = i2c_write_blocking(MPU_I2C_PORT, MPU6050_ADDRESS, &reg, 1, true);
    if (result != 1) return false;
    
    // Depois lê os dados
    result = i2c_read_blocking(MPU_I2C_PORT, MPU6050_ADDRESS, buffer, len, false);
    return result == (int)len;
}

// Inicializa o MPU6050
bool mpu6050_init(mpu6050_t *mpu) {
    if (mpu->initialized) return true;
    
    mpu6050_i2c_init();
    
    // Sai do modo sleep e reseta o dispositivo
    if (!mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x80)) return false;
    sleep_ms(100); // Aguarda reset
    
    // Sai do modo sleep e usa o clock interno
    if (!mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x00)) return false;
    sleep_ms(10);
    
    // Habilita todos os sensores
    if (!mpu6050_write_register(MPU6050_REG_PWR_MGMT_2, 0x00)) return false;
    
    // Configura escalas padrão
    mpu->accel_scale = MPU6050_ACCEL_SCALE_2G;
    mpu->gyro_scale = MPU6050_GYRO_SCALE_250DPS;
    
    if (!mpu6050_set_accel_scale(mpu, mpu->accel_scale)) return false;
    if (!mpu6050_set_gyro_scale(mpu, mpu->gyro_scale)) return false;
    
    // Configura filtro passa-baixa para reduzir ruído
    if (!mpu6050_set_dlpf(mpu, MPU6050_DLPF_44HZ)) return false;
    
    // Zera os offsets
    mpu->offsets.accel_x_offset = 0;
    mpu->offsets.accel_y_offset = 0;
    mpu->offsets.accel_z_offset = 0;
    mpu->offsets.gyro_x_offset = 0;
    mpu->offsets.gyro_y_offset = 0;
    mpu->offsets.gyro_z_offset = 0;
    
    mpu->initialized = true;
    return true;
}

// Testa a conexão com o MPU6050
bool mpu6050_test_connection(mpu6050_t *mpu) {
    uint8_t who_am_i;
    if (!mpu6050_read_register(MPU6050_REG_WHO_AM_I, &who_am_i)) return false;
    
    // O MPU6050 deve retornar 0x68
    return (who_am_i == 0x68 || who_am_i == 0x70);
}

// Desliga o MPU6050 e libera recursos
void mpu6050_shutdown(mpu6050_t *mpu) {
    // Coloca o dispositivo em modo sleep
    mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x40);
    
    // Libera os pinos I2C
    gpio_set_function(MPU_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(MPU_SCL_PIN, GPIO_FUNC_NULL);
    
    i2c_deinit(MPU_I2C_PORT);
    
    mpu->initialized = false;
}

// Configura a escala do acelerômetro
bool mpu6050_set_accel_scale(mpu6050_t *mpu, mpu6050_accel_scale_t scale) {
    uint8_t config = (scale << 3);
    if (!mpu6050_write_register(MPU6050_REG_ACCEL_CONFIG, config)) return false;
    
    mpu->accel_scale = scale;
    mpu->accel_scale_factor = mpu6050_get_accel_sensitivity(scale);
    return true;
}

// Configura a escala do giroscópio
bool mpu6050_set_gyro_scale(mpu6050_t *mpu, mpu6050_gyro_scale_t scale) {
    uint8_t config = (scale << 3);
    if (!mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, config)) return false;
    
    mpu->gyro_scale = scale;
    mpu->gyro_scale_factor = mpu6050_get_gyro_sensitivity(scale);
    return true;
}

// Configura o filtro passa-baixa digital
bool mpu6050_set_dlpf(mpu6050_t *mpu, mpu6050_dlpf_t dlpf) {
    return mpu6050_write_register(MPU6050_REG_CONFIG, dlpf);
}

// Lê todos os dados brutos do sensor
bool mpu6050_read_raw(mpu6050_t *mpu, mpu6050_raw_data_t *raw_data) {
    uint8_t buffer[14];
    
    // Lê todos os registradores de dados de uma vez (0x3B a 0x48)
    if (!mpu6050_read_registers(MPU6050_REG_ACCEL_XOUT_H, buffer, 14)) return false;
    
    // Converte os dados (big-endian para little-endian)
    raw_data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    raw_data->temperature = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    raw_data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw_data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw_data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // Aplica os offsets de calibração
    raw_data->accel_x -= mpu->offsets.accel_x_offset;
    raw_data->accel_y -= mpu->offsets.accel_y_offset;
    raw_data->accel_z -= mpu->offsets.accel_z_offset;
    raw_data->gyro_x -= mpu->offsets.gyro_x_offset;
    raw_data->gyro_y -= mpu->offsets.gyro_y_offset;
    raw_data->gyro_z -= mpu->offsets.gyro_z_offset;
    
    return true;
}

// Lê e converte os dados para unidades físicas
bool mpu6050_read_data(mpu6050_t *mpu, mpu6050_data_t *data) {
    mpu6050_raw_data_t raw;
    
    if (!mpu6050_read_raw(mpu, &raw)) return false;
    
    // Converte para unidades físicas
    data->accel_x_g = raw.accel_x / mpu->accel_scale_factor;
    data->accel_y_g = raw.accel_y / mpu->accel_scale_factor;
    data->accel_z_g = raw.accel_z / mpu->accel_scale_factor;
    
    data->gyro_x_dps = raw.gyro_x / mpu->gyro_scale_factor;
    data->gyro_y_dps = raw.gyro_y / mpu->gyro_scale_factor;
    data->gyro_z_dps = raw.gyro_z / mpu->gyro_scale_factor;
    
    data->temperature_c = mpu6050_raw_to_celsius(raw.temperature);
    
    return true;
}

// Lê apenas os dados do acelerômetro
bool mpu6050_read_accel_raw(mpu6050_t *mpu, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];
    
    if (!mpu6050_read_registers(MPU6050_REG_ACCEL_XOUT_H, buffer, 6)) return false;
    
    *x = (int16_t)((buffer[0] << 8) | buffer[1]) - mpu->offsets.accel_x_offset;
    *y = (int16_t)((buffer[2] << 8) | buffer[3]) - mpu->offsets.accel_y_offset;
    *z = (int16_t)((buffer[4] << 8) | buffer[5]) - mpu->offsets.accel_z_offset;
    
    return true;
}

// Lê apenas os dados do giroscópio
bool mpu6050_read_gyro_raw(mpu6050_t *mpu, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];
    
    if (!mpu6050_read_registers(MPU6050_REG_GYRO_XOUT_H, buffer, 6)) return false;
    
    *x = (int16_t)((buffer[0] << 8) | buffer[1]) - mpu->offsets.gyro_x_offset;
    *y = (int16_t)((buffer[2] << 8) | buffer[3]) - mpu->offsets.gyro_y_offset;
    *z = (int16_t)((buffer[4] << 8) | buffer[5]) - mpu->offsets.gyro_z_offset;
    
    return true;
}

// Lê apenas a temperatura
bool mpu6050_read_temperature_raw(mpu6050_t *mpu, int16_t *temp) {
    uint8_t buffer[2];
    
    if (!mpu6050_read_registers(MPU6050_REG_TEMP_OUT_H, buffer, 2)) return false;
    
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    return true;
}

// Calibra o sensor calculando offsets médios
void mpu6050_calibrate(mpu6050_t *mpu, int samples) {
    if (samples <= 0) samples = 1000;
    
    long accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    long gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    
    // Zera os offsets temporariamente para calibração
    mpu6050_offsets_t temp_offsets = mpu->offsets;
    mpu->offsets.accel_x_offset = 0;
    mpu->offsets.accel_y_offset = 0;
    mpu->offsets.accel_z_offset = 0;
    mpu->offsets.gyro_x_offset = 0;
    mpu->offsets.gyro_y_offset = 0;
    mpu->offsets.gyro_z_offset = 0;
    
    for (int i = 0; i < samples; i++) {
        mpu6050_raw_data_t raw;
        if (mpu6050_read_raw(mpu, &raw)) {
            accel_x_sum += raw.accel_x;
            accel_y_sum += raw.accel_y;
            accel_z_sum += raw.accel_z - (int16_t)(mpu->accel_scale_factor); // Subtrai 1g do eixo Z
            gyro_x_sum += raw.gyro_x;
            gyro_y_sum += raw.gyro_y;
            gyro_z_sum += raw.gyro_z;
        }
        sleep_ms(2);
    }
    
    // Calcula os offsets médios
    mpu->offsets.accel_x_offset = accel_x_sum / samples;
    mpu->offsets.accel_y_offset = accel_y_sum / samples;
    mpu->offsets.accel_z_offset = accel_z_sum / samples;
    mpu->offsets.gyro_x_offset = gyro_x_sum / samples;
    mpu->offsets.gyro_y_offset = gyro_y_sum / samples;
    mpu->offsets.gyro_z_offset = gyro_z_sum / samples;
}

// Define offsets manualmente
void mpu6050_set_offsets(mpu6050_t *mpu, mpu6050_offsets_t *offsets) {
    mpu->offsets = *offsets;
}

// Obtém os offsets atuais
void mpu6050_get_offsets(mpu6050_t *mpu, mpu6050_offsets_t *offsets) {
    *offsets = mpu->offsets;
}

// Retorna o fator de escala do acelerômetro
float mpu6050_get_accel_sensitivity(mpu6050_accel_scale_t scale) {
    switch (scale) {
        case MPU6050_ACCEL_SCALE_2G:  return 16384.0f;
        case MPU6050_ACCEL_SCALE_4G:  return 8192.0f;
        case MPU6050_ACCEL_SCALE_8G:  return 4096.0f;
        case MPU6050_ACCEL_SCALE_16G: return 2048.0f;
        default: return 16384.0f;
    }
}

// Retorna o fator de escala do giroscópio
float mpu6050_get_gyro_sensitivity(mpu6050_gyro_scale_t scale) {
    switch (scale) {
        case MPU6050_GYRO_SCALE_250DPS:  return 131.0f;
        case MPU6050_GYRO_SCALE_500DPS:  return 65.5f;
        case MPU6050_GYRO_SCALE_1000DPS: return 32.8f;
        case MPU6050_GYRO_SCALE_2000DPS: return 16.4f;
        default: return 131.0f;
    }
}

// Converte temperatura bruta para Celsius
float mpu6050_raw_to_celsius(int16_t raw_temp) {
    // Fórmula do datasheet: Temperature = 36.53 + (raw_temp / 340.0)
    return 36.53f + ((float)raw_temp / 340.0f);
}

// Calcula o ângulo pitch (inclinação para frente/trás) em graus
float mpu6050_calculate_pitch(float accel_x, float accel_y, float accel_z) {
    return atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;
}

// Calcula o ângulo roll (inclinação lateral) em graus
float mpu6050_calculate_roll(float accel_x, float accel_y, float accel_z) {
    return atan2(accel_y, accel_z) * 180.0f / M_PI;
}

// Calcula a magnitude de um vetor 3D
float mpu6050_calculate_magnitude(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
}