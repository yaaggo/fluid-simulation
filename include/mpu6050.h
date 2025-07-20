#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

/*
* Biblioteca para controle do sensor MPU6050 (giroscópio + acelerômetro)
* Utiliza comunicação I2C para interface com o sensor
*/

// Pinos I2C (ajuste conforme sua configuração)
#define MPU_SDA_PIN 0
#define MPU_SCL_PIN 1
#define MPU_I2C_PORT i2c0

// Endereço I2C do MPU6050 (pode ser 0x68 ou 0x69 dependendo do pino AD0)
#define MPU6050_ADDRESS 0x68

// Registradores importantes do MPU6050
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_PWR_MGMT_2    0x6C
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_WHO_AM_I      0x75

// Registradores de dados do acelerômetro
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_ACCEL_XOUT_L  0x3C
#define MPU6050_REG_ACCEL_YOUT_H  0x3D
#define MPU6050_REG_ACCEL_YOUT_L  0x3E
#define MPU6050_REG_ACCEL_ZOUT_H  0x3F
#define MPU6050_REG_ACCEL_ZOUT_L  0x40

// Registradores de dados do giroscópio
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_GYRO_XOUT_L   0x44
#define MPU6050_REG_GYRO_YOUT_H   0x45
#define MPU6050_REG_GYRO_YOUT_L   0x46
#define MPU6050_REG_GYRO_ZOUT_H   0x47
#define MPU6050_REG_GYRO_ZOUT_L   0x48

// Registrador de temperatura
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_TEMP_OUT_L    0x42

// Escalas do acelerômetro
typedef enum {
    MPU6050_ACCEL_SCALE_2G = 0,
    MPU6050_ACCEL_SCALE_4G = 1,
    MPU6050_ACCEL_SCALE_8G = 2,
    MPU6050_ACCEL_SCALE_16G = 3
} mpu6050_accel_scale_t;

// Escalas do giroscópio
typedef enum {
    MPU6050_GYRO_SCALE_250DPS = 0,
    MPU6050_GYRO_SCALE_500DPS = 1,
    MPU6050_GYRO_SCALE_1000DPS = 2,
    MPU6050_GYRO_SCALE_2000DPS = 3
} mpu6050_gyro_scale_t;

// Filtro passa-baixa digital
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ = 2,
    MPU6050_DLPF_44HZ = 3,
    MPU6050_DLPF_21HZ = 4,
    MPU6050_DLPF_10HZ = 5,
    MPU6050_DLPF_5HZ = 6
} mpu6050_dlpf_t;

// Estrutura para dados brutos do sensor
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
} mpu6050_raw_data_t;

// Estrutura para dados processados (em unidades físicas)
typedef struct {
    float accel_x_g;      // aceleração em g
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;     // velocidade angular em graus/segundo
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;  // temperatura em Celsius
} mpu6050_data_t;

// Estrutura para offset/calibração
typedef struct {
    int16_t accel_x_offset;
    int16_t accel_y_offset;
    int16_t accel_z_offset;
    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;
} mpu6050_offsets_t;

// Estrutura principal do MPU6050
typedef struct {
    bool initialized;
    mpu6050_accel_scale_t accel_scale;
    mpu6050_gyro_scale_t gyro_scale;
    float accel_scale_factor;
    float gyro_scale_factor;
    mpu6050_offsets_t offsets;
} mpu6050_t;

// Funções principais
bool mpu6050_init(mpu6050_t *mpu);
bool mpu6050_test_connection(mpu6050_t *mpu);
void mpu6050_shutdown(mpu6050_t *mpu);

// Configuração do sensor
bool mpu6050_set_accel_scale(mpu6050_t *mpu, mpu6050_accel_scale_t scale);
bool mpu6050_set_gyro_scale(mpu6050_t *mpu, mpu6050_gyro_scale_t scale);
bool mpu6050_set_dlpf(mpu6050_t *mpu, mpu6050_dlpf_t dlpf);

// Leitura de dados
bool mpu6050_read_raw(mpu6050_t *mpu, mpu6050_raw_data_t *raw_data);
bool mpu6050_read_data(mpu6050_t *mpu, mpu6050_data_t *data);
bool mpu6050_read_accel_raw(mpu6050_t *mpu, int16_t *x, int16_t *y, int16_t *z);
bool mpu6050_read_gyro_raw(mpu6050_t *mpu, int16_t *x, int16_t *y, int16_t *z);
bool mpu6050_read_temperature_raw(mpu6050_t *mpu, int16_t *temp);

// Calibração
void mpu6050_calibrate(mpu6050_t *mpu, int samples);
void mpu6050_set_offsets(mpu6050_t *mpu, mpu6050_offsets_t *offsets);
void mpu6050_get_offsets(mpu6050_t *mpu, mpu6050_offsets_t *offsets);

// Funções auxiliares
float mpu6050_get_accel_sensitivity(mpu6050_accel_scale_t scale);
float mpu6050_get_gyro_sensitivity(mpu6050_gyro_scale_t scale);
float mpu6050_raw_to_celsius(int16_t raw_temp);

// Cálculos úteis
float mpu6050_calculate_pitch(float accel_x, float accel_y, float accel_z);
float mpu6050_calculate_roll(float accel_x, float accel_y, float accel_z);
float mpu6050_calculate_magnitude(float x, float y, float z);

#endif