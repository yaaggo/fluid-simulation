#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "include/button.h"
#include "include/display.h"
#include "include/mpu6050.h"

#define BALL_RADIUS 3
#define GRAVITY_SENSITIVITY 0.15f
#define DAMPING 0.95f
#define BOUNCE_FACTOR 0.6f

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 8
#define BLOCK_SIZE 8

const uint8_t maze[MAZE_HEIGHT][MAZE_WIDTH] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,1,0,1,0,1,1,1,1,1,0,1,1,0,1},
    {1,0,1,0,0,0,0,0,0,0,1,0,1,0,0,1},
    {1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,1},
    {1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,1},
    {1,1,1,1,1,0,1,1,1,1,1,1,1,1,2,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

void draw_maze(display *disp) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            if (maze[y][x] == 1) {
                display_draw_rectangle(x * BLOCK_SIZE, y * BLOCK_SIZE, (x + 1) * BLOCK_SIZE - 1, (y + 1) * BLOCK_SIZE - 1, true, true, disp);
            } else if (maze[y][x] == 2) {
                display_draw_rectangle(x * BLOCK_SIZE + 2, y * BLOCK_SIZE + 2, (x + 1) * BLOCK_SIZE - 3, (y + 1) * BLOCK_SIZE - 3, false, true, disp);
            }
        }
    }
}

bool check_collision(float x, float y) {
    if (x - BALL_RADIUS < 0 || x + BALL_RADIUS > DISPLAY_WIDTH || y - BALL_RADIUS < 0 || y + BALL_RADIUS > DISPLAY_HEIGHT) {
        return true;
    }
    int maze_x = (int)(x / BLOCK_SIZE);
    int maze_y = (int)(y / BLOCK_SIZE);
    
    if (maze_x >= 0 && maze_x < MAZE_WIDTH && maze_y >= 0 && maze_y < MAZE_HEIGHT) {
        if (maze[maze_y][maze_x] == 1) {
            return true;
        }
    }
    return false;
}

bool check_win_condition(float x, float y) {
    int maze_x = (int)(x / BLOCK_SIZE);
    int maze_y = (int)(y / BLOCK_SIZE);
    if (maze[maze_y][maze_x] == 2) {
        return true;
    }
    return false;
}

display disp;
mpu6050_t mpu;

int main() {
    stdio_init_all();


    display_init(&disp);
    button_init();
    
    if (!mpu6050_init(&mpu)) {
        display_draw_string(5, 20, "MPU6050 FALHOU!", true, &disp);
        display_update(&disp);
        while(1);
    }
    mpu6050_calibrate(&mpu, 1000);
    
    float ball_x = 12.0f, ball_y = 12.0f;
    float vel_x = 0.0f, vel_y = 0.0f;
    bool game_won = false;

    while (1) {
        button_event event = button_get_event();
        if (event != BUTTON_NONE) {
            if (event == BUTTON_A) {
                display_shutdown(&disp);
                reset_usb_boot(0, 0);
            }
            if (event == BUTTON_B) {
                ball_x = 12.0f; ball_y = 12.0f;
                vel_x = 0.0f; vel_y = 0.0f;
                game_won = false;
            }
            button_clear_event();
        }

        if (game_won) {
            display_clear(&disp);
            display_draw_string(35, 20, "VENCEU!", true, &disp);
            display_draw_string(10, 40, "BOTAO B: NEW", true, &disp);
            display_draw_string(10, 50, "BOTAO A: EXIT", true, &disp);
            display_update(&disp);
            sleep_ms(100);
            continue;
        }

        mpu6050_data_t sensor_data;
        mpu6050_read_data(&mpu, &sensor_data);

        vel_x += sensor_data.accel_x_g * GRAVITY_SENSITIVITY;
        vel_y -= sensor_data.accel_y_g * GRAVITY_SENSITIVITY;

        vel_x *= DAMPING;
        vel_y *= DAMPING;

        float next_x = ball_x + vel_x;
        float next_y = ball_y + vel_y;

        if (check_collision(next_x, ball_y)) {
            vel_x = -vel_x * BOUNCE_FACTOR;
        } else {
            ball_x = next_x;
        }

        if (check_collision(ball_x, next_y)) {
            vel_y = -vel_y * BOUNCE_FACTOR;
        } else {
            ball_y = next_y;
        }
        
        if(check_win_condition(ball_x, ball_y)) {
            game_won = true;
        }

        display_clear(&disp);
        
        draw_maze(&disp);
        
        display_draw_circle((int)ball_x, (int)ball_y, BALL_RADIUS, true, true, &disp);
        
        display_update(&disp);

        sleep_ms(10); 
    }

    return 0;
}