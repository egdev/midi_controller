#pragma once

#define NUM_FADERS    9
#define NUM_CHANNELS  17

#define REPEAT_CTR_RELOAD         31  // retries to reach target position
#define TIMEOUT_CTR_RELOAD       255 // give up after how many mS
#define MANUAL_MOVE_CTR_RELOAD   255 // ignore new position request for how many mS

#define MF_DEADBAND             3
#define MF_PWM_PERIOD           3
#define MF_DUTY_CYCLE_UP        1
#define MF_DUTY_CYCLE_DOWN      1

#define AIN_DEADBAND 7
