/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The robot follows a line with a PD controller and a small state machine:
 *  - Left and right corners are detected as a wide pattern on one side. The robot drives
 *    forward onto the corner, stops, and then turns in place.
 *  - The T intersection at the end of the track is detected as a wide pattern on both sides.
 *    The robot stops and turns 180 degrees there and follows the track back the other way.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author Jamal Bhola
 *
 */
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/Systick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"
#include "inc/Print_Binary.h"

// Motor speeds (PWM duty values)
#define PWM_NOMINAL 2000
#define MIN_SPEED 1200
#define MAX_PWM 5000
#define TURN_SPEED 2500
#define SEARCH_SPEED 1800

// PD controller
#define KP 5
#define KD 10
#define SLOWDOWN 3
// The D term is averaged over roughly this many samples. Set to 1 to turn the averaging off.
#define D_FILTER 4

// Sensor groups (bit 7 = leftmost sensor, bit 0 = rightmost sensor)
#define CENTER_MASK 0x18
#define LEFT_MASK 0xF0
#define RIGHT_MASK 0x0F

// A "wide" pattern (corner or T) is this many sensors or more seeing black
#define INTERSECTION_MIN_SENSORS 5
// One half of the array counts as "full" when this many of its 4 sensors see black
#define FULL_SIDE_SENSORS 3
#define INTERSECTION_CONFIRM_SAMPLES 3
#define LOST_CONFIRM_SAMPLES 3
#define CLEAR_SAMPLES 15
#define TIMEOUT_SAMPLES 300
#define PAUSE_SAMPLES 20
// During a 180 degree turn, ignore any line seen before this many samples have passed.
// This stops the robot from locking onto an arm of the T instead of the track behind it.
// It must be shorter than the time a full 180 degree turn takes.
#define UTURN_MIN_SAMPLES 25

// Print the sensor byte once every this many samples (10 = ten times per second)
#define PRINT_EVERY_N_SAMPLES 10

typedef enum {
    FOLLOW,
    CLEAR_INTERSECTION,
    PAUSE_BEFORE_TURN,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_AROUND,
    SEARCH,
    STOPPED
} Robot_State;

static volatile uint8_t sensor_data = 0;
static volatile uint8_t new_sample = 0;

static Robot_State state = FOLLOW;
static Robot_State pending_turn = TURN_RIGHT;
static Robot_State last_side = TURN_RIGHT;
static int32_t prev_error = 0;
static int32_t filtered_d = 0;
static uint16_t state_count = 0;
static uint16_t wide_count = 0;
static uint16_t lost_count = 0;
static uint16_t left_votes = 0;
static uint16_t right_votes = 0;
static uint8_t center_cleared = 0;

void Line_Follow_Step(uint8_t data);
void Timer_A1_Periodic_Task(void);
void SysTick_Handler(void);

int main(void)
{
    uint8_t print_count = 0;

    DisableInterrupts();

    Clock_Init48MHz();

    LED1_Init();
    LED2_Init();

    Reflectance_Sensor_Init();

    EUSCI_A0_UART_Init_Printf();

    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    Motor_Init();

    EnableInterrupts();

    while(1)
    {
        if (new_sample)
        {
            // Clear the flag before reading the data. If the interrupt delivers a
            // newer sample right after this line, its flag is kept instead of erased.
            new_sample = 0;
            uint8_t data = sensor_data;

            Line_Follow_Step(data);

            print_count++;
            if (print_count >= PRINT_EVERY_N_SAMPLES)
            {
                print_count = 0;
                Print_Binary(data);
            }
        }
    }
}

static uint8_t Count_Active_Sensors(uint8_t data)
{
    uint8_t count = 0;
    while (data)
    {
        count += data & 0x01;
        data >>= 1;
    }
    return count;
}

static uint8_t Is_Line_Pattern(uint8_t data)
{
    uint8_t active = Count_Active_Sensors(data);
    return (active > 0) && (active < INTERSECTION_MIN_SENSORS);
}

// True when both halves of the sensor array see black at once: the bar of the T
static uint8_t Is_T_Pattern(uint8_t data)
{
    return (Count_Active_Sensors(data & LEFT_MASK) >= FULL_SIDE_SENSORS) &&
           (Count_Active_Sensors(data & RIGHT_MASK) >= FULL_SIDE_SENSORS);
}

static uint16_t Clamp_PWM(int32_t value)
{
    if (value < 0)
    {
        return 0;
    }
    if (value > MAX_PWM)
    {
        return MAX_PWM;
    }
    return (uint16_t)value;
}

static void Enter_State(Robot_State new_state)
{
    state = new_state;
    state_count = 0;
    center_cleared = 0;
}

static void Reset_Votes(void)
{
    wide_count = 0;
    left_votes = 0;
    right_votes = 0;
}

static void Track_Side(int32_t error)
{
    if (error < 0)
    {
        last_side = TURN_LEFT;
    }
    else if (error > 0)
    {
        last_side = TURN_RIGHT;
    }
}

static void Resume_Following(uint8_t data)
{
    prev_error = Reflectance_Sensor_Position(data);
    filtered_d = 0;
    Track_Side(prev_error);
    lost_count = 0;
    Reset_Votes();
    Enter_State(FOLLOW);
}

static Robot_State Choose_Turn(void)
{
    uint16_t full = FULL_SIDE_SENSORS * wide_count;
    if ((left_votes >= full) && (right_votes >= full))
    {
        return TURN_AROUND;
    }
    if (left_votes > right_votes)
    {
        return TURN_LEFT;
    }
    if (right_votes > left_votes)
    {
        return TURN_RIGHT;
    }
    return last_side;
}

static void Follow_PD(uint8_t data)
{
    int32_t error = Reflectance_Sensor_Position(data);
    Track_Side(error);
    int32_t magnitude = (error < 0) ? -error : error;
    int32_t base = PWM_NOMINAL - (SLOWDOWN * magnitude);
    if (base < MIN_SPEED)
    {
        base = MIN_SPEED;
    }

    filtered_d = (((D_FILTER - 1) * filtered_d) + (error - prev_error)) / D_FILTER;
    prev_error = error;

    int32_t correction = (KP * error) + (KD * filtered_d);
    Motor_Forward(Clamp_PWM(base + correction), Clamp_PWM(base - correction));
}

static void Update_LEDs(void)
{
    switch(state)
    {
        case FOLLOW:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_GREEN);
            break;

        case CLEAR_INTERSECTION:
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_BLUE);
            break;

        case PAUSE_BEFORE_TURN:
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_GREEN);
            break;

        case TURN_LEFT:
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_WHITE);
            break;

        case TURN_RIGHT:
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_SKY_BLUE);
            break;

        case TURN_AROUND:
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_RED);
            break;

        case SEARCH:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_YELLOW);
            break;

        case STOPPED:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_RED);
            break;
    }
}

void Line_Follow_Step(uint8_t data)
{
    state_count++;

    switch(state)
    {
        case FOLLOW:
            if (data == 0)
            {
                Reset_Votes();
                lost_count++;
                if (lost_count >= LOST_CONFIRM_SAMPLES)
                {
                    Enter_State(SEARCH);
                }
                break;
            }
            lost_count = 0;

            if (Count_Active_Sensors(data) >= INTERSECTION_MIN_SENSORS)
            {
                uint8_t left = Count_Active_Sensors(data & LEFT_MASK);
                uint8_t right = Count_Active_Sensors(data & RIGHT_MASK);
                left_votes += left;
                right_votes += right;
                wide_count++;
                if (left > right)
                {
                    last_side = TURN_LEFT;
                }
                else if (right > left)
                {
                    last_side = TURN_RIGHT;
                }

                Motor_Forward(TURN_SPEED, TURN_SPEED);
                if (wide_count >= INTERSECTION_CONFIRM_SAMPLES)
                {
                    pending_turn = Choose_Turn();
                    Reset_Votes();
                    if (pending_turn == TURN_AROUND)
                    {
                        Enter_State(PAUSE_BEFORE_TURN);
                    }
                    else
                    {
                        Enter_State(CLEAR_INTERSECTION);
                    }
                }
                break;
            }

            Reset_Votes();
            Follow_PD(data);
            break;

        case CLEAR_INTERSECTION:
            Motor_Forward(TURN_SPEED, TURN_SPEED);
            if (Is_T_Pattern(data))
            {
                pending_turn = TURN_AROUND;
                Enter_State(PAUSE_BEFORE_TURN);
            }
            else if (state_count >= CLEAR_SAMPLES)
            {
                Enter_State(PAUSE_BEFORE_TURN);
            }
            break;

        case PAUSE_BEFORE_TURN:
            Motor_Stop();
            if (state_count >= PAUSE_SAMPLES)
            {
                Enter_State(pending_turn);
            }
            break;

        case TURN_LEFT:
        case TURN_RIGHT:
        case TURN_AROUND:
        {
            uint16_t min_samples = (state == TURN_AROUND) ? UTURN_MIN_SAMPLES : 0;

            if (state == TURN_LEFT)
            {
                Motor_Left(TURN_SPEED, TURN_SPEED);
            }
            else
            {
                Motor_Right(TURN_SPEED, TURN_SPEED);
            }

            if ((data & CENTER_MASK) == 0)
            {
                center_cleared = 1;
            }
            else if (center_cleared && (state_count >= min_samples))
            {
                Resume_Following(data);
                break;
            }

            if (state_count >= TIMEOUT_SAMPLES)
            {
                Enter_State(STOPPED);
            }
            break;
        }

        case SEARCH:
            if (Is_Line_Pattern(data))
            {
                Resume_Following(data);
                break;
            }

            if (last_side == TURN_LEFT)
            {
                Motor_Left(SEARCH_SPEED, SEARCH_SPEED);
            }
            else
            {
                Motor_Right(SEARCH_SPEED, SEARCH_SPEED);
            }

            if (state_count >= TIMEOUT_SAMPLES)
            {
                Enter_State(STOPPED);
            }
            break;

        case STOPPED:
            Motor_Stop();
            if (Is_Line_Pattern(data))
            {
                Resume_Following(data);
            }
            break;
    }

    Update_LEDs();
}

void Timer_A1_Periodic_Task(void)
{
    // Counts 0 to 9 and repeats, giving one sensor reading every 10 ms.
    // Step 0 starts the reading and step 1 (1 ms later) finishes it.
    static uint8_t phase = 0;

    if (phase == 0)
    {
        Reflectance_Sensor_Start();
    }
    else if (phase == 1)
    {
        sensor_data = Reflectance_Sensor_End();
        new_sample = 1;
    }

    phase++;
    if (phase >= 10)
    {
        phase = 0;
    }
}

void SysTick_Handler(void)
{
}
