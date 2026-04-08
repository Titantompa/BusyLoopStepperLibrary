#pragma once
#include <Arduino.h>

namespace BusyLoopStepper
{
    typedef enum
    {
        STEP_IDLE = 0,     // No movement nor processing, step pin is low and direction pin holds it's last state
        STEP_PULSING = 1,  // While in this state the step pulse is high, this should be held for a minimum configurable time to ensure that the step is registered by the driver
        STEP_RUNNING = 2,  // While in this state the step pulse is low, the length of this period dictates the speed at which the stepper is running
        STEP_REVERSING = 3, // Switching between forward and reverse direction, need to wait for the driver to settle before sending pulses
        STEP_EBRAKE // Stop the stepper as quickly as possible, stepper is idle after decellerating to a halt even though stepper might stop sooner or later!
    } StepperState;

    /**
     * @brief
     *
     * @tparam CLOCK_FREQUENCY The cpu clock frequency in Hz, this is used to convert the speed and accelleration values from steps per second and steps per second per second
     * to ticks, which are used for the timing of the step pulses. This is a template parameter to allow for compile time calculations of the timer ticks based on the clock frequency.
     */
    template <uint32_t CLOCK_FREQUENCY>
    class StepperData
    {
    public:
        StepperData()
            : startTime(0),
              delay(0),
              state(STEP_IDLE),
              direction(0), // uninitialized
              stepPin(gpio_num_t::GPIO_NUM_NC),
              dirPin(gpio_num_t::GPIO_NUM_NC),
              currentSpeed(0),
              maxSpeed(10), // Default to ten steps per second
              accelleration(10),
              firstStepDuration(0),
              remainingSteps(0),
              currentPosition(0),
              pulseDuration(CLOCK_FREQUENCY / 1000000), // 1us pulse duration
              reverseDelay(CLOCK_FREQUENCY / 1000000)   // 1us reverse delay
        {
        }

        /**
         * @brief The number of timer ticks to delay before the next action should be taken. This value is decremented on each timer interrupt. When it reaches 0, the state should be processed
         * which in turn causes this value to be set to the new delay before processing again.
         */
        volatile uint32_t startTime;
        volatile uint32_t delay;

        /**
         * @brief bit 0-1: state of the stepper, bit 2: direction of the stepper (0 for forward in positive direction, 1 for reverse in negative direction)
         */
        volatile BusyLoopStepper::StepperState state;

        /**
         * @brief -1 or 1, depending on the direction of the stepper. This is used to update the current position and remaining steps when a pulse is sent to the step pin.
         */
        volatile int32_t direction;
        volatile gpio_num_t stepPin;
        volatile gpio_num_t dirPin;

        /**
         * @brief The current speed, expressed in ticks per step. This is the period between the pulses to the step pin.
         */
        volatile uint32_t currentSpeed;

        /**
         * @brief The maximum speed of the stepper motor expressed as ticks per step.
         * This is effectively the timer delay between each pulse when the stepper is running at maximum speed.
         */
        volatile uint32_t maxSpeed;

        /**
         * @brief Expressed as steps per second per second, this is the rate at which the stepper should accelerate or decelerate.
         * This is used to calculate the speed of the stepper motor through the intervals of the pulses sent to the step pin.
         */
        volatile uint32_t accelleration;

        /**
         * @brief Pre computed duration of the first step. This is used to quickly calculate the initial delay for the first step when starting the stepper motor.
         * The duration of subsequent steps is derived from the previous step and the accelleration, so this value is only used for the first step. This is calculated whenever the accelleration is updated.
         */
        volatile uint32_t firstStepDuration;

        /**
         * @brief The remaining number of steps to move. This is used to calculate decelleration and when to stop the stepper motor.
         * When this reaches 0, the stepper should stop moving and go into the STEP_IDLE state. The value is continuously updated as pulses are sent to the stepper driver.
         */
        volatile uint32_t remainingSteps;

        /**
         * @brief Holds the current position in steps, of the stepper motor.
         * This is used to keep track of the position of the stepper motor and can be used for relative movements. This is continuously updated as pulses are sent to the stepper driver.
         */
        volatile int32_t currentPosition;

        /**
         * @brief The number of timer ticks to delay after setting the step pin high before setting it low again. This is used to ensure that the step pin is held high long enough to
         * be registered by the stepper driver. This is the minimum delay, on top of the processing time of the timer interrupt, so the actual delay will always be longer.
         * Note, the timer interrupt takes ~5.6us to process, which is the effective minimum pulse duration, despite which value had been configured here.
         */
        volatile uint32_t pulseDuration;

        /**
         * @brief The number of timer ticks to delay before and after making a change to the dir pin and before sending pulses on the step pin. This is used to ensure that the driver has time to settle.
         *  This is the minimum delay, in addition to the processing time of the timer interrupt, so the actual delay will always be longer.
         */
        volatile uint32_t reverseDelay;

    public:
        /**
         * @brief
         *
         */
        void setup()
        {
            pinMode(dirPin, OUTPUT);
            pinMode(stepPin, OUTPUT);
            gpio_set_level((gpio_num_t)dirPin, direction == 1 ? HIGH : LOW);
        }

        /**
         * @brief
         *
         */
        bool is_idle()
        {
            return state == STEP_IDLE;
        }

        /**
         * @brief Set the stepper max speed expressed as steps per second.
         * This is used to calculate the timer delay between each pulse when the stepper is running at maximum speed.
         * @param maxSpeed
         */
        void set_stepper_max_speed(uint32_t inMaxSpeed)
        {
            assert(inMaxSpeed > 0); // Max speed must be greater than 0 to avoid division by zero and negative speeds which are not supported in this implementation

            maxSpeed = ceill((long double)CLOCK_FREQUENCY / (long double)inMaxSpeed);

            // Serial.println("Max " + String(inMaxSpeed) + " sps, " + String(maxSpeed) + " clk");
        }

        /**
         * @brief Set stepper position, this can be called at any time to update the current position of the stepper motor, this is useful for relative movements and for keeping track of the position of the stepper motor.
         *
         * @param position The new position of the stepper motor in steps.
         */
        void set_position(int32_t position)
        {
            currentPosition = position;
        }

        /**
         * @brief Get the current position of the stepper. 
         * @warning When the stepper is not idle this is a volatile value that may have changed even before this call returns.
         * @return The current position of the stepper (volatile)
         */
        int32_t get_position() const
        {
            return currentPosition;
        }

        /**
         * @brief Set the stepper accelleration
         * This is using the formula for constant accelleration, s = ut + 0.5at^2, which simplifies to s = 0.5at^2 when starting from rest. Solving for t gives us t = sqrt(2s/a) (or even
         * simpler t = sqrt(2/a) since the steps to travel is always one) which is the formula used to calculate the first step duration.
         * @param accelleration
         */
        void set_stepper_accelleration(uint32_t inAccelleration)
        {
            assert(inAccelleration > 0); // Accelleration must be greater than 0 to avoid division by zero and negative accelleration which is not supported in this implementation
            assert(state == STEP_IDLE);  // Only allow changing accelleration when the stepper is idle, otherwise we would need to handle the state transitions and timing differently

            accelleration = inAccelleration;

            // calculate the duration of the first step in timer ticks, this is used to calculate the initial speed when starting the stepper motor
            firstStepDuration = ceill(sqrt(2.0l / accelleration) * (long double)CLOCK_FREQUENCY);

            // Serial.println("Acc: " + String(inAccelleration) + " sps2, " + String(firstStepDuration) + " clk");
        }

        /**
         * @brief Set the minimum pulse duration for the step pulses.
         * 
         * @param duration The minimum pulse duration in microseconds
         */

        void set_stepper_min_pulse_duration(int32_t duration)
        {
            assert(duration < 1000); // Just to have some sane value

            pulseDuration = (CLOCK_FREQUENCY/1000000)*duration;
        }

        /**
         * @brief Move the stepper motor a relative number of steps.
         * @note This will do task delays to allow for direction changes!
         *
         * @param steps The number of steps to move. Positive values move forward, negative values move backward.
         */
        void goto_relative(int32_t steps)
        {
            assert(state == STEP_IDLE); // Only allow starting a movement when the stepper is idle, otherwise we would need to handle the state transitions and timing differently
            assert(steps != 0);

            set_stepper_direction(steps > 0);

            remainingSteps = labs(steps);
            startTime = 0;
            delay = 0;
            currentSpeed = 0;

            // Start the stepper
            state = STEP_RUNNING;
        }

        /**
         * @brief Move the stepper motor to a specific position (measured in steps).
         * @note This will do task delays to allow for direction changes!
         *
         * @param position The target position, expressed in steps relative to the 0 position.
         * 
         * @return The number of steps that will be taken to reach the position
         */
        uint32_t goto_absolute(int32_t position)
        {
            assert(state == STEP_IDLE); // Only allow starting a movement when the stepper is idle, otherwise we would need to handle the state transitions and timing differently

            if(currentPosition != position)
            {
                int32_t relative = position - currentPosition;

                goto_relative(relative);

                return labs(relative);
            }

            return 0;
        }

        /**
         * @brief Stop the stepper as quickly as possible
         * 
         * @todo There is a race condition here with the stepper engine task, risking that the stepper goes to the idle state immediately
         * 
         * @warning The stepper is idle for as long as it would take to decellerate to a stop. But it is unknown if the stepper has already stopped or
         * is still moving at that point.
         * @warning Since there is no way to measure how far the stepper motor travels before coming to a stop, the position of the stepper will be unknown
         */
        void emergency_brake()
        {
            // Set the state
            state = STEP_EBRAKE;
            // Schedule it immediately
            delay = 0;
        }

        /**
         * @brief Set the stepper direction object
         * @note This does vTaskDelay to make sure the stepper driver is given enough time to react to the change in direction before sending pulses to the step pin.
         * The delay is based on the reverseDelay parameter, which is set to 1us by default, but can be configured as needed.
         * @param forward
         */
        void set_stepper_direction(bool forward)
        {
            assert(state == STEP_IDLE); // Only allow changing direction when the stepper is idle, otherwise we would need to handle the state transitions and timing differently

            uint32_t newDirection = forward ? 1 : -1;

            if(newDirection != direction)
            {
                gpio_set_level((gpio_num_t)dirPin, forward ? HIGH : LOW);
                direction = newDirection;
                vTaskDelay(reverseDelay); // wait for the driver to settle after changing direction
            }
        }
    };

    /**
     * @brief
     *
     * @tparam NUM_STEPPERS The number of stepper motors to initialize
     */
    template <uint8_t NUM_STEPPERS, uint32_t CLOCK_FREQUENCY>
    class BusyLoopStepper
    {
        static_assert(NUM_STEPPERS > 0, "Number of steppers must be greater than 0");
        static_assert(NUM_STEPPERS <= 3, "Number of steppers must be less than or equal to 3");

    public:
        inline StepperData<CLOCK_FREQUENCY> &operator[](const int index)
        {
            assert(index >= 0 && index < NUM_STEPPERS);
            return _stepperData[index];
        }

        inline uint8_t numSteppers() const
        {
            return NUM_STEPPERS;
        }

        inline void initializeSteppers()
        {
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                _stepperData[i].setup();
            }
        }

        inline void emergencyBrake()
        {
            for (int i = 0; i < NUM_STEPPERS; i++)
            {
                if(!_stepperData[i].is_idle())
                {
                    _stepperData[i].emergency_brake();
                }
            }
        }

        /**
         * @brief
         * The speed of the stepper is determined in the following order:
         * 1 - if the stepper needs to decellerate to stop at the right position, when so the speed is determined by the remaining steps and the accelleration (which doubles as a decelleration value as well)
         * 2 - if the stepper is moving above maximum speed, it should decellerate towards maximum speed (this allows for changing the crusing speed while the stepper is running)
         * 3 - if the stepper is not moving at maximum speed yet, it should accellerate
         * 4 - keep maximum speed
         *
         * When a pulse is sent to the step pin, the values of the current position and the remaining steps are updated.
         * When the remaining steps reaches 0, the stepper should stop moving and go into the STEP_IDLE state.
         *
         * In theory it should be possible to add to the number of remaining steps while the stepper is running, which would allow for changing the target position on the fly. Reducing the number
         * of remaining step could cause the stepper to not be able to decellerate properly, and instead overshoot the target position.
         *
         * It should also be possible to panic break by changing the state of the stepper to STEP_IDLE, which would cause the stepper to stop immediately without decellerating. This could be
         *  useful in case of an emergency, but should be used with caution as the position of the stepper would be lost. Back-EMF could also cause damage to the stepper motor or the driver when stopping abruptly like this!
         *
         * Note that the processing time of this ISR is not accounted for when calculating when to process each steppers state the next time. So it will always run a bit slower than the calculated
         * speed, which could cause issues at very high speeds. A possible solution to this would be to measure the processing time of the ISR and subtract it from the timer delay when calculating
         * the next timer delay. This would allow for more accurate timing, but would also add some overhead to the processing of each stepper state.
         */
        void IRAM_ATTR busyLoop()
        {
            for (;;)
            {
                _iterationCounter++;
                uint32_t frameStart = ESP.getCycleCount();

                for (int i = 0; i < NUM_STEPPERS; i++)
                {
                    auto &stepper = _stepperData[i];

                    // uint32_t phaseStart = ESP.getCycleCount();

                    // Only process the stepper if it is pulsing or running, if it is idle or reversing we don't need to do anything in the timer interrupt
                    if (stepper.state == STEP_PULSING || stepper.state == STEP_RUNNING || stepper.state == STEP_EBRAKE)
                    {
                        // Only process the stepper when it's time
                        if (ESP.getCycleCount() - stepper.startTime >= stepper.delay)
                        {
                            switch (stepper.state)
                            {
                            case STEP_EBRAKE:
                                gpio_set_level((gpio_num_t)stepper.stepPin, LOW);
                                if(stepper.remainingSteps == 0)
                                {
                                    // If already stoppped, then just go idle
                                    stepper.state = STEP_IDLE;
                                }
                                else
                                {
                                    // Calculate how long it would take to decellerate from the current speed
                                    float stepsPerSecond = (float)CLOCK_FREQUENCY / (float)stepper.currentSpeed;
                                    float timeToStop = stepsPerSecond / (float)stepper.accelleration;

                                    // Make the stepper continue being active for however long it would take to brake with decelleration
                                    stepper.state = STEP_RUNNING;

                                    // Clearing the remaining steps will cause the stepper to stop being processed once it is scheduled for the next step
                                    stepper.remainingSteps = 0;
                                    stepper.delay = timeToStop * CLOCK_FREQUENCY;
                                }
                                break;
                            case STEP_PULSING:
                                gpio_set_level((gpio_num_t)stepper.stepPin, LOW);

                                stepper.state = STEP_RUNNING;

                                if (stepper.remainingSteps == 0)
                                {
                                    // During the last step we need to wait for the motor to catch up and come to a stop. It is presumed that this takes as long as the first step.
                                    gpio_set_level((gpio_num_t)stepper.stepPin, LOW);
                                    // stepper.startTime = phaseStart;
                                    stepper.delay = stepper.firstStepDuration;
                                    stepper.currentSpeed = stepper.firstStepDuration;
                                }
                                else if (stepper.currentSpeed == 0)
                                {
                                    // Always start out with the pre-calculated duration for the first step, it can't be derived from parameters from the previous step
                                    // stepper.startTime = phaseStart;
                                    stepper.delay = stepper.firstStepDuration;
                                    stepper.currentSpeed = stepper.firstStepDuration;
                                }
                                else
                                {
                                    if (stepper.currentSpeed < stepper.maxSpeed)
                                    {
                                        // TODO: break towards the max speed!!!!!!
                                        // TODO: break towards the max speed!!!!!!
                                        // TODO: break towards the max speed!!!!!!
                                        // temp implementation is to just drop to max speed without decellerating
                                        // stepper.startTime = phaseStart;
                                        stepper.delay = stepper.maxSpeed;
                                        stepper.currentSpeed = stepper.maxSpeed;
                                    }
                                    else
                                    {
                                        float stepPerSecond = float(CLOCK_FREQUENCY) / stepper.currentSpeed;
                                        float stepsToStop = (stepPerSecond * stepPerSecond) / (2.0f * stepper.accelleration);

                                        if (stepsToStop >= ((float)stepper.remainingSteps - 1.0f))
                                        {
                                            // Decellerate to stop
                                            float accelerationDuringPreviousStepInTicks = (float(CLOCK_FREQUENCY) * float(CLOCK_FREQUENCY)) / ((float)stepper.currentSpeed * (float)stepper.accelleration); // Accelleration during previous step in ticks
                                            float futureSpeedInTicks = ((float)stepper.currentSpeed * (float)accelerationDuringPreviousStepInTicks) / (accelerationDuringPreviousStepInTicks - stepper.currentSpeed);

                                            stepper.currentSpeed = floorf(futureSpeedInTicks);
                                            // stepper.startTime = phaseStart;
                                            stepper.delay = stepper.currentSpeed;
                                        }
                                        else if (stepper.currentSpeed > stepper.maxSpeed)
                                        {
                                            // Accellerate towards max speed
                                            float accelerationDuringPreviousStepInTicks = (float(CLOCK_FREQUENCY) * float(CLOCK_FREQUENCY)) / ((float)stepper.currentSpeed * (float)stepper.accelleration); // Accelleration during previous step in ticks
                                            float futureSpeedInTicks = ((float)stepper.currentSpeed * (float)accelerationDuringPreviousStepInTicks) / (accelerationDuringPreviousStepInTicks + (float)stepper.currentSpeed);

                                            // TODO: COMPENSATE FOR OVERSHOOTING AND GOING FASTER THAN THE MAX SPEED
                                            // TODO: COMPENSATE FOR OVERSHOOTING AND GOING FASTER THAN THE MAX SPEED
                                            // TODO: COMPENSATE FOR OVERSHOOTING AND GOING FASTER THAN THE MAX SPEED
                                            // TODO: COMPENSATE FOR OVERSHOOTING AND GOING FASTER THAN THE MAX SPEED

                                            stepper.currentSpeed = ceilf(futureSpeedInTicks);
                                            // stepper.startTime = phaseStart;
                                            stepper.delay = stepper.currentSpeed;
                                        }
                                        else
                                        {
                                            // Maintain max speed
                                            stepper.currentSpeed = stepper.maxSpeed;
                                            // stepper.startTime = phaseStart;
                                            stepper.delay = stepper.maxSpeed;
                                        }
                                    }

                                    if (stepper.delay <= stepper.pulseDuration)
                                    {
                                        // TODO: This needs to be some safe value...
                                        stepper.delay = 0;
                                    }
                                    else
                                    {
                                        stepper.delay -= stepper.pulseDuration; // subtract the pulse duration to account for the time the step pin is held high
                                    }
                                }

                                break;
                            case STEP_RUNNING:
                                if (stepper.remainingSteps == 0)
                                {
                                    // Flag the stepper is idle
                                    stepper.currentSpeed = 0;
                                    stepper.state = STEP_IDLE;
                                }
                                else
                                {
                                    gpio_set_level((gpio_num_t)stepper.dirPin, stepper.direction == 1 ? HIGH : LOW);
                                    gpio_set_level((gpio_num_t)stepper.stepPin, HIGH);
                                    stepper.startTime = ESP.getCycleCount();
                                    stepper.delay = stepper.pulseDuration;
                                    stepper.currentPosition += stepper.direction;
                                    stepper.remainingSteps--;
                                    stepper.state = STEP_PULSING;
                                }
                                break;
                            }
                        }
                    }
                }

                _timerIsrCycleSnapshot = ESP.getCycleCount() - frameStart;
            }
        }

    private:
        StepperData<CLOCK_FREQUENCY> _stepperData[NUM_STEPPERS];
        volatile uint32_t _timerIsrCycleSnapshot;
        volatile uint32_t _iterationCounter;
    };
}