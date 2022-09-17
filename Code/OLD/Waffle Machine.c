// Waffle Machine code

#include <TMCStepper.h>
#include <Servo.h>

// Pins for Einsy Rambo controller
// https://reprap.org/wiki/EinsyRambo_development

// Waffle configuration
#define WAFFLE_COOK_TIME 30 // Seconds it takes to make a waffle

// Pump configuration
#define PUMP_ON_TIME        // Time pump is on

// SPI communication
#define SW_MISO         50 // Software Master In Slave Out (MISO)
#define SW_MOSI         51 // Software Master Out Slave In (MOSI)
#define SW_SCK          52 // Software Slave Clock (SCK)

// Servo configuration
Servo servo;
int angle = 30;
int servoDelay = 100;
#define SERVO_PIN        8 // Servo pin (Fan 1)

// Motor configuration

// E0-motor
#define CS_E0_PIN       A12 // Chip select E0-motor
#define EN_E0_PIN       26  // Enable pin E0-motor
#define DIR_E0_PIN      43  // Direction pin E0-motor
#define STEP_E0_PIN     34  // Step pin E0-motor
#define E0_MAX_SPEED    40  // E0-motor max speed
#define E0_MIN_SPEED  1000  // E0-motor min speed
int E0_DIRECTION = 0;       // Direction E0-motor
int E0_STALL_VALUE = 5;     // Stall value E0-motor

// Z-motors
#define CS_Z_PIN        A13 // Chip select Z-motors
#define EN_Z_PIN        27  // Enable pin Z-motors
#define DIR_Z_PIN       47  // Direction pin Z-motors
#define STEP_Z_PIN      35  // Step pin Z-motors
#define Z_MAX_SPEED    40  // Z-motors max speed
#define Z_MIN_SPEED  1000  // Z-motors min speed
int Z_DIRECTION = 0;        // Direction Z-motors
int Z_STALL_VALUE = 5;      // Stall value Z-motors

// Y-motor
#define CS_Y_PIN        39 // Chip select Y-motor
#define EN_Y_PIN        28 // Enable pin Y-motor
#define DIR_Y_PIN       48 // Direction pin Y-motor
#define STEP_Y_PIN      36 // Step pin Y-motor
#define Y_MAX_SPEED    40  // Y-motor max speed
#define Y_MIN_SPEED  1000  // Y-motor min speed
int Y_DIRECTION = 0;       // Direction Y-motor
int Y_STALL_VALUE = 5;     // Stall value Y-motor

// X-motor
#define CS_X_PIN        41 // Chip select X-motor
#define EN_X_PIN        29 // Enable pin X-motor
#define DIR_X_PIN       49 // Direction pin X-motor
#define STEP_Y_PIN      37 // Step pin X-motor
#define X_MAX_SPEED    40  // X-motor max speed
#define X_MIN_SPEED  1000  // X-motor min speed
int X_DIRECTION = 0;       // Direction X-motor
int X_STALL_VALUE = 5;     // Stall value X-motor

// Button pins
int MAKE_WAFFLE_PIN   = 11;
int EMERGENCY_BUTTON_PIN = 12;

// TMC2130 driver setup
#define R_SENSE 0.11f // R_sense resistor for SilentStepStick2130
using namespace TMC2130_n;
TMC2130Stepper E0_DRIVER(EN_E0_PIN, R_SENSE);
TMC2130Stepper Z_DRIVER(EN_Z_PIN, R_SENSE);
TMC2130Stepper Y_DRIVER(EN_Y_PIN, R_SENSE);
TMC2130Stepper X_DRIVER(EN_X_PIN, R_SENSE);

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

ISR(TIMER1_COMPA_vect) {
        //STEP_PORT ^= 1 << STEP_BIT_POS;
        digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}

// Setup
void setup() {

    // SPI setup
    SPI.begin();
    pinMode(MISO, INPUT_PULLUP);
    pinMode(SW_MISO, INPUT_PULLUP);

    // Serial setup
    Serial.begin(9600);

    // Servo setup
    servo.attach(SERVO_PIN);

    // Motor setup

    // E0-motor setup
    pinMode(CS_E0_PIN, OUTPUT);
    pinMode(EN_E0_PIN, OUTPUT);
    pinMode(DIR_E0_PIN, OUTPUT);
    pinMode(STEP_E0_PIN, OUTPUT);

    // Z-motors setup
    pinMode(CS_Z_PIN, OUTPUT);
    pinMode(EN_Z_PIN, OUTPUT);
    pinMode(DIR_Z_PIN, OUTPUT);
    pinMode(STEP_Z_PIN, OUTPUT);

    // Y-motor setup
    pinMode(CS_Y_PIN, OUTPUT);
    pinMode(EN_y_PIN, OUTPUT);
    pinMode(DIR_y_PIN, OUTPUT);
    pinMode(STEP_y_PIN, OUTPUT);

    // E0-motor setup
    pinMode(CS_X_PIN, OUTPUT);
    pinMode(EN_X_PIN, OUTPUT);
    pinMode(DIR_X_PIN, OUTPUT);
    pinMode(STEP_X_PIN, OUTPUT);

    // Button setup
    pinMode(GRN_PIN, INPUT_PULLUP);
    pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP);

    digitalWrite(EN_PIN, LOW);

    // Setup E0-motor-driver
    E0_DRIVER.begin();
    E0_DRIVER.toff(4);
    E0_DRIVER.blank_time(24);
    E0_DRIVER.rms_current(400); // Current in mA
    E0_DRIVER.microsteps(8);
    E0_DRIVER.en_pwm_mode(true);
    E0_DRIVER.pwm_autoscale(true);
    E0_DRIVER.TCOOLTHRS(0xFFFFF); // 20bit max
    E0_DRIVER.THIGH(0);
    E0_DRIVER.semin(5);
    E0_DRIVER.semax(2);
    E0_DRIVER.sedn(0b01);
    E0_DRIVER.sgt(STALL_VALUE);
    delayMicroseconds(160);

    // Setup Z-motors-driver
    Z_DRIVER.begin();
    Z_DRIVER.toff(4);
    Z_DRIVER.blank_time(24);
    Z_DRIVER.rms_current(400); // Current in mA
    Z_DRIVER.microsteps(8);
    Z_DRIVER.en_pwm_mode(true);
    Z_DRIVER.pwm_autoscale(true);
    Z_DRIVER.TCOOLTHRS(0xFFFFF); // 20bit max
    Z_DRIVER.THIGH(0);
    Z_DRIVER.semin(5);
    Z_DRIVER.semax(2);
    Z_DRIVER.sedn(0b01);
    Z_DRIVER.sgt(Z_STALL_VALUE);
    delayMicroseconds(160);

    // Setup Y-motor-driver
    Y_DRIVER.begin();
    Y_DRIVER.toff(4);
    Y_DRIVER.blank_time(24);
    Y_DRIVER.rms_current(400); // Current in mA
    Y_DRIVER.microsteps(8);
    Y_DRIVER.en_pwm_mode(true);
    Y_DRIVER.pwm_autoscale(true);
    Y_DRIVER.TCOOLTHRS(0xFFFFF); // 20bit max
    Y_DRIVER.THIGH(0);
    Y_DRIVER.semin(5);
    Y_DRIVER.semax(2);
    Y_DRIVER.sedn(0b01);
    Y_DRIVER.sgt(Y_STALL_VALUE);
    delayMicroseconds(160);

    // Setup X-motor-driver
    X_DRIVER.begin();
    X_DRIVER.toff(4);
    X_DRIVER.blank_time(24);
    X_DRIVER.rms_current(400); // Current in mA
    X_DRIVER.microsteps(8);
    X_DRIVER.en_pwm_mode(true);
    X_DRIVER.pwm_autoscale(true);
    X_DRIVER.TCOOLTHRS(0xFFFFF); // 20bit max
    X_DRIVER.THIGH(0);
    X_DRIVER.semin(5);
    X_DRIVER.semax(2);
    X_DRIVER.sedn(0b01);
    X_DRIVER.sgt(X_STALL_VALUE);
    delayMicroseconds(160);

    // Setup E0-motor-driver
    E0_DRIVER.begin();
    E0_DRIVER.toff(4);
    E0_DRIVER.blank_time(24);
    E0_DRIVER.rms_current(400); // Current in mA
    E0_DRIVER.microsteps(8);
    E0_DRIVER.en_pwm_mode(true);
    E0_DRIVER.pwm_autoscale(true);
    E0_DRIVER.TCOOLTHRS(0xFFFFF); // 20bit max
    E0_DRIVER.THIGH(0);
    E0_DRIVER.semin(5);
    E0_DRIVER.semax(2);
    E0_DRIVER.sedn(0b01);
    E0_DRIVER.sgt(E0_STALL_VALUE);
    delayMicroseconds(160);

    // Set stepper interrupt
    {
        cli(); // Stop interrupts
        TCCR1A = 0; // Set entire TCCR1A register to 0
        TCCR1B = 0; // Same for TCCR1B
        TCNT1  = 0; // Initialize counter value to 0
        OCR1A = 256;  // = (16*10^6) / (1*1024) - 1 (must be <65536)
        // turn on CTC mode
        TCCR1B |= (1 << WGM12);
        // Set CS11 bits for 8 prescaler
        TCCR1B |= (1 << CS11);// | (1 << CS10);
        // enable timer compare interrupt
        TIMSK1 |= (1 << OCIE1A);
        sei();//allow interrupts
    }

    // Current state
    int currentState = STANDBY;

    // Last known button input
    int previousButtonState = LOW;

}

// Loop
void loop() {
    switch (currentState) {
        case STANDBY: {
            Serial.println("Standby");
            if (getButtonFallingEdge(MAKE_WAFFLE_PIN)) {
                currentState = OPEN_IRON;
            }
            break;
        }

        case OPEN_IRON: {
            Serial.println("Opening waffle iron.");
            digitalWrite(EN_Z_PIN, HIGH);
            digitalWrite(EN_X_PIN, HIGH);


            currentState = MOVE_TUBE_TO_POSISTION;
            break;
        }

        case MOVE_TUBE_TO_POSITION: {
            Serial.println("Moving servo to position.");
            servo.write(angle);
            delay(servoDelay);
            Serial.println("Servo is in position.");
            currentState = DISPENSE_WAFFLE_MIXTURE;
            break;
        }

        case DISPENSE_WAFFLE_MIXTURE: {
            Serial.println("Dispensing waffle mixture.");
            digitalWrite(EN_E0_PIN, HIGH);


            Serial.println("Waffle mixture has been dispensed.");
            currentState = MOVE_TUBE_BACK_TO_POSITION;
            break;
        }

        case MOVE_TUBE_BACK_TO_POSITION: {
            Serial.println("Moving servo to default position.");
            servo.write(0);
            delay(servoDelay);
            Serial.println("Servo is in default position.");
            currentState = COOK_WAFFLE;
            break;
        }

        case CLOSE_IRON: {
            Serial.println("Cooking waffle.");
            digitalWrite(EN_Z_PIN, HIGH);

            break;
        }

        case COOK_WAFFLE: {
            Serial.println("Cooking waffle.");
            digitalWrite(EN_Z_PIN, HIGH);


            delay(WAFFLE_COOK_TIME);
            currentState = OPEN_IRON;
            break;
        }

        case WAFFLE_DONE: {
            Serial.println("Waffle is done.");


            currentState = OPEN_IRON_DONE;
            break;
        }
        case OPEN_IRON_DONE: {
            Serial.println("Opening waffle iron.");


            currentState = OPEN_IRON;
            break;
        }
        case DISPENSE_WAFFLE: {
            Serial.print("Dispensing waffle.");


            currentState = STANDBY;
            break;
        }
        case EMERGENCY: {
            EN_E0_PIN = LOW;
            EN_Z_PIN = LOW;
            EN_Y_PIN = LOW;
            EN_X_PIN = LOW;
            Serial.println("Emergency button has been pressed!");
            Serial.println("Please power cycle to restore function.");
        }
    }

    // Emergency button check
    if (digitalRead(EMERGENCY_BUTTON_PIN) == HIGH) {
        currentState = EMERGENCY;
    }

    // Function to get "falling edge" of button.
    bool getButtonFallingEdge(int MAKE_WAFFLE_PIN) {
        bool fallingEdge = false;
        int newButtonInput = digitalRead(MAKE_WAFFLE_PIN);
        if ((previousButtonState == HIGH) && (newButtonInput == LOW))
        {
            fallingEdge = true;
        }
        previousButtonState = newButtonInput;
        return fallingEdge;
    }
}

