#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// ========= CONFIGURATION =========
#define PACKET_SIZE 13
#define SERVO_MIN_US 520 
#define SERVO_MAX_US 2480
#define SERVO_INIT_VAL 1500

// ========= PINS (ESP32) =========
const uint8_t PMW_A = 15, AIN_1 = 4, AIN_2 = 2;   // Motor A
const uint8_t PWM_B = 5,  BIN_1 = 16, BIN_2 = 17; // Motor B

// ========= GLOBALS =========
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
uint16_t servo_positions[6] = {1500, 1500, 1500, 1500, 1500, 1500};
const uint8_t servo_channels[6] = {1, 2, 3, 4, 5, 6};

// ========= PACKET STRUCTURE =========
// Using #pragma pack to ensure 1:1 mapping with the serial buffer
#pragma pack(push, 1)
typedef struct {
    uint8_t head0;    // 0xDE (222)
    uint8_t head1;    // 0xAD (173)
    uint8_t data[7];  // [joy_T, joy_X, joy_Y, joy_g, joy_R, joy_H, command]
    uint8_t crcH;     // CRC High Byte
    uint8_t crcL;     // CRC Low Byte
    uint8_t foot0;    // 0xBE (190)
    uint8_t foot1;    // 0xEF (239)
} RobotPacket;
#pragma pack(pop)

// ========= UTILITIES =========
uint16_t usToTicks(float us) {
    return (uint16_t)(us * 4096.0f / 20000.0f);
}

// CRC16-CCITT (Matches Python crccheck.crc.Crc16)
uint16_t calcCRC16(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF; // Standard CCITT starting seed
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

void setMotorSpeed(uint8_t motor, uint16_t speed) {
    // motor: 0=A, 1=B
    // speed: 0-255 (Forward), 256-511 (Backward)
    uint8_t pwr = (speed > 255) ? (speed - 256) : speed;
    bool forward = (speed <= 255);

    uint8_t p1 = (motor == 0) ? AIN_1 : BIN_1;
    uint8_t p2 = (motor == 0) ? AIN_2 : BIN_2;
    uint8_t en = (motor == 0) ? PMW_A : PWM_B;

    digitalWrite(p1, forward ? HIGH : LOW);
    digitalWrite(p2, forward ? LOW : HIGH);
    analogWrite(en, pwr);
}

void initial_pos() {
    for(int i=0; i<6; i++) {
        servo_positions[i] = SERVO_INIT_VAL;
        pwm.setPWM(servo_channels[i], 0, usToTicks(SERVO_INIT_VAL));
    }
    Serial.println("Servos Reset to Center");
}

// ========= DATA HANDLER =========
void data_handler(RobotPacket *p) {
    // p->data[0]: joy_T, [1]: joy_X, [2]: joy_Y, [3]: joy_g, [4]: joy_R, [5]: joy_H, [6]: command

    // 1. Handle Incremental Servos (Base, Shoulder, Elbow, Tilt)
    // Mapping packet data to servo_positions array


    for (int i = 0; i < 4; i++) {
      int s_idx = (i == 3) ? 4 : i;
      if (i == 0 || i == 4) {
        if (p->data[i] == 1)      servo_positions[s_idx] += 30;
        else if (p->data[i] == 2) servo_positions[s_idx] -= 30;
      } 
      else{
        if (p->data[i] == 1)      servo_positions[s_idx] += 10;
        else if (p->data[i] == 2) servo_positions[s_idx] -= 10;
      }
      servo_positions[s_idx] = constrain(servo_positions[s_idx], SERVO_MIN_US, SERVO_MAX_US);
      pwm.setPWM(servo_channels[s_idx], 0, usToTicks(servo_positions[s_idx]));
    }

    // 2. Handle Gripper (Binary Open/Close) - data[4] (joy_R)
    if (p->data[4] == 2)      servo_positions[5] = 2000; // Close
    else if (p->data[4] == 1) servo_positions[5] = 1000; // Open
    pwm.setPWM(servo_channels[5], 0, usToTicks(servo_positions[5]));

    // 3. Reset Button - data[5] (joy_H)
    if (p->data[5] == 1) initial_pos();

    // 4. DC Motors - data[6] (command)
    switch (p->data[6]) {
        case 0x00: Serial.println("Forward: || |");  break;
        case 0x01: Serial.println("Forward");        setMotorSpeed(0, 255); setMotorSpeed(1, 511); break;
        case 0x02: Serial.println("Forward: | ||");  setMotorSpeed(0, 511); setMotorSpeed(1, 255); break;
        case 0x03: Serial.println("->");             setMotorSpeed(0, 511); setMotorSpeed(1, 511); break;
        case 0x04: Serial.println("Stop");           setMotorSpeed(0, 0);   setMotorSpeed(1, 0);   break;
        case 0x05: Serial.println("<-");             setMotorSpeed(0, 255); setMotorSpeed(1, 255); break;
        case 0x06: Serial.println("Backward: | ||"); setMotorSpeed(0, 511); setMotorSpeed(1, 511); break;
        case 0x07: Serial.println("Backward");       setMotorSpeed(0, 511); setMotorSpeed(1, 255); break;
        case 0x08: Serial.println("Backward: || |"); setMotorSpeed(0, 255); setMotorSpeed(1, 0);   break;
        default: break;
    }
}

// ========= MAIN LOOPS =========
void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(50);

    pinMode(AIN_1, OUTPUT); pinMode(AIN_2, OUTPUT); pinMode(PMW_A, OUTPUT);
    pinMode(BIN_1, OUTPUT); pinMode(BIN_2, OUTPUT); pinMode(PWM_B, OUTPUT);

    delay(100);
    initial_pos();
    Serial.println("ESP32 Robot Controller Online");
}

void loop() {
    static uint8_t buffer[PACKET_SIZE];
    static int index = 0;

    while (Serial.available()) {
        uint8_t byteIn = Serial.read();

        // 1. Find Header
        if (index == 0 && byteIn != 0xDE) continue;
        if (index == 1 && byteIn != 0xAD) { index = 0; continue; }

        buffer[index++] = byteIn;

        // 2. Once Full Packet Received
        if (index >= PACKET_SIZE) {
            RobotPacket* p = (RobotPacket*)buffer;

            // 3. Verify Footer
            if (p->foot0 == 0xBE && p->foot1 == 0xEF) {
                // 4. Check CRC (on first 9 bytes: Header + Data)
                uint16_t calc = calcCRC16(buffer, 9);
                uint16_t received = (p->crcH << 8) | p->crcL;
                
                // uint8_t sub[9] = {222, 173, 0,0,0,0,0,0,4};
                // uint16_t test_crc = calcCRC16(sub, 9);

                // Serial.print("Test CRC for {0xDE,0xAD,0,0,0,0,0,0,4}: "); 
                // Serial.println(test_crc, HEX);

                data_handler(p);
                if (calc == received) {
                    data_handler(p);
                } else {
                    Serial.print("CRC Error! Calc: "); Serial.print(calc, HEX);
                    Serial.print(" Recv: "); Serial.println(received, HEX);
                }
            } else {
                Serial.println("Footer Mismatch");
            }
            index = 0; // Reset for next packet
        }
    }
}