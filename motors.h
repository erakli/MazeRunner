#ifndef MOTORS_H
#define MOTORS_H

enum Moves {
    Move_Stop,
    Move_Forward,
    Move_Backwards,
    Move_Left,
    Move_Right
};


class Motor {
public:
    enum Directions {
        Direction_Forward,
        Direction_Backward
    };

    Motor(uint8_t pin, uint8_t pwmPin, bool reverseDirection = false);
    void SetSpeed(uint8_t speed);
    void SetDirection(Directions direction);
    
    void Forward();
    void Backward();
    void Stop();
private:
    uint8_t pin;
    uint8_t pwmPin;
    uint8_t forwardCommand;
    uint8_t backwardCommand;
};

enum Motors {
    Motor_Right = 0,
    Motor_Left = 1
};

extern Motor g_motors[];

void MotorsSetDirection(Motor::Directions rightMotor, Motor::Directions leftMotor);
void MotorsSetSpeed(uint8_t rightMotor, uint8_t leftMotor);
void MotorsStop();
void MotorsSetMove(Moves moveCommand, uint8_t speedRight, uint8_t speedLeft);


#endif