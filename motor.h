#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    enum Directions {
        Direction_Forward,
        Direction_Backward
    };

    Motor(uint8_t pin, uint8_t pwmPin, bool reverseDirection = false);
    void setSpeed(uint8_t speed);
    void setDirection(Directions direction);
    
    void forward();
    void backward();
    void stop();

    String name;

private:
    uint8_t m_pin;
    uint8_t m_pwmPin;
    uint8_t m_forwardCommand;
    uint8_t m_backwardCommand;
    bool m_reverseDirection;
};

#endif