/*Libraries*/
#include "VarSpeedServo.h"

/*Global varibles*/
VarSpeedServo servo1, servo2, servo3, servo4, servo5;
const byte servo1Pin = 1, servo2Pin = 2, servo3Pin = 3, servo4Pin = 4, servo5Pin = 5;
const byte r1 = 120, r2 = 96, r3 = 29.5;

/*Setup*/
void setup() {
    servo1.attach(servo1Pin, 544, 2400);
    servo2.attach(servo2Pin, 544, 2400);
    servo3.attach(servo3Pin, 544, 2400);
    servo4.attach(servo4Pin, 544, 2400);
    servo5.attach(servo5Pin, 544, 2400);
}
void loop() {
}

/*Functions*/

/**
 * @brief Direct controle over the angles of the motors.
 * @param angle1 The angle of servo1 (0<=angle1<=180)
 * @param angle2 The angle of servo2 (34<=angle2<=180)
 * @param angle3 The angle of servo3 (54<=angle3<=180)
 * @param angle4 The angle of servo4 (0<=angle4<=180)
 * @param angle5 The angle of servo5 (0<=angle5<=180)
 * @param speed The speed of the motors (put 0 for max speed)
 * 
*/
void armSetAngle(byte angle1, byte angle2, byte angle3, byte angle4, byte angle5, byte speed) {
    servo1.write(angle1, speed);  //0 <=θ1<=180, default θ1=87
    servo2.write(angle2, speed);  //34<=θ2<=180, default θ2=115
    servo3.write(angle3, speed);  //54<=θ3<=180, default θ3=78
    servo4.write(angle4, speed);  //0 <=θ4<=180, default θ4=181
    servo5.write(angle5, speed);  //0 <=θ5<=180, default θ5=69
}

/**
 * @brief Set arm to default position.
 * @param speed The speed of the motors (put 0 for max speed)
*/
void armSetDefault(byte speed) {
    servo1.write(87, speed);
    servo2.write(115, speed);
    servo3.write(78, speed);
    servo4.write(181, speed);
    servo5.write(69, speed);
}

/**
 * @brief Get the angle of a motor.
 * @param motorNumber The number of the motors 0->5
*/
int getMotorAngle(byte motorNumber) {
    if (motorNumber == 1) {
        return servo1.read();
    } else if (motorNumber == 2) {
        return servo2.read();
    } else if (motorNumber == 3) {
        return servo3.read();
    } else if (motorNumber == 4) {
        return servo4.read();
    } else if (motorNumber == 5) {
        return servo5.read();
    }
}

/**
 * @brief Controle over the arm position.
 * @param xPos The x position of the end effector
 * @param yPos The y position of the end effector
 * @param zPos The z position of the end effector
 * @param pitchAngle The pitch angle of the end effector
 * @param attackAngle The attack angle of the end effector
 * @param speed The speed of the motors (put 0 for max speed)
 * 
*/
void armSetPosition(byte xPos, byte yPos, byte zPos, byte pitchAngle, byte attackAngle, byte speed) {
    float x = sqrt(xPos * xPos + yPos * yPos) - r3 * cos((float)attackAngle * PI / 180);
    float y = zPos - r3 * sin((float)attackAngle * PI / 180);

    float c = sqrt(x * x + y * y);
    float theta1 = atan2(yPos, xPos);
    float theta2 = atan2(y, x) + acos((float)(c * c + r1 * r1 - r2 * r2) / (2 * c * r1));
    float theta3 = atan2(y, x) - acos((float)(c * c + r2 * r2 - r1 * r1) / (2 * c * r2)) - theta2;
    float theta5 = attackAngle * PI / 180 - (theta3 + theta2);

    int angle1 = 87 + theta1 * 180 / PI;
    int angle2 = 245 - theta2 * 180 / PI;
    int angle3 = 210 + theta3 * 180 / PI;
    int angle4 = 68 - pitchAngle;
    int angle5 = 96 + theta5 * 180 / PI;

    servo1.write(angle1, speed);
    servo2.write(angle2, speed);
    servo3.write(angle3, speed);
    servo4.write(angle4, speed);
    servo5.write(angle5, speed);
}
