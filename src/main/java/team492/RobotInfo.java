/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

public class RobotInfo {
    //
    // Field dimensions in inches.
    //

    //
    // Robot dimensions in inches.
    //

    //
    // Joystick ports.
    //
    public static final int XBOX_DRIVERCONTROLLER               = 0;
    public static final int XBOX_OPERATORCONTROLLER             = 1;

    //
    // CAN IDs.
    //
    public static final int CANID_ELEVATOR = 0;
    public static final int CANID_LEFTFRONTWHEEL                = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONTWHEEL               = 4;    // 40A: Yellow
    public static final int CANID_LEFTREARWHEEL                 = 5;    // 40A: Green
    public static final int CANID_RIGHTREARWHEEL                = 6;    // 40A: Blue

    //
    // PDP Channels.
    //

    //
    // Analog Input ports.
    //

    //
    // Digital Input/Output ports.
    //
    public static final int LIMITSWITCH_DIO = 0;

    //
    // Relay channels.
    //

    //
    // Solenoid channels.
    //
    public static final int PNEUMATIC_CANID = 1;
    public static final int PNEUMATIC_CH1 = 1;
    public static final int PNEUMATIC_CH2 = 2;

    //
    // Vision subsystem.
    //

    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT = 1.0/0.0098; //9.8mV per
    // inch
    // public static final double SONAR_ERROR_THRESHOLD = 50.0; //value should not
    // jump 50-in per time slice.

    //
    // DriveBase subsystem.
    //
    public static final double DRIVE_STALL_TIMEOUT = 0.5;

    public static final double ENCODER_X_INCHES_PER_COUNT = 1.6577438;
    public static final double ENCODER_X_KP = 0.017;
    public static final double ENCODER_X_KI = 0.0;
    public static final double ENCODER_X_KD = 0.0;
    public static final double ENCODER_X_KF = 0.0;
    public static final double ENCODER_X_TOLERANCE = 2.0;

    public static final double ENCODER_X_KP_SMALL = 0.022;
    public static final double ENCODER_X_KI_SMALL = 0.0;
    public static final double ENCODER_X_KD_SMALL = 0.0;
    public static final double ENCODER_X_KF_SMALL = 0.0;
    public static final double ENCODER_X_TOLERANCE_SMALL = 1.5;

    public static final double ENCODER_Y_INCHES_PER_COUNT = 2.355935875;
    public static final double ENCODER_Y_KP = 0.011;
    public static final double ENCODER_Y_KI = 0.0;
    public static final double ENCODER_Y_KD = 0.001;
    public static final double ENCODER_Y_KF = 0.0;
    public static final double ENCODER_Y_TOLERANCE = 2.0;

    public static final double GYRO_TURN_KP = 0.0055;
    public static final double GYRO_TURN_KI = 0.0;
    public static final double GYRO_TURN_KD = 0.00008;
    public static final double GYRO_TURN_KF = 0.0;
    public static final double GYRO_TURN_TOLERANCE = 2.0;

    //
    // Other subsystems.
    //
    public static final int SERVO1_PWM_PORT = 0;
    public static final int SERVO2_PWM_PORT = 1;
    public static final int RINGLIGHT_CHANNEL = 1;

    public static final double ELEVATOR_KP = 0.5;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.05; // just a placeholder
    public static final double ELEVATOR_TOLERANCE = 1.0; // inches
    public static final double ELEVATOR_INCHES_PER_COUNT = 1 / 800.0; // another placeholder
    public static final double ELEVATOR_CAL_POWER = 0.5;
    public static final double ELEVATOR_MIN_POS = 0.0;
    public static final double ELEVATOR_MAX_POS = 48.0; // inches
    public static final double ELEVATOR_POWER_COMP = 0.25; // constant power compensation factor, placeholder

} // class RobotInfo
