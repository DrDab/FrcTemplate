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

    public static final double DRIVE_SLOW_XSCALE = 0.5;
    public static final double DRIVE_SLOW_YSCALE = 0.5;
    public static final double DRIVE_SLOW_TURNSCALE = 0.4;

    public static final double DRIVE_MEDIUM_XSCALE = 0.75;
    public static final double DRIVE_MEDIUM_YSCALE = 0.75;
    public static final double DRIVE_MEDIUM_TURNSCALE = 0.6;

    public static final double DRIVE_FAST_XSCALE = 1.0;
    public static final double DRIVE_FAST_YSCALE = 1.0;
    public static final double DRIVE_FAST_TURNSCALE = 0.8;

    public static final double DRIVE_GYRO_ASSIST_KP = 1.5;
    public static final double DRIVE_MAX_ROTATION_RATE = 6.5; // radians per second

    public static final double DRIVE_MAX_XPID_POWER = 0.5;
    public static final double DRIVE_MAX_YPID_POWER = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE = 0.6;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE = 1.0;

    // practice robot: 0.012/0.0/0.0
    public static final double ENCODER_X_INCHES_PER_COUNT = 1.6577438;
    public static final double ENCODER_X_KP = 0.017;
    public static final double ENCODER_X_KI = 0.0;
    public static final double ENCODER_X_KD = 0.0;
    public static final double ENCODER_X_KF = 0.0;
    public static final double ENCODER_X_TOLERANCE = 2.0;

    // practice robot: 0.024/0.0/0.0
    public static final double ENCODER_X_KP_SMALL = 0.022;
    public static final double ENCODER_X_KI_SMALL = 0.0;
    public static final double ENCODER_X_KD_SMALL = 0.0;
    public static final double ENCODER_X_KF_SMALL = 0.0;
    public static final double ENCODER_X_TOLERANCE_SMALL = 1.5;

    // comp robot: 0.02/0.0/0.0016
    // practice robot: 0.01/0.0/0.001
    public static final double ENCODER_Y_INCHES_PER_COUNT = 2.355935875;
    public static final double ENCODER_Y_KP = 0.011;
    public static final double ENCODER_Y_KI = 0.0;
    public static final double ENCODER_Y_KD = 0.001;
    public static final double ENCODER_Y_KF = 0.0;
    public static final double ENCODER_Y_TOLERANCE = 2.0;

    // Comp robot: 0.015/0.0/0.001
    // practice robot: 0.008/0.0/0.0007
    // 3/24 comp robot: 0.0055/0.0/0.00008
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

} // class RobotInfo
