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

import trclib.TrcPidController;
import trclib.TrcUtil;

public class RobotInfo
{
    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 52*12 + 5.25;
    public static final double FIELD_WIDTH                      = 26*12 + 11.25;

    // origin is on the initiation line, all the way to the left
    public static final double INITIATION_LINE_TO_ALLIANCE_WALL = 120;
    public static final double TARGET_X_POS = FIELD_WIDTH - 94.66;
    public static final double FEEDER_STATION_RIGHT_X_POS = 130.66;
    public static final double TRENCH_RUN_X_POS = FIELD_WIDTH - 27.75;
    public static final double LAST_TRENCH_BALL_Y_POS = -194.63;

    public static final double HIGH_TARGET_HEIGHT = 98.25;
    public static final double HIGH_VISION_TARGET_HEIGHT = 89.75;//TrcUtil.average(81.25, HIGH_TARGET_HEIGHT);
    public static final double PIVOT_HEIGHT = 23.5; // in from ground

    public static final double ROBOT_WIDTH = 36;
    public static final double ROBOT_LENGTH = 37;

    public static final double ROBOT_DRIVE_WIDTH = 21;
    public static final double ROBOT_DRIVE_LENGTH = 21;

    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    public static final double BATTERY_NOMINAL_VOLTAGE          = 12.0;

    //
    // Robot dimensions.
    //

    //
    // Joystick ports.
    //
    public static final int XBOX_DRIVERCONTROLLER               = 0;
    public static final int JSPORT_OPERATORSTICK                = 1;
    public static final int JSPORT_BUTTON_PANEL                 = 2;
    public static final int JSPORT_SWITCH_PANEL                 = 3;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONT_STEER = 23;    // 40A: Orange
    public static final int CANID_RIGHTFRONT_STEER = 24;    // 40A: Yellow
    public static final int CANID_LEFTREAR_STEER = 25;    // 40A: Green
    public static final int CANID_RIGHTREAR_STEER = 26;    // 40A: Blue

    public static final int CANID_LEFTFRONT_DRIVE = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONT_DRIVE = 4;    // 40A: Yellow
    public static final int CANID_LEFTREAR_DRIVE = 5;    // 40A: Green
    public static final int CANID_RIGHTREAR_DRIVE = 6;    // 40A: Blue

    public static final int CANID_INTAKE = 7;
    public static final int CANID_CONVEYOR = 8;
    public static final int CANID_SHOOTER_PITCH = 9;
    public static final int CANID_FLYWHEEL = 10;
    public static final int CANID_CLIMBER = 11;

    public static final int CANID_LEFT_LIDAR = 21;
    public static final int CANID_RIGHT_LIDAR = 22;

    public static final int CANID_PDP = 16;
    public static final int CANID_PCM = 17;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_LEFT_FRONT_WHEEL = 3;
    public static final int PDP_CHANNEL_RIGHT_FRONT_WHEEL = 0;
    public static final int PDP_CHANNEL_LEFT_BACK_WHEEL = 12;
    public static final int PDP_CHANNEL_RIGHT_BACK_WHEEL = 15;

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;

    //
    // Digital Input/Output ports.
    //
    public static final int CONVEYOR_PROXIMITY_SENSOR = 3;
    public static final int INTAKE_PROXIMITY_SENSOR = 2;
    public static final int DIO_LEFT_LIDAR = 3;
    public static final int DIO_RIGHT_LIDAR = 4;

    public static final int PWM_CHANNEL_LED = 0;
    public static final int NUM_LEDS = 60;

    //
    // Relay channels.
    //
    public static final int FLASHLIGHT_RELAY_CHANNEL = 0;

    //
    // Solenoid channels.
    //
    public static final int SOL_INTAKE_EXTEND = 0;
    public static final int SOL_INTAKE_RETRACT = 1;
    public static final int SOL_LATCH_EXTEND = 2;
    public static final int SOL_LATCH_RETRACT = 3;

    //
    // Vision subsystem.
    //
    public static final double CAMERA_Y_OFFSET = 12;  // in from pivot of arm + is forward
    public static final double CAMERA_Y_OFFSET_TO_PIVOT = 26;
    public static final double CAMERA_X_OFFSET = 0;    //Inches from pivot of arm to center of camera, + = right
    public static final double CAMERA_DATA_TIMEOUT              = 0.5;  //500ms
    public static final double CAMERA_CENTERED_THRESHOLD        = 2;    // +- 2 inches in x axis

    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    public static final double LIDAR_INTER_SENSOR_DIST = 19.375;
    public static final double LIDAR_SENSOR_Y_OFFSET = -3.5; // in from bumper edge. + is forward TODO: tune

    public static final double SHOOTER_BARREL_LENGTH = 30; // inches TODO: tune

    //
    // DriveBase subsystem.
    //

    public static final double STEER_DEGREES_PER_TICK = 360.0 / 4096.0;
    public static final double STEER_MAX_REQ_VEL = 1000.0; // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL = 5000; // deg/sec^2
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL = ((18700 * 0.81 / 56.67) / 60.0) * 360.0; // deg/sec
    public static final double STEER_MAX_VEL_TICKS_PER_100MS =
        (STEER_MAX_VEL / STEER_DEGREES_PER_TICK) / 10.0; // ticks/100ms

    // order is lf, rf, lr, rr
    public static final int[] STEER_ZEROS = new int[]{ 3551, 479, 3656, 1270 }; // this is a backup if the zeros file isn't found
    public static final TrcPidController.PidCoefficients magicSteerCoeff = new TrcPidController.PidCoefficients(2.0,
        0.01, 0, 1023.0 / STEER_MAX_VEL_TICKS_PER_100MS, 5.0 / STEER_DEGREES_PER_TICK);

    public static final double DRIVE_STALL_TIMEOUT              = 0.5;

    public static final double DRIVE_SLOW_SCALE                 = 0.5;
    public static final double DRIVE_SLOW_TURNSCALE             = 0.3;

    public static final double DRIVE_MEDIUM_SCALE               = 0.75;
    public static final double DRIVE_MEDIUM_TURNSCALE           = 0.6;

    public static final double DRIVE_FAST_SCALE                 = 1.0;
    public static final double DRIVE_FAST_TURNSCALE             = 0.8;

    public static final double DRIVE_GYRO_ASSIST_KP             = 1.5;
    public static final double DRIVE_MAX_ROTATION_RATE          = 6.5;      //radians per second

    public static final double DRIVE_MAX_XPID_POWER             = 0.5;
    public static final double DRIVE_MAX_YPID_POWER             = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE         = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE         = 0.6;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE      = 1.0;

    public static final double DRIVE_RAMP_RATE = 0.2;

    public static final double ENCODER_INCHES_PER_COUNT = 2.2421;
    public static final double ENCODER_KP = 0.011;
    public static final double ENCODER_KI = 0.0;
    public static final double ENCODER_KD = 0.0013;
    public static final double ENCODER_KF = 0.0;
    public static final double ENCODER_TOLERANCE = 2.0;

    public static final double ROBOT_TOP_SPEED = 300; // in/sec

    public static final double ROBOT_MAX_REQ_SPEED = 0.2 * ROBOT_TOP_SPEED;
    public static final double ROBOT_MAX_ACCEL = 200;

    public static final double PURE_PURSUIT_FOLLOWING_DISTANCE = 10;
    public static final double PURE_PURSUIT_POS_TOLERANCE = 2.0;
    public static final double PURE_PURSUIT_HEADING_TOLERANCE = 2.0;
    public static final double PURE_PURSUIT_KF = 1.0 / ROBOT_TOP_SPEED;
    public static final double PURE_PURSUIT_MOVE_OUTPUT_LIMIT = 0.6;

    public static final double GYRO_TURN_KP_BIG = 0.0055;
    public static final double GYRO_TURN_KD_BIG = 0.0007;

    public static final double GYRO_TURN_KP                     = 0.013;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 1.0;

    //
    // Shooter
    //
    public static final double SHOOTER_BOTTOM_POS = -10;

    public static final double FLYWHEEL_DIAMETER = 4.0; // in
    public static final double FLYWHEEL_GEAR_RATIO = 18.0 / 12.0;
    public static final double FLYWHEEL_INCHES_PER_TICK = Math.PI * FLYWHEEL_DIAMETER / FLYWHEEL_GEAR_RATIO / 2048.0; // falcon has 2048 cpr encoder
    public static final double FLYWHEEL_TOP_SPEED = 720;

    public static final double FLYWHEEL_LOW_ANGLE = 0; // angle when right up against goal
    public static final double FLYWHEEL_LOW_SPEED = 200;
    public static final double FLYWHEEL_HIGH_ANGLE = 70; // angle when right up against wall
    public static final double FLYWHEEL_HIGH_SPEED = 406.7;
}   // class RobotInfo
