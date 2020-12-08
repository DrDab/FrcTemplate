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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import frclib.FrcRobotBase;
import frclib.FrcXboxController;
import frclib.FrcServo;
import frclib.FrcAHRSGyro;
import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;
import frclib.FrcDigitalInput;
import frclib.FrcPneumatic;

import hallib.HalDashboard;
import trclib.TrcDigitalInput;
import trclib.TrcEnhancedServo;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot.RunMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends FrcRobotBase 
{
        //
        // Robot preferences.
        //

        //
        // Global constants.
        //
        public static final String programName = "FrcTemplate";

        //
        // Global objects.
        //
        public final DriverStation ds = DriverStation.getInstance();
        public final HalDashboard dashboard = HalDashboard.getInstance();
        public TrcPidController.PidCoefficients tunePidCoeff;

        //
        // Inputs.
        //
        public FrcXboxController operatorXboxController;
        public FrcXboxController driverXboxController;
        public FrcAHRSGyro gyro = null;

        //
        // Sensors.
        //
        public TrcDigitalInput elevatorLowerLimitSwitch;
        public FrcCANTalon elevatorMotor;
        public TrcPidController elevatorPidController;

        //
        // DriveBase subsystem.
        //
        public FrcCANSparkMax leftFrontWheel;
        public FrcCANSparkMax leftRearWheel;
        public FrcCANSparkMax rightFrontWheel;
        public FrcCANSparkMax rightRearWheel;
        public TrcMecanumDriveBase driveBase;

        public TrcPidController encoderXPidCtrl;
        public TrcPidController encoderYPidCtrl;
        public TrcPidController gyroTurnPidCtrl;
        public TrcPidDrive pidDrive;

        //
        // Vision subsystem.
        //

        //
        // Miscellaneous subsystem.
        //
        public FrcServo servo1;
        public FrcServo servo2;
        public TrcEnhancedServo enhancedServo;
        public FrcPneumatic pneumatic;
        public TrcPidActuator elevator;

        //
        // FMS Match info.
        //

        private FrcAuto autoMode;

        /**
         * Constructor.
         */
        public Robot() 
        {
                super(programName);
        } // Robot

        /**
         * This function is run when the robot is first started up and should be used
         * for any initialization code.
         */
        @Override
        public void robotInit() 
        {
                //
                // Create and initialize global objects.
                //

                //
                // Create and initialize inputs.
                //
                driverXboxController = new FrcXboxController("DriverController", RobotInfo.XBOX_DRIVERCONTROLLER);
                operatorXboxController = new FrcXboxController("OperatorController", RobotInfo.XBOX_OPERATORCONTROLLER);

                //
                // Create and initialize sensors.
                //
                gyro = new FrcAHRSGyro("NavX", SPI.Port.kMXP);

                //
                // Create and initialize DriveBase subsystem.
                //
                leftFrontWheel = new FrcCANSparkMax("LeftFrontWheel", RobotInfo.CANID_LEFTFRONTWHEEL, true);
                leftRearWheel = new FrcCANSparkMax("LeftRearWheel", RobotInfo.CANID_LEFTREARWHEEL, true);
                rightFrontWheel = new FrcCANSparkMax("RightFrontWheel", RobotInfo.CANID_RIGHTFRONTWHEEL, true);
                rightRearWheel = new FrcCANSparkMax("RightRearWheel", RobotInfo.CANID_RIGHTREARWHEEL, true);

                driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel,
                                gyro);
                driveBase.setOdometryScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);

                //
                // Create PID controllers for DriveBase PID drive.
                //
                encoderXPidCtrl = new TrcPidController("encoderXPidCtrl",
                                new TrcPidController.PidCoefficients(RobotInfo.ENCODER_X_KP_SMALL,
                                                RobotInfo.ENCODER_X_KI_SMALL, RobotInfo.ENCODER_X_KD_SMALL,
                                                RobotInfo.ENCODER_X_KF_SMALL),
                                RobotInfo.ENCODER_X_TOLERANCE_SMALL, driveBase::getXPosition);
                encoderYPidCtrl = new TrcPidController("encoderYPidCtrl",
                                new TrcPidController.PidCoefficients(RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI,
                                                RobotInfo.ENCODER_Y_KD, RobotInfo.ENCODER_Y_KF),
                                RobotInfo.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
                gyroTurnPidCtrl = new TrcPidController("gyroTurnPidCtrl",
                                new TrcPidController.PidCoefficients(RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI,
                                                RobotInfo.GYRO_TURN_KD, RobotInfo.GYRO_TURN_KF),
                                RobotInfo.GYRO_TURN_TOLERANCE, driveBase::getHeading);
                gyroTurnPidCtrl.setAbsoluteSetPoint(true);

                pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
                pidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
                pidDrive.setMsgTracer(globalTracer);

                //
                // Create and initialize Vision subsystem.
                //

                //
                // Create and initialize other subsystems.
                //
                this.servo1 = new FrcServo("servo1", RobotInfo.SERVO1_PWM_PORT);
                this.servo2 = new FrcServo("servo2", RobotInfo.SERVO2_PWM_PORT);
                this.servo2.setInverted(true);
                this.enhancedServo = new TrcEnhancedServo("enhancedServo", this.servo1, this.servo2);

                this.pneumatic = new FrcPneumatic("pneumatic", RobotInfo.PNEUMATIC_CANID, RobotInfo.PNEUMATIC_CH1, RobotInfo.PNEUMATIC_CH2);

                this.elevatorLowerLimitSwitch = new FrcDigitalInput("elevatorLowerLimitSwitch", RobotInfo.LIMITSWITCH_DIO);
                this.elevatorMotor = new FrcCANTalon("elevatorMotor", RobotInfo.CANID_ELEVATOR);

                this.elevatorPidController = new TrcPidController("elevatorPidController",
                                new TrcPidController.PidCoefficients(RobotInfo.ELEVATOR_KP, RobotInfo.ELEVATOR_KI,
                                                RobotInfo.ELEVATOR_KD),
                                RobotInfo.ELEVATOR_TOLERANCE, this.elevator::getPosition);
                this.elevator = new TrcPidActuator("elevator", this.elevatorMotor, this.elevatorLowerLimitSwitch,
                                this.elevatorPidController, RobotInfo.ELEVATOR_CAL_POWER, RobotInfo.ELEVATOR_MIN_POS,
                                RobotInfo.ELEVATOR_MAX_POS);
                this.elevator.setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_COUNT);

                //
                // AutoAssist commands.
                //

                //
                // Create Robot Modes.
                //
                autoMode = new FrcAuto(this);
                setupRobotModes(new FrcTeleOp(this), autoMode, new FrcTest(this), new FrcDisabled(this));

        } // robotInit

        public double getElevatorPowerComp() 
        {
                return RobotInfo.ELEVATOR_POWER_COMP;
        }

        public void robotStartMode(RunMode runMode, RunMode prevMode) 
        {
                //
                // Start subsystems.
                //
                setOdometryEnabled(true);

                //
                // Read FMS Match info.
                //

                //
                // Read Tune PID Coefficients if in TEST_MODE.
                //

                //
                // Start trace logging.
                //

        } // robotStartMode

        public void robotStopMode(RunMode runMode, RunMode nextMode)
        {
                //
                // Stop subsystems.
                //
                setOdometryEnabled(false);

                //
                // Stop trace logging.
                //

        } // robotStopMode

        public void setOdometryEnabled(boolean enabled) 
        {
                this.driveBase.setOdometryEnabled(enabled);
                this.elevatorMotor.setOdometryEnabled(enabled);
        }

} // class Robot
