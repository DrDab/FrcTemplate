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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import common.CmdPidDrive;
import common.CmdTimedDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frclib.FrcChoiceMenu;
import frclib.FrcColor;
import frclib.FrcJoystick;
import frclib.FrcRemoteVisionProcessor;
import frclib.FrcXboxController;
import trclib.TrcEvent;
import trclib.TrcRobot.RunMode;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = "FrcTest";

    public enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_DISTANCE_DRIVE,
        Y_DISTANCE_DRIVE,
        TURN_DEGREES,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        LIVE_WINDOW
    }   // enum Test

    private enum State
    {
        START,
        DONE
    }   // State

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    //
    // Test choice menu.
    //
    private FrcChoiceMenu<Test> testMenu;
    private Test test;

    private CmdTimedDrive timedDriveCommand = null;
    private CmdPidDrive pidDriveCommand = null;

    private int motorIndex = 0;

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        //
        // Create and populate Test Mode specific menus.
        //
        testMenu = new FrcChoiceMenu<>("Test/Tests");
        testMenu.addChoice("Sensors Test", FrcTest.Test.SENSORS_TEST, true, false);
        testMenu.addChoice("Subsystems Test", FrcTest.Test.SUBSYSTEMS_TEST);
        testMenu.addChoice("Drive Motors Test", FrcTest.Test.DRIVE_MOTORS_TEST);
        testMenu.addChoice("X Timed Drive", FrcTest.Test.X_TIMED_DRIVE);
        testMenu.addChoice("Y Timed Drive", FrcTest.Test.Y_TIMED_DRIVE);
        testMenu.addChoice("X Distance Drive", FrcTest.Test.X_DISTANCE_DRIVE);
        testMenu.addChoice("Y Distance Drive", FrcTest.Test.Y_DISTANCE_DRIVE);
        testMenu.addChoice("Turn Degrees", FrcTest.Test.TURN_DEGREES);
        testMenu.addChoice("Tune X PID", FrcTest.Test.TUNE_X_PID);
        testMenu.addChoice("Tune Y PID", FrcTest.Test.TUNE_Y_PID);
        testMenu.addChoice("Tune Turn PID", FrcTest.Test.TUNE_TURN_PID);
        testMenu.addChoice("Live Window", FrcTest.Test.LIVE_WINDOW, false, true);
    } // FrcTest

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);

        robot.ledIndicator.reset();

        //
        // Retrieve menu choice values.
        //
        test = testMenu.getCurrentChoiceObject();

        robot.gyroTurnPidCtrl.setNoOscillation(false);
        robot.gyroTurnPidCtrl.setTargetTolerance(RobotInfo.GYRO_TURN_TOLERANCE);

        if (test != Test.SUBSYSTEMS_TEST)
        {
            robot.driveBase.resetOdometry(true, false);
        }

        boolean liveWindowEnabled = false;
        switch (test)
        {
            case SENSORS_TEST:
                //
                // Make sure no joystick controls on sensors test.
                //
                robot.operatorStick.setButtonHandler(null);
                robot.buttonPanel.setButtonHandler(null);
                robot.switchPanel.setButtonHandler(null);
                //
                // Sensors Test is the same as Subsystems Test without joystick
                // control.
                // So let it flow to the next case.
                //
            case SUBSYSTEMS_TEST:
                break;

            case DRIVE_MOTORS_TEST:
                motorIndex = 0;
                break;

            case X_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(robot, 0.0, robot.driveTime, robot.drivePower, 0.0, 0.0);
                break;

            case Y_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(robot, 0.0, robot.driveTime, 0.0, robot.drivePower, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(
                    robot, robot.pidDrive, 0.0, robot.driveDistance, 0.0, 0.0, robot.drivePowerLimit, false, false);
                break;

            case Y_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(
                    robot, robot.pidDrive, 0.0, 0.0, robot.driveDistance, 0.0, robot.drivePowerLimit, false, false);
                break;

            case TURN_DEGREES:
                pidDriveCommand = new CmdPidDrive(
                    robot, robot.pidDrive, 0.0, 0.0, 0.0, robot.turnDegrees, robot.drivePowerLimit, false, false);
                break;

            case TUNE_X_PID:
                pidDriveCommand = new CmdPidDrive(
                    robot, robot.pidDrive, 0.0, robot.driveDistance, 0.0, 0.0, robot.drivePowerLimit, false, true);
                break;

            case TUNE_Y_PID:
                pidDriveCommand = new CmdPidDrive(
                    robot, robot.pidDrive, 0.0, 0.0, robot.driveDistance, 0.0, robot.drivePowerLimit, false, true);
                break;

            case TUNE_TURN_PID:
                pidDriveCommand = new CmdPidDrive(
                    robot, robot.pidDrive, 0.0, 0.0, 0.0, robot.turnDegrees, robot.drivePowerLimit, false, true);
                break;

            case LIVE_WINDOW:
                liveWindowEnabled = true;
                break;

            default:
                break;
        }

        LiveWindow.setEnabled(liveWindowEnabled);
        sm.start(State.START);
    } // startMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //
    @Override
    public void runPeriodic(double elapsedTime)
    {
        switch (test)
        {
            case SENSORS_TEST:
                doSensorsTest();
                break;

            case SUBSYSTEMS_TEST:
                //
                // Allow TeleOp to run so we can control the robot in subsystems
                // test mode.
                //
                super.runPeriodic(elapsedTime);
                doSensorsTest();
                break;

            case DRIVE_MOTORS_TEST:
                doDriveMotorsTest();
                break;

            default:
                break;
        }
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        switch (test)
        {
            case SENSORS_TEST:
                super.runContinuous(elapsedTime);
                break;

            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                double lfEnc = robot.leftFrontWheel.getPosition();
                double rfEnc = robot.rightFrontWheel.getPosition();
                double lbEnc = robot.leftBackWheel.getPosition();
                double rbEnc = robot.rightBackWheel.getPosition();
                robot.dashboard.displayPrintf(2, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
                robot.dashboard.displayPrintf(3, "Enc:lb=%.0f,rb=%.0f", lbEnc, rbEnc);
                robot.dashboard.displayPrintf(4, "average=%f", (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                robot.dashboard.displayPrintf(5, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                    robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading());
                timedDriveCommand.cmdPeriodic(elapsedTime);
                break;

            case X_DISTANCE_DRIVE:
            case Y_DISTANCE_DRIVE:
            case TURN_DEGREES:
            case TUNE_X_PID:
            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                robot.dashboard.displayPrintf(2, "xPos=%.1f,yPos=%.1f,heading=%.1f, lf=%.2f,rf=%.2f,lb=%.2f,rb=%.2f",
                    robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading(),
                    robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition(),
                    robot.leftBackWheel.getPosition(), robot.rightBackWheel.getPosition());
                robot.encoderXPidCtrl.displayPidInfo(3);
                robot.encoderYPidCtrl.displayPidInfo(5);
                robot.gyroTurnPidCtrl.displayPidInfo(7);
                pidDriveCommand.cmdPeriodic(elapsedTime);
                break;

            default:
                break;
        }

        if (robot.pidDrive.isActive())
        {
            robot.encoderXPidCtrl.printPidInfo(robot.globalTracer, false, robot.battery);
            robot.encoderYPidCtrl.printPidInfo(robot.globalTracer, false, robot.battery);
            robot.gyroTurnPidCtrl.printPidInfo(robot.globalTracer, false, robot.battery);
        }
    } // runContinuous

    @Override
    public void driverControllerButtonEvent(int button, boolean pressed)
    {
        boolean processedInput = false;

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                break;

            case FrcXboxController.BUTTON_B:
                break;

            case FrcXboxController.BUTTON_X:
                break;

            case FrcXboxController.BUTTON_Y:
                break;

            case FrcXboxController.LEFT_BUMPER:
                break;

            case FrcXboxController.RIGHT_BUMPER:
                break;

            case FrcXboxController.BACK:
                break;

            case FrcXboxController.START:
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }

        if (!processedInput)
        {
            super.driverControllerButtonEvent(button, pressed);
        }
    } // operatorStickButtonEvent

    @Override
    public void operatorStickButtonEvent(int button, boolean pressed)
    {
        boolean processedInput = false;

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }

        if (!processedInput)
        {
            super.operatorStickButtonEvent(button, pressed);
        }
    } // operatorStickButtonEvent

    @Override
    public void buttonPanelButtonEvent(int button, boolean pressed)
    {
        boolean processedInput = false;

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;
        }

        if (!processedInput)
        {
            super.buttonPanelButtonEvent(button, pressed);
        }
    } // operatorStickButtonEvent

    /**
     * This method reads all sensors and prints out their values. This is a very
     * useful diagnostic tool to check if all sensors are working properly. For
     * encoders, since test subsystem mode is also teleop mode, you can operate
     * the joysticks to turn the motors and check the corresponding encoder
     * counts.
     */
    private void doSensorsTest()
    {
        double lfPos = robot.leftFrontWheel.getPosition();
        double rfPos = robot.rightFrontWheel.getPosition();
        double lbPos = robot.leftBackWheel.getPosition();
        double rbPos = robot.rightBackWheel.getPosition();
        double driveBaseAverage = (lfPos + rfPos + lbPos + rbPos) / 4.0;
        robot.dashboard.displayPrintf(1, "Sensors Test (Batt=%.1f/%.1f):", robot.battery.getVoltage(),
            robot.battery.getLowestVoltage());
        robot.dashboard
            .displayPrintf(2, "DriveBase: lf=%.3f,rf=%.3f,lb=%.3f,rb=%.3f,avg=%.3f", lfPos, rfPos, lbPos, rbPos,
                driveBaseAverage);
        robot.dashboard
            .displayPrintf(3, "DriveBase: X=%.1f,Y=%.1f,Heading=%.1f,GyroRate=%.3f", robot.driveBase.getXPosition(),
                robot.driveBase.getYPosition(), robot.driveBase.getHeading(), robot.gyro.getZRotationRate().value);
        robot.dashboard.displayPrintf(4, "Sensors: pressure=%.1f", robot.getPressure());

        if (robot.vision != null)
        {
            FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
            if (pose != null)
            {
                robot.dashboard
                    .displayPrintf(13, "RaspiVision: x=%.1f,y=%.1f,objectYaw=%.1f", pose.x, pose.y, pose.objectYaw);
            }
            else
            {
                robot.dashboard.displayPrintf(13, "RaspiVision: No target found!");
            }
        }
    } // doSensorsTest

    /**
     * This method runs each of the four wheels in sequence for a fixed number
     * of seconds. It is for diagnosing problems with the drive train. At the
     * end of the run, you should check the amount of encoder counts each wheel
     * has accumulated. They should be about the same. If not, you need to check
     * the problem wheel for friction or chain tension etc. You can also use
     * this test to check if a motor needs to be "inverted" (i.e. turning in the
     * wrong direction).
     */
    private void doDriveMotorsTest()
    {
        robot.dashboard.displayPrintf(1, "Motors Test: index=%d", motorIndex);
        robot.dashboard.displayPrintf(2, "Enc: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f", robot.leftFrontWheel.getPosition(),
            robot.rightFrontWheel.getPosition(), robot.leftBackWheel.getPosition(), robot.rightBackWheel.getPosition());

        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            switch (state)
            {
                case START:
                    //
                    // Spin a wheel for 5 seconds.
                    //
                    switch (motorIndex)
                    {
                        case 0:
                            //
                            // Run the left front wheel.
                            //
                            robot.leftFrontWheel.set(robot.drivePower);
                            robot.rightFrontWheel.set(0.0);
                            robot.leftBackWheel.set(0.0);
                            robot.rightBackWheel.set(0.0);
                            break;

                        case 1:
                            //
                            // Run the right front wheel.
                            //
                            robot.leftFrontWheel.set(0.0);
                            robot.rightFrontWheel.set(robot.drivePower);
                            robot.leftBackWheel.set(0.0);
                            robot.rightBackWheel.set(0.0);
                            break;

                        case 2:
                            //
                            // Run the left back wheel.
                            //
                            robot.leftFrontWheel.set(0.0);
                            robot.rightFrontWheel.set(0.0);
                            robot.leftBackWheel.set(robot.drivePower);
                            robot.rightBackWheel.set(0.0);
                            break;

                        case 3:
                            //
                            // Run the right back wheel.
                            //
                            robot.leftFrontWheel.set(0.0);
                            robot.rightFrontWheel.set(0.0);
                            robot.leftBackWheel.set(0.0);
                            robot.rightBackWheel.set(robot.drivePower);
                            break;
                    }
                    motorIndex = motorIndex + 1;
                    timer.set(robot.driveTime, event);
                    sm.waitForSingleEvent(event, motorIndex < 4 ? State.START : State.DONE);
                    break;

                case DONE:
                    //
                    // We are done, stop all wheels.
                    //
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    } // doDriveMotorsTest

} // class FrcTest
