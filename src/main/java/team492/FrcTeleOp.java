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

import trclib.TrcMecanumDriveBase;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;
import frclib.FrcXboxController;

public class FrcTeleOp implements TrcRobot.RobotMode 
{
    //
    // TeleOp mode global objects.
    //
    private Robot robot;

    public FrcTeleOp(Robot robot) 
    {
        //
        // Create and initialize global object.
        //
        this.robot = robot;
    } // FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode) 
    {
        //
        // Configure joysticks.
        //

        robot.operatorXboxController.setButtonHandler(this::operatorControllerButtonEvent);
        robot.driverXboxController.setButtonHandler(this::driverControllerButtonEvent);

        //
        // Initialize subsystems for TeleOp mode if necessary.
        //

    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disable subsystems before exiting if necessary.
        //

    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime) 
    {
        FrcXboxController driverGamepad = robot.driverXboxController;
        FrcXboxController operatorGamepad = robot.operatorXboxController;

        //
        // DriveBase operation.
        //
        double x = driverGamepad.getLeftXWithDeadband(true);
        double y = driverGamepad.getRightYWithDeadband(true);
        double rot = driverGamepad.getRightTriggerWithDeadband(true) - driverGamepad.getLeftTriggerWithDeadband(true);
        robot.driveBase.holonomicDrive(x, y, rot, false);

        //
        // Analog control of subsystem is done here if necessary.
        //

        // elevator control
        double elevatorPower = operatorGamepad.getRightYWithDeadband(false);
        robot.elevator.setPower(elevatorPower);


        //
        // Update dashboard info.
        //
        TrcMecanumDriveBase driveBase = robot.driveBase;
        robot.dashboard.displayPrintf(3, "Mecan: x=%.1f,y=%.1f,rot=%.1f", x, y, rot);
        robot.dashboard.displayPrintf(4, "DrvBase: x=%.3f, y=%.3f, r=%.3f", driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading());
        /*
        robot.dashboard.displayPrintf(4, "DrvBase: pos=%s, vel=%s", robot.driveBase.getFieldPosition(),
                robot.driveBase.getFieldVelocity());
                */
        robot.dashboard.displayPrintf(5, "Elevator: pos=%.3f, limitLower=%b", robot.elevator.getPosition(),
                robot.elevatorLowerLimitSwitch.isActive());
        robot.dashboard.displayPrintf(6, "Pneumatics: ext=%b", robot.pneumatic.isExtended());
        robot.dashboard.displayPrintf(7, "EnhncdServo: pos=%.3f, pwr=%.3f", robot.enhancedServo.getPosition(),
                robot.enhancedServo.getPower());
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime) 
    {
        //
        // Do subsystem assist here if necessary.
        //

    } // runContinuous

    //
    // Implements FrcButtonHandler.
    //
    public void operatorControllerButtonEvent(int button, boolean pressed) 
    {
        robot.dashboard.displayPrintf(1, " OperatorController: button=0x%04x %s", button,
                pressed ? "pressed" : "released");

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
                if (pressed)
                {
                    robot.elevator.zeroCalibrate();
                }
                break;

            case FrcXboxController.START:
                robot.elevator.setManualOverride(pressed);
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }
    } // operatorControllerButtonEvent

    public void driverControllerButtonEvent(int button, boolean pressed) 
    {
        robot.dashboard.displayPrintf(2, " DriverController: button=0x%04x %s", button,
                pressed ? "pressed" : "released");

        switch (button) 
        {
            case FrcXboxController.BUTTON_A:
                if (pressed) 
                {
                    robot.pidDrive.setRelativeYTarget(120.0, null);
                }
                break;

            case FrcXboxController.BUTTON_B:
                if (pressed) 
                {
                    robot.pidDrive.setRelativeXTarget(-60.0, null);
                }
                break;

            case FrcXboxController.BUTTON_X:
                if (pressed) 
                {
                    robot.pidDrive.setRelativeYTarget(-84.0, null);
                }
                break;

            case FrcXboxController.BUTTON_Y:
                if (pressed) 
                {
                    robot.pidDrive.setRelativeTurnTarget(-90.0, null);
                }
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
    } // driverControllerButtonEvent

} // class FrcTeleOp
