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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.SPI;
import frclib.FrcAHRSGyro;
import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;
import frclib.FrcJoystick;
import frclib.FrcPdp;
import frclib.FrcRemoteVisionProcessor;
import frclib.FrcRobotBase;
import frclib.FrcRobotBattery;
import frclib.FrcTalonServo;
import frclib.FrcXboxController;
import hallib.HalDashboard;
import trclib.TrcEnhancedServo;
import trclib.TrcHolonomicPurePursuitDrive;
import trclib.TrcPidController;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcPidDrive;
import trclib.TrcRobot.RunMode;
import trclib.TrcRobotBattery;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcUtil;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Date;
import java.util.Locale;
import java.util.Scanner;
import java.util.stream.IntStream;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase
{
    public static class Preferences
    {
        public final boolean isPracticeBot = false;
        public final boolean useController = true;
        public final boolean useTraceLog = true;
        public final boolean useNavX = true;
        public final boolean useGyroAssist = false;
        public final boolean useVision = true;
        public final boolean useStreamCamera = true;
        public final boolean doAutoUpdates = false;

        public final boolean debugPowerConsumption = false;
        public final boolean debugDriveBase = false;
        public final boolean debugPidDrive = false;
        public final boolean debugSubsystems = false;
        public final boolean debugVision = false;
        public final boolean debugLoopTime = true;
    }   //class Preferences

    public enum DriveOrientation
    {
        ROBOT, FIELD, INVERTED
    }   // enum DriveOrientation

    //
    // Global constants.
    //
    public static final String programName = "InfiniteRecharge";
    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;
    //
    // Global objects.
    //
    public final Preferences preferences = new Preferences();
    public final DriverStation ds = DriverStation.getInstance();
    public final HalDashboard dashboard = HalDashboard.getInstance();
    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick, rightDriveStick;
    public FrcXboxController driverController;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public FrcAHRSGyro gyro;
    public AnalogInput pressureSensor;
    //
    // DriveBase subsystem.
    //
    public FrcCANSparkMax lfDriveMotor, rfDriveMotor, lrDriveMotor, rrDriveMotor;
    public FrcCANTalon lfSteerMotor, rfSteerMotor, lrSteerMotor, rrSteerMotor;
    public TrcSwerveModule leftFrontWheel;
    public TrcSwerveModule leftBackWheel;
    public TrcSwerveModule rightFrontWheel;
    public TrcSwerveModule rightBackWheel;
    public TrcSwerveDriveBase driveBase;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;
    public TrcHolonomicPurePursuitDrive purePursuit;
    //
    // Vision subsystem.
    //
    public VisionTargeting vision = null;
    //
    // Miscellaneous subsystem.
    //
    public LEDIndicator ledIndicator;

    private double nextUpdateTime = TrcUtil.getCurrentTime();
    //
    // FMS provided the following info:
    //  - event name
    //  - match type
    //  - match number
    //  - alliance
    //  - location
    //  - replay number???
    //
    public String eventName = "Unknown";
    public MatchType matchType = MatchType.None;
    public int matchNumber = 0;
    public Alliance alliance = Alliance.Red;
    public int location = 1;
    public String gameSpecificMessage = null;
    public boolean traceLogOpened = false;
    private DriveOrientation driveOrientation = DriveOrientation.FIELD;
    //
    // Other robot subystems.
    //
    public WallAlignment alignment;
    public Conveyor conveyor;
    public Shooter shooter;
    public Intake intake;
    public Climber climber;
    public TaskAutoShooter autoShooter;
    public TaskAutoAlign autoAlign;
    public TaskSnapToAngle snapToAngle;

    public enum DriveSpeed
    {
        SLOW, MEDIUM, FAST
    }

    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public DriveSpeed driveSpeed = DriveSpeed.MEDIUM;
    private int numBalls;
    public double driveTime;
    public double drivePower;
    public double driveDistance;
    public double turnDegrees;
    public double drivePowerLimit;
    public TrcPidController.PidCoefficients tunePidCoeff;

    private FrcAuto autoMode;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    private FrcCANSparkMax createSparkMax(String name, int id)
    {
        FrcCANSparkMax spark = new FrcCANSparkMax(name, id, true);
        spark.motor.restoreFactoryDefaults();
        spark.setInverted(false);
        spark.setPositionSensorInverted(false);
        spark.setBrakeModeEnabled(true);
        spark.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        spark.motor.burnFlash();
        return spark;
    }

    private FrcCANTalon createSteerTalon(String name, int id, boolean inverted)
    {
        FrcCANTalon talon = new FrcCANTalon(name, id);
        talon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talon.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        talon.motor.enableVoltageCompensation(true);
        talon.motor.overrideLimitSwitchesEnable(false);
        talon.configFwdLimitSwitchNormallyOpen(true);
        talon.configRevLimitSwitchNormallyOpen(true);
        talon.setBrakeModeEnabled(true);
        talon.setPositionSensorInverted(inverted);
        talon.setInverted(!inverted);
        return talon;
    }

    private TrcSwerveModule createModule(String name, FrcCANSparkMax drive, FrcCANTalon steer, int steerZero)
    {
        steer.motor.getSensorCollection().setPulseWidthPosition(0, 10); // reset index
        TrcUtil.sleep(50); // guarantee reset
        ErrorCode error = steer.motor.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true, -steerZero, 10);
        if (error != ErrorCode.OK)
        {
            System.out.printf("Encoder error! - Module=%s, error=%s\n", name, error.name());
        }
        TrcUtil.sleep(50); // guarantee reset
        int modPos = (int) TrcUtil.modulo(steer.motor.getSelectedSensorPosition(), 4096);
        int pos = modPos > 2048 ? modPos - 4096 : modPos;
        steer.motor.setSelectedSensorPosition(pos, 0, 10);
        TrcUtil.sleep(50);

        System.out.printf("Module=%s, Zero=%d, PwmPos=%d, quadPos=%d, selectedPos=%d\n", name, steerZero,
            steer.motor.getSensorCollection().getPulseWidthPosition(),
            steer.motor.getSensorCollection().getQuadraturePosition(), steer.motor.getSelectedSensorPosition());

        FrcTalonServo servo = new FrcTalonServo(name + ".servo", steer, RobotInfo.magicSteerCoeff,
            RobotInfo.STEER_DEGREES_PER_TICK, RobotInfo.STEER_MAX_REQ_VEL, RobotInfo.STEER_MAX_ACCEL);
        TrcSwerveModule module = new TrcSwerveModule(name, drive, new TrcEnhancedServo(name + ".enhancedServo", servo));
        module.disableSteeringLimits();
        return module;
    }

    private int[] getSteerZeroPositions()
    {
        try (Scanner in = new Scanner(new FileReader("/home/lvuser/steerzeros.txt")))
        {
            return IntStream.range(0, 4).map(e -> in.nextInt()).toArray();
        }
        catch (Exception e)
        {
            globalTracer.traceErr("Robot.getSteerZeroPositions", "ERROR! Steer zero position file not found!");
            return RobotInfo.STEER_ZEROS;
        }
    }

    public void saveSteerZeroPositions()
    {
        globalTracer.traceInfo("Robot.saveSteerZeroPositions", "Saved steer zeros!");
        try (PrintStream out = new PrintStream(new FileOutputStream("/home/lvuser/steerzeros.txt")))
        {
            out.printf("%.0f\n",
                TrcUtil.modulo(lfSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
            out.printf("%.0f\n",
                TrcUtil.modulo(rfSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
            out.printf("%.0f\n",
                TrcUtil.modulo(lrSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
            out.printf("%.0f\n",
                TrcUtil.modulo(rrSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        //
        // Inputs.
        //
        if (preferences.useController)
        {
            driverController = new FrcXboxController("DriverController", RobotInfo.XBOX_DRIVERCONTROLLER);
            operatorStick = new FrcJoystick("operatorStick", RobotInfo.JSPORT_OPERATORSTICK);
            buttonPanel = new FrcJoystick("buttonPanel", RobotInfo.JSPORT_BUTTON_PANEL);
            switchPanel = new FrcJoystick("switchPanel", RobotInfo.JSPORT_SWITCH_PANEL);
            driverController.setLeftYInverted(true);
        }
        else
        {
            leftDriveStick = new FrcJoystick("DriverLeftStick", RobotInfo.XBOX_DRIVERCONTROLLER);
            rightDriveStick = new FrcJoystick("DriverRightStick", RobotInfo.XBOX_DRIVERCONTROLLER + 1);
            operatorStick = new FrcJoystick("operatorStick", RobotInfo.JSPORT_OPERATORSTICK + 1);
            buttonPanel = new FrcJoystick("buttonPanel", RobotInfo.JSPORT_BUTTON_PANEL + 1);
            switchPanel = new FrcJoystick("switchPanel", RobotInfo.JSPORT_SWITCH_PANEL + 1);
            rightDriveStick.setYInverted(true);
        }
        operatorStick.setYInverted(false);

        //
        // Sensors.
        //
        pdp = new FrcPdp(RobotInfo.CANID_PDP);
        battery = new FrcRobotBattery(pdp);
        gyro = preferences.useNavX ? new FrcAHRSGyro("NavX", SPI.Port.kMXP) : null;
        pressureSensor = new AnalogInput(RobotInfo.AIN_PRESSURE_SENSOR);
        //
        // DriveBase subsystem.
        //
        lfDriveMotor = createSparkMax("LFDrive", RobotInfo.CANID_LEFTFRONT_DRIVE);
        rfDriveMotor = createSparkMax("RFDrive", RobotInfo.CANID_RIGHTFRONT_DRIVE);
        lrDriveMotor = createSparkMax("LRDrive", RobotInfo.CANID_LEFTREAR_DRIVE);
        rrDriveMotor = createSparkMax("RRDrive", RobotInfo.CANID_RIGHTREAR_DRIVE);

        // rf lr are inverted always, lf and rr are inverted on comp, not inverted on practice
        lfSteerMotor = createSteerTalon("LFSteer", RobotInfo.CANID_LEFTFRONT_STEER, !preferences.isPracticeBot);
        rfSteerMotor = createSteerTalon("RFSteer", RobotInfo.CANID_RIGHTFRONT_STEER, true);
        lrSteerMotor = createSteerTalon("LRSteer", RobotInfo.CANID_LEFTREAR_STEER, true);
        rrSteerMotor = createSteerTalon("RRSteer", RobotInfo.CANID_RIGHTREAR_STEER, !preferences.isPracticeBot);

        int[] zeros = getSteerZeroPositions();
        leftFrontWheel = createModule("LeftFrontWheel", lfDriveMotor, lfSteerMotor, zeros[0]);
        rightFrontWheel = createModule("RightFrontWheel", rfDriveMotor, rfSteerMotor, zeros[1]);
        leftBackWheel = createModule("LeftRearWheel", lrDriveMotor, lrSteerMotor, zeros[2]);
        rightBackWheel = createModule("RightRearWheel", rrDriveMotor, rrSteerMotor, zeros[3]);

        pdp.registerEnergyUsed(new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_FRONT_WHEEL, "LeftFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_BACK_WHEEL, "LeftBackWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_FRONT_WHEEL, "RightFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_BACK_WHEEL, "RightBackWheel"));

        driveBase = new TrcSwerveDriveBase(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, gyro,
            RobotInfo.ROBOT_DRIVE_WIDTH, RobotInfo.ROBOT_DRIVE_LENGTH);
        driveBase.setSynchronizeOdometriesEnabled(false);
        driveBase.setOdometryScales(RobotInfo.ENCODER_INCHES_PER_COUNT);
        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController("encoderXPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_KP, RobotInfo.ENCODER_KI, RobotInfo.ENCODER_KD, RobotInfo.ENCODER_KF),
            RobotInfo.ENCODER_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController("encoderYPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_KP, RobotInfo.ENCODER_KI, RobotInfo.ENCODER_KD, RobotInfo.ENCODER_KF),
            RobotInfo.ENCODER_TOLERANCE, driveBase::getYPosition);
        gyroTurnPidCtrl = new TrcPidController("gyroTurnPidCtrl",
            new PidCoefficients(RobotInfo.GYRO_TURN_KP_BIG, 0, RobotInfo.GYRO_TURN_KD_BIG),
            RobotInfo.GYRO_TURN_TOLERANCE, driveBase::getHeading);
        encoderXPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_XPID_POWER);
        encoderYPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_YPID_POWER);
        gyroTurnPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_TURNPID_POWER);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
        encoderXPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_XPID_RAMP_RATE);
        encoderYPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_YPID_RAMP_RATE);
        gyroTurnPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_TURNPID_RAMP_RATE);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        pidDrive.setMsgTracer(globalTracer, true, false);

        purePursuit = new TrcHolonomicPurePursuitDrive("purePursuit", driveBase,
            RobotInfo.PURE_PURSUIT_FOLLOWING_DISTANCE, RobotInfo.PURE_PURSUIT_POS_TOLERANCE,
            RobotInfo.PURE_PURSUIT_HEADING_TOLERANCE, encoderYPidCtrl.getPidCoefficients(),
            new TrcPidController.PidCoefficients(RobotInfo.GYRO_TURN_KP, 0, RobotInfo.GYRO_TURN_KD),
            new TrcPidController.PidCoefficients(0, 0, 0, RobotInfo.PURE_PURSUIT_KF));
        purePursuit.setMoveOutputLimit(RobotInfo.PURE_PURSUIT_MOVE_OUTPUT_LIMIT);
        purePursuit.setMsgTracer(globalTracer, true, false);
        //
        // Vision subsystem.
        //
        if (preferences.useVision)
        {
            vision = new VisionTargeting();
        }
        //
        // Miscellaneous subsystems.
        //
        ledIndicator = new LEDIndicator();
        shooter = new Shooter(this);
        conveyor = new Conveyor(this);
        alignment = new WallAlignment();
        intake = new Intake(this); // must initialize after conveyor
        climber = new Climber(this);

        setDriveOrientation(DriveOrientation.FIELD);

        if (preferences.useStreamCamera)
        {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("DriverDisplay", 0);
            camera.setResolution(160, 120);
        }

        //
        // AutoAssist commands.
        //
        autoAlign = new TaskAutoAlign(this);
        autoShooter = new TaskAutoShooter(this);
        snapToAngle = new TaskSnapToAngle(this);

        //
        // Create Robot Modes.
        //
        autoMode = new FrcAuto(this);
        setupRobotModes(new FrcTeleOp(this), autoMode, new FrcTest(this), new FrcDisabled(this));

        pdp.registerEnergyUsedForAllUnregisteredChannels();

        HalDashboard.refreshKey("Test/DriveTime", 5.0);
        HalDashboard.refreshKey("Test/DrivePower", 0.2);
        HalDashboard.refreshKey("Test/DriveDistance", 6.0);
        HalDashboard.refreshKey("Test/TurnDegrees", 90.0);
        HalDashboard.refreshKey("Test/DrivePowerLimit", 0.5);
        HalDashboard.refreshKey("Test/TuneKp", RobotInfo.GYRO_TURN_KP);
        HalDashboard.refreshKey("Test/TuneKi", RobotInfo.GYRO_TURN_KI);
        HalDashboard.refreshKey("Test/TuneKd", RobotInfo.GYRO_TURN_KD);
        HalDashboard.refreshKey("Test/TuneKf", 0.0);
    }   //robotInit

    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        final String funcName = "robotStartMode";

        driveOrientation = DriveOrientation.FIELD;

        if (runMode != RunMode.DISABLED_MODE)
        {
            openTraceLog(
                runMode == RunMode.AUTO_MODE ? "FrcAuto" : runMode == RunMode.TELEOP_MODE ? "FrcTeleOp" : "FrcTest");
            setTraceLogEnabled(true);

            Date now = new Date();
            globalTracer.traceInfo(funcName, "[%.3f] %s: ***** %s *****", TrcUtil.getModeElapsedTime(), now.toString(),
                runMode);

            //            pdp.setTaskEnabled(true);
            //            battery.setEnabled(true);
            driveBase.resetOdometry(true, false);
            lfDriveMotor.setOdometryEnabled(true);
            rfDriveMotor.setOdometryEnabled(true);
            lrDriveMotor.setOdometryEnabled(true);
            rrDriveMotor.setOdometryEnabled(true);
            driveBase.setOdometryEnabled(true);
            shooter.setEnabled(true);
            shooter.setManualOverrideEnabled(false);
            conveyor.setManualOverrideEnabled(false);
            climber.latch();

            dashboard.clearDisplay();

            ledIndicator.reset();

            if (runMode == RunMode.AUTO_MODE)
            {
                lfDriveMotor.motor.setOpenLoopRampRate(0);
                rfDriveMotor.motor.setOpenLoopRampRate(0);
                lrDriveMotor.motor.setOpenLoopRampRate(0);
                rrDriveMotor.motor.setOpenLoopRampRate(0);
            }
            else
            {
                lfDriveMotor.motor.setOpenLoopRampRate(RobotInfo.DRIVE_RAMP_RATE);
                rfDriveMotor.motor.setOpenLoopRampRate(RobotInfo.DRIVE_RAMP_RATE);
                lrDriveMotor.motor.setOpenLoopRampRate(RobotInfo.DRIVE_RAMP_RATE);
                rrDriveMotor.motor.setOpenLoopRampRate(RobotInfo.DRIVE_RAMP_RATE);
            }

            if (runMode == RunMode.AUTO_MODE || runMode == RunMode.TEST_MODE)
            {
                driveTime = HalDashboard.getNumber("Test/DriveTime", 5.0);
                drivePower = HalDashboard.getNumber("Test/DrivePower", 0.2);
                driveDistance = HalDashboard.getNumber("Test/DriveDistance", 6.0);
                turnDegrees = HalDashboard.getNumber("Test/TurnDegrees", 90.0);
                drivePowerLimit = HalDashboard.getNumber("Test/DrivePowerLimit", 0.5);
                if (runMode == RunMode.TEST_MODE)
                {
                    tunePidCoeff = new TrcPidController.PidCoefficients(
                        HalDashboard.getNumber("Test/TuneKp", RobotInfo.GYRO_TURN_KP),
                        HalDashboard.getNumber("Test/TuneKi", RobotInfo.GYRO_TURN_KI),
                        HalDashboard.getNumber("Test/TuneKd", RobotInfo.GYRO_TURN_KD),
                        HalDashboard.getNumber("Test/TuneKf", 0.0));
                }
            }

            //            gyro.setElapsedTimerEnabled(true);
            //            TrcDigitalInput.setElapsedTimerEnabled(true);
            //            TrcMotor.setElapsedTimerEnabled(true);
            //            TrcServo.setElapsedTimerEnabled(true);
        }
    }   //robotStartMode

    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        final String funcName = "robotStopMode";

        if (runMode != RunMode.DISABLED_MODE)
        {
            //            gyro.printElapsedTime(globalTracer);
            //            gyro.setElapsedTimerEnabled(false);
            //            TrcDigitalInput.printElapsedTime(globalTracer);
            //            TrcDigitalInput.setElapsedTimerEnabled(false);
            //            TrcMotor.printElapsedTime(globalTracer);
            //            TrcMotor.setElapsedTimerEnabled(false);
            //            TrcServo.printElapsedTime(globalTracer);
            //            TrcServo.setElapsedTimerEnabled(false);

            globalTracer.traceInfo(funcName, "mode=%s,heading=%.1f", runMode.name(), driveBase.getHeading());
            driveBase.setOdometryEnabled(false);
            cancelAllAuto();
            battery.setEnabled(false);
            pdp.setTaskEnabled(false);
            conveyor.stop();
            shooter.stopFlywheel();
            shooter.pitchMotor.set(0);
            shooter.setEnabled(false);

            //            for (int i = 0; i < FrcPdp.kPDPChannels; i++)
            //            {
            //                String channelName = pdp.getChannelName(i);
            //                if (channelName != null)
            //                {
            //                    globalTracer
            //                        .traceInfo(funcName, "[PDP-%02d] %s: EnergyUsed=%.3f Wh", i, channelName, pdp.getEnergyUsed(i));
            //                }
            //            }

            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(funcName, "TotalEnergy=%.3fWh (%.2f%%)", totalEnergy,
                totalEnergy * 100.0 / RobotInfo.BATTERY_CAPACITY_WATT_HOUR);
            setTraceLogEnabled(false);
            closeTraceLog();

            ledIndicator.reset();
        }
    }   //robotStopMode

    public void getFMSInfo()
    {
        eventName = ds.getEventName();
        if (eventName.length() == 0)
        {
            eventName = "Unknown";
        }
        matchType = ds.getMatchType();
        matchNumber = ds.getMatchNumber();
    }

    public void getGameInfo()
    {
        alliance = ds.getAlliance();
        location = ds.getLocation();
        gameSpecificMessage = ds.getGameSpecificMessage();
    }

    public void openTraceLog(String defaultName)
    {
        if (preferences.useTraceLog && !traceLogOpened)
        {
            String fileName;

            if (ds.isFMSAttached())
            {
                getFMSInfo();
                fileName = String.format("%s_%s%03d", eventName, matchType, matchNumber);
            }
            else
            {
                fileName = defaultName;
            }

            traceLogOpened = globalTracer.openTraceLog("/home/lvuser/tracelog", fileName);
        }
    }

    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            globalTracer.closeTraceLog();
            traceLogOpened = false;
        }
    }

    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            globalTracer.setTraceLogEnabled(enabled);
        }
    }

    /**
     * Stops the lower level subsystems. This does NOT stop any auto/auto assist commands.
     */
    public void stopSubsystems()
    {
        pidDrive.cancel();
        driveBase.stop();
    }

    public void updateDashboard(RunMode runMode)
    {
        final String funcName = "updateDashboard";
        double currTime = TrcUtil.getModeElapsedTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;

            if (preferences.debugPowerConsumption)
            {
                HalDashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                HalDashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                HalDashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                if (runMode == RunMode.TELEOP_MODE)
                {
                    globalTracer.traceInfo(funcName, "[%.3f] Battery: currVoltage=%.2f, lowestVoltage=%.2f", currTime,
                        battery.getVoltage(), battery.getLowestVoltage());
                    globalTracer.traceInfo(funcName, "[%.3f] Total=%.2fA", currTime, pdp.getTotalCurrent());
                }
            }

            if (preferences.debugDriveBase)
            {
                double xPos = driveBase.getXPosition();
                double yPos = driveBase.getYPosition();
                double heading = driveBase.getHeading();

                HalDashboard.putNumber("DriveBase/xPos", xPos);
                HalDashboard.putNumber("DriveBase/yPos", yPos);
                HalDashboard.putData("DriveBase/heading", gyro.getGyroSendable());

                //
                // DriveBase debug info.
                //
                double lfEnc = leftFrontWheel.getPosition();
                double rfEnc = rightFrontWheel.getPosition();
                double lbEnc = leftBackWheel.getPosition();
                double rbEnc = rightBackWheel.getPosition();

                dashboard
                    .displayPrintf(8, "DriveBase: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f", lfEnc, rfEnc, lbEnc,
                        rbEnc, (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                dashboard.displayPrintf(9, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f", xPos, yPos, heading);

                if (preferences.debugPidDrive)
                {
                    encoderXPidCtrl.displayPidInfo(10);
                    encoderYPidCtrl.displayPidInfo(12);
                    gyroTurnPidCtrl.displayPidInfo(14);
                }
            }

            if (preferences.debugSubsystems)
            {
                if (preferences.debugVision && vision != null)
                {
                    FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
                    if (pose != null)
                    {
                        dashboard.displayPrintf(13, "VisionTarget: x=%.1f,y=%.1f,objectYaw=%.1f", pose.x, pose.y,
                            pose.objectYaw);
                    }
                    else
                    {
                        dashboard.displayPrintf(13, "VisionTarget: No target found!");
                    }
                }
            }
        }
    }   //updateDashboard

    public int getNumBalls()
    {
        return numBalls;
    }

    public void incNumBalls()
    {
        setNumBalls(getNumBalls() + 1);
        if (getNumBalls() > 5)
        {
            globalTracer.traceErr("incNumBalls", "Invalid number of balls: %d! Resetting to five.", getNumBalls());
            setNumBalls(5);
        }
    }

    public void decNumBalls()
    {
        setNumBalls(getNumBalls() - 1);
        if (getNumBalls() < 0)
        {
            globalTracer.traceErr("decNumBalls", "Invalid number of balls: %d! Resetting to zero.", getNumBalls());
            setNumBalls(0);
        }
    }

    public void setNumBalls(int numBalls)
    {
        this.numBalls = numBalls;
    }

    public double[] getDriveInputs()
    {
        double x, y, rot;
        double mag;
        if (preferences.useController)
        {
            x = driverController.getLeftXWithDeadband(false);
            y = driverController.getLeftYWithDeadband(false);
            rot = driverController.getRightXWithDeadband(true);
            mag = TrcUtil.magnitude(x, y);
            mag = Math.copySign(Math.pow(mag, 3), mag);
        }
        else
        {
            x = rightDriveStick.getXWithDeadband(false);
            y = rightDriveStick.getYWithDeadband(false);
            rot = leftDriveStick.getXWithDeadband(true);
            mag = TrcUtil.magnitude(x, y);
            mag = Math.copySign(Math.pow(mag, 2), mag);
        }
        switch (driveSpeed)
        {
            case SLOW:
                mag *= RobotInfo.DRIVE_SLOW_SCALE;
                rot *= RobotInfo.DRIVE_SLOW_TURNSCALE;
                break;

            case MEDIUM:
                mag *= RobotInfo.DRIVE_MEDIUM_SCALE;
                rot *= RobotInfo.DRIVE_MEDIUM_TURNSCALE;
                break;

            case FAST:
                mag *= RobotInfo.DRIVE_FAST_SCALE;
                rot *= RobotInfo.DRIVE_MEDIUM_TURNSCALE;
                break;
        }
        x *= mag;
        y *= mag;
        return new double[] { x, y, rot };
    }

    public double getDriveGyroAngle()
    {
        switch (driveOrientation)
        {
            case ROBOT:
                return 0;

            case INVERTED:
                return 180;

            default:
            case FIELD:
                return driveBase.getHeading();
        }
    }

    public DriveOrientation getDriveOrientation()
    {
        return driveOrientation;
    }

    public void setDriveOrientation(DriveOrientation driveOrientation)
    {
        this.driveOrientation = driveOrientation;
        ledIndicator.setDriveOrientation(driveOrientation);
    }

    /**
     * Checks if any auto processes are running, be it auto mode or auto assist, etc.
     *
     * @return True if any auto processes are active, false otherwise.
     */
    public boolean isAutoActive()
    {
        return autoMode.isAutoActive();
    }

    public void cancelAllAuto()
    {
        if (autoMode.isAutoActive())
        {
            autoMode.cancel();
        }
        if (autoShooter.isActive())
        {
            autoShooter.cancel();
        }
        if (autoAlign.isActive())
        {
            autoAlign.cancel();
        }
        if (climber.isActive())
        {
            climber.cancel();
        }
    }

    public void setAntiDefenseEnabled(boolean enabled)
    {
        setAntiDefenseEnabled("AntiDefense", enabled);
    }

    public void setAntiDefenseEnabled(String owner, boolean enabled)
    {
        if (enabled && driveBase.acquireExclusiveAccess(owner))
        {
            leftFrontWheel.setSteerAngle(-45);
            rightFrontWheel.setSteerAngle(45);
            leftBackWheel.setSteerAngle(-135);
            rightBackWheel.setSteerAngle(135);
        }
        else if (!enabled && driveBase.releaseExclusiveAccess(owner))
        {
            leftFrontWheel.setSteerAngle(0);
            rightFrontWheel.setSteerAngle(0);
            leftBackWheel.setSteerAngle(0);
            rightBackWheel.setSteerAngle(0);
        }
    }

    public void traceStateInfo(double elapsedTime, String stateName, double xTarget, double yTarget, double turnTarget)
    {
        final String funcName = "traceStateInfo";
        StringBuilder msg = new StringBuilder();

        // msg.append(String
        //     .format("[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f", elapsedTime, stateName,
        //         driveBase.getXPosition(), xTarget, driveBase.getYPosition(), yTarget, driveBase.getHeading(),
        //         turnTarget));

        if (driveBase != null)
        {
            msg.append(String.format(Locale.US, "tag=\">>>>>\" state=\"%s\"", stateName));
            msg.append(
                String.format(Locale.US, " xPos=\"%6.2f\" xTarget=\"%6.2f\"", driveBase.getXPosition(), xTarget));
            msg.append(
                String.format(Locale.US, " yPos=\"%6.2f\" yTarget=\"%6.2f\"", driveBase.getYPosition(), yTarget));
            msg.append(String
                .format(Locale.US, " heading=\"%6.2f\" headingTarget=\"%6.2f\"", driveBase.getHeading(), turnTarget));
        }

        if (battery != null)
        {
            msg.append(String.format(",voltage=\"%5.2fV(%5.2fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
        }

        globalTracer.logEvent(funcName, "StateInfo", "%s", msg);
    }   //traceStateInfo

    //
    // Getters for sensor data.
    //

    public double getPressure()
    {
        return (pressureSensor.getVoltage() - 0.5) * 50.0;
    }   //getPressure

}   //class Robot
