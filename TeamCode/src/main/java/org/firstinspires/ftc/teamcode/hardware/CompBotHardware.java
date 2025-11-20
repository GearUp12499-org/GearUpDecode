package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.systems.REmover;

import io.github.gearup12499.taskshark.Lock;

public class CompBotHardware extends HardwareMapper {

    public static final REmover.RobotPose redFarStart = new REmover.RobotPose(-63, -16, Math.PI);

    public static final REmover.RobotPose redGoalStart = new REmover.RobotPose(54.33,-51.34,2.19);

    public static final  REmover.RobotPose redReadAT = new REmover.RobotPose(46.92, -11.92, -2.55);

    public static final REmover.RobotPose closeShoot = new REmover.RobotPose(60.0, -10.48, 0.55 * Math.PI);
    public static final REmover.RobotPose midShoot = new REmover.RobotPose(12, -12, 3 * Math.PI / 4);
    public static final REmover.RobotPose farShoot = new REmover.RobotPose(-55,-12.39,2.76);
    public static final REmover.RobotPose set1pos = new REmover.RobotPose(12, -33.5, -Math.PI / 2);
    public static final REmover.RobotPose set2pos = new REmover.RobotPose(-36, -33.5, -Math.PI / 2);

    public static final REmover.RobotPose blueBase = new REmover.RobotPose(-38, -33, 0);

    public static final REmover.RobotPose gateWaypoint = new REmover.RobotPose(0, -48, 0);

    public static final REmover.RobotPose gatePos = new REmover.RobotPose(0, -55, 0);

    public static final double INTAKE_POWER = 0.7;
    public static final double FLIPPER_DOWN = 0.515;
    public static final double FLIPPER_UP = 0.900;

    public static final double SHOOT_CLOSE_RANGE = 1160.0;
    public static final double SHOOT_MID_RANGE = 1200.0;
    public static final double SHOOT_FAR_RANGE = 1460.0;

    public static final long GSC_EXPOSURE = 0;
    public static final int GSC_GAIN = 50;

    public static final float COLOR_FRONT_GAIN = 15.0f;


    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontLeft;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("backLeft")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor backLeft;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @HardwareName("indexer")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @AutoClearEncoder
    public DcMotorEx indexer;

    @HardwareName("intake")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor intake;

    @HardwareName("pinpoint")
    public GoBildaPinpoint2Driver pinpoint;

    @HardwareName("idxMag1")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel idxMag1;

    @HardwareName("idxMag2")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel idxMag2;

    @HardwareName("idxMag3")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel idxMag3;

    @HardwareName("idxMag4")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel idxMag4;

    @HardwareName("flipper")
    @GoBildaExtendedServo
    public ServoImplEx flipper;

    @HardwareName("shooter1")
    @ZeroPower(DcMotor.ZeroPowerBehavior.FLOAT)
    @Reversed
    public DcMotorEx shooter1;

    @HardwareName("limelightLight1")
    public Servo limelightLight1;

    @HardwareName("limelightLight2")
    public Servo limelightLight2;

    @HardwareName("indicator1")
    public Servo indicator1;

    @HardwareName("indicator2")
    public Servo indicator2;

    @HardwareName("frontColor1")
    public RevColorSensorV3 frontColor1;

    @HardwareName("frontColor2")
    public RevColorSensorV3 frontColor2;

    @HardwareName("backColor1")
    public RevColorSensorV3 backColor1;

    @HardwareName("backColor2")
    public RevColorSensorV3 backColor2;

    @HardwareName("shooterHood1")
    @GoBildaExtendedServo
    public ServoImplEx shooterHood1;

    @HardwareName("shooterHood2")
    @GoBildaExtendedServo
    public ServoImplEx shooterHood2;

    @HardwareName("Webcam 1")
    public WebcamName gsc;

    public CompBotHardware(HardwareMap map) {
        super(map);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontColor1.setGain(COLOR_FRONT_GAIN);
        frontColor2.setGain(COLOR_FRONT_GAIN);
        backColor1.setGain(45.0f);
        backColor2.setGain(45.0f);

        pinpoint.setOffsets(-3.9, -3.875, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.REVERSED, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);
    }

    public static class Locks {
        /**
         * {@link CompBotHardware#frontLeft}, {@link CompBotHardware#frontRight},
         * {@link CompBotHardware#backLeft}, {@link CompBotHardware#backRight}
         */
        public static final Lock DRIVE_MOTORS = new Lock.StrLock("driveMotors");

        /**
         * {@link CompBotHardware#indexer}, {@link CompBotHardware#flipper}
         */
        public static final Lock INDEXER = new Lock.StrLock("indexer");

        /**
         * {@link CompBotHardware#shooter1}, {@link CompBotHardware#shooterHood1}, {@link CompBotHardware#shooterHood2}
         */
        public static final Lock SHOOTER = new Lock.StrLock("shooter");

        /**
         * {@link CompBotHardware#intake}
         */
        public static final Lock INTAKE = new Lock.StrLock("intake");
    }
}
