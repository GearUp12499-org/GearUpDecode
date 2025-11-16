package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.systems.REmover;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import io.github.gearup12499.taskshark.Lock;

public class CompBotHardware extends HardwareMapper {

    public static final REmover.RobotPose redFarStart = new REmover.RobotPose(-63, -16, 0);
    public static final Pose2D redFarStartPose = new Pose2D(DistanceUnit.INCH, -63, -16, AngleUnit.RADIANS, Math.PI);

    public static final REmover.RobotPose shootPos = new REmover.RobotPose(12, -12, 3 * Math.PI / 4);

    public static final REmover.RobotPose blueBase = new REmover.RobotPose(-38, -33, 0);

    public static final REmover.RobotPose gateWaypoint = new REmover.RobotPose(0, -48, 0);

    public static final REmover.RobotPose gatePos = new REmover.RobotPose(0, -55, 0);

    public static final double FLIPPER_DOWN = 0.515;
    public static final double FLIPPER_UP = 0.900;
    public static final double SHOOT_MIDRANGE = 1200.0;


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
        frontColor1.setGain(45.0f);
        frontColor2.setGain(45.0f);
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
         * {@link CompBotHardware#intake}
         */
        public static final Lock INTAKE = new Lock.StrLock("intake");
    }
}
