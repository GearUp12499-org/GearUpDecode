package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.systems.REmover;
import org.firstinspires.ftc.teamcode.tools.GearUpMath;

import io.github.gearup12499.taskshark.Lock;

public class CompBotHardware extends HardwareMapper {
    public static final double INTAKE_POWER = 0.7;
    public static final double FLIPPER_DOWN = 0.500;
    public static final double FLIPPER_UP = 0.900;

    public static final double SHOOT_CLOSE_RANGE = 1160.0;
    public static final double SHOOT_MID_RANGE = 1140.0;
    public static final double SHOOT_FAR_RANGE = 1400.0;

    public static final long GSC_EXPOSURE = 0;
    public static final int GSC_GAIN = 100;

    public static final float COLOR_FRONT_GAIN = 15.0f;

    public static final double HOOD_UP = 1.0;
    public static final double HOOD_DOWN = 0.7;
    public static final double SHOOT_MIN_DIST = 24.0;
    public static final double SHOOT_HOOD_UP_DIST = 40.0;

    public static final double COLOR_RED = 0.3;

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
        Log.i("Hardware", shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        /*
         * these GOATED PID coefficients courtesy of 19075 Clockworks
         * https://youtu.be/phrrq8zaOAU
         */
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(380, 40, 20, 0));
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontColor1.setGain(COLOR_FRONT_GAIN);
        frontColor2.setGain(COLOR_FRONT_GAIN);

        shooterHood1.setPosition(HOOD_UP);

        pinpoint.setOffsets(-3.9, -3.875 + 0.05, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.REVERSED, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);
    }

    public void integratePositionData(REmover.RobotPose pose) {
//        Pose2D current = pinpoint.getPosition();
//        Log.i(
//                "gearup",
//                String.format(
//                        "Integrating new pose data.\nPinpoint: %.2f %.2f in %.1f deg\nAprilTag: %.2f %.2f in %.1f deg",
//                        current.getX(DistanceUnit.INCH),
//                        current.getY(DistanceUnit.INCH),
//                        current.getHeading(AngleUnit.DEGREES),
//                        pose.x,
//                        pose.y,
//                        Math.toDegrees(pose.a)
//                )
//        );
//        Pose2D newPose = new Pose2D(
//                DistanceUnit.INCH,
//                (current.getX(DistanceUnit.INCH) + pose.x) / 2.0,
//                (current.getY(DistanceUnit.INCH) + pose.y) / 2.0,
//                AngleUnit.RADIANS,
//                GearUpMath.wrapAngle((current.getHeading(AngleUnit.RADIANS) + pose.a) / 2.0)
//        );
        // slow!
//        pinpoint.setPosition(newPose);
    }

    public static boolean isHoodUp(double distance) {
        return distance > SHOOT_HOOD_UP_DIST;
    }

    public static double speedForHoodUp(double distance) {
        return 5.67 * distance + 784;
    }

    public static double speedForHoodDown(double distance) {
        return 6.19 * distance + 786;
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
