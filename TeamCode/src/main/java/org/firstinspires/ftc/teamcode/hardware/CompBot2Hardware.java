package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import android.util.Pair;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PoseSet;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.drivers.IGoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.drivers.NoOpPrism;
import org.firstinspires.ftc.teamcode.systems.REmover;
import org.firstinspires.ftc.teamcode.utilities.StaticStore;

import io.github.gearup12499.taskshark.Lock;
import kotlin.Triple;

public class CompBot2Hardware extends HardwareMapper {
    public static final double DROP_DOWN_SWEET_SPOT = 0.49;
    public static final double DROP_DOWN_BOTTOM = 0.44;
    public static final double DROP_DOWN_TOP = 0.64;

    public static final double SLIDER_OUT = 0.10;
    public static final double SLIDER_MIDDLE = 0.25;
    public static final double SLIDER_IN = 0.95;

    public static final double TICKS_PER_DEG = 159.5;
    public static final int TURRET_CW_STOP = 9400;
    public static final int TURRET_CCW_STOP = -9400;

    public static final double BALL_STOP_STOWED = 0.37;
    public static final double BALL_STOP_MIDDLE = 0.47;

    public static final double FLIPPER_DOWN = 0.25;
    public static final double FLIPPER_MID = 0.50;
    public static final double FLIPPER_UP = 0.70;

    public static final double HOOD_UP = 0.5578;
    public static final double HOOD_50 = 0.3700;
    public static final double HOOD_25 = 0.2756;
    public static final double HOOD_DOWN = 0.1817;

    public static final double BOTTOM_BALL_STOP = 0.54;
    public static final double BOTTOM_STOP_STOWED = 0.40;
    public static final double BOTTOM_STOP_OUT = 0.15;

    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = -0.60;

    public static final double SHOOT_CLOSE_RANGE = 1260.0;
    public static final double SHOOT_MID_RANGE = 1340.0;
    public static final double SHOOT_MID_RANGE2 = 1360.0;
    public static final double SHOOT_FAR_RANGE = 1940.0;

    public static final double SHOOT_FAR_RANGE_AUTO = 1880.0;// x < 24

    public static final double SHOOT_MAX_DIST = 108.0;
    public static final double SHOOT_HOOD_UP_DIST = 32.0;
    public static final double SHOOT_MIN_DIST = 21.26; // based on 42, -42

    // UP 0.13 DOWN 0.42
    public static final double SHOOTER_STOP_UP = 0.63;
    public static final double SHOOTER_STOP_DOWN = 1;

    @HardwareName("limelight")
    public Limelight3A limelight;

    @HardwareName("imu")
    public IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
    ));

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx frontRight;

    @HardwareName("backRight")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx backRight;

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx frontLeft;

    @HardwareName("backLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx backLeft;

    @HardwareName("intake1")
    private DcMotorEx intake1;

    @HardwareName("intake2")
    @Reversed
    private DcMotorEx intake2;

    @EncoderFor("intake2")
    @Reversed
    public Encoder turretEncoder;

    @HardwareName("shoot1")
    @Reversed
    private DcMotorEx shoot1;

    @HardwareName("shoot2")
    private DcMotorEx shoot2;

    @HardwareName("flipper")
    public ServoImplEx flipper;

    @HardwareName("bottomBallStop")
    public ServoImplEx bottomBallStop;

    @HardwareName("ballStopEncoder")
    public AnalogInput ballStopEncoder;

    @HardwareName("hood")
    public ServoImplEx hood;

    @HardwareName("hoodEncoder")
    public AnalogInput hoodEncoder;

    @HardwareName("slider")
    public ServoImplEx slider;

    @HardwareName("sliderEncoder")
    public AnalogInput sliderEncoder;

    @HardwareName("shooterBallStop")
    public ServoImplEx shooterBallStop;

    @HardwareName("pinpoint")
    public GoBildaPinpoint2Driver pinpoint;

//    @HardwareName("distanceRight")
//    public Rev2mDistanceSensor distanceRight;

    @HardwareName("distanceLeft")
    public Rev2mDistanceSensor distanceLeft;

    @HardwareName("colorTopRight")
    public RevColorSensorV3 colorTopRight;

    @HardwareName("colorBottomRight")
    public RevColorSensorV3 colorBottomRight;

    @HardwareName("colorTopLeft")
    public RevColorSensorV3 colorTopLeft;

    @HardwareName("colorBottomLeft")
    public RevColorSensorV3 colorBottomLeft;

    @HardwareName("prism")
    public GoBildaPrismDriver actualPrism;

    public IGoBildaPrismDriver prism;

    @HardwareName("frontRamp")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel frontRamp;

    @HardwareName("middleRamp")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel middleRamp;

    @HardwareName("Webcam 1")
    public WebcamName webcam1;

    @HardwareName("Webcam 2")
    public WebcamName webcam2;

    @HardwareName("turret1")
    public CRServo servoTurret1;

    @HardwareName("turret2")
    public CRServo servoTurret2;


    public CompBot2Hardware(HardwareMap map) {
        super(map);

        pinpoint.setOffsets(-2.933, -5.020, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.REVERSED, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);

//        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(380, 40, 20, 0));
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(760, 40, 20, 0));

        refreshPrismState();
    }

    public void refreshPrismState() {
        if (StaticStore.INSTANCE.getPrismBroken())
            prism = NoOpPrism.INSTANCE;
        else prism = actualPrism;
    }

    // move on init is banned in the auto-teleop transition
    public void initMotion() {
//        dropDown.setPosition(DROP_DOWN_SWEET_SPOT);
    }

    private boolean shooterMode = false;

    private void setupShooterPow() {
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMode = false;
    }

    public void setShooterPower(double power) {
        if (shooterMode) setupShooterPow();
        shoot1.setPower(power);
        shoot2.setPower(power);
    }
    public double getShoot1Vel() {
        return shoot1.getVelocity();
    }
    public double getShoot1Power() {
        return shoot1.getPower();
    }

    public double gethoodpos(){
        return hood.getPosition();
    }

    public void setIntakePower(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public void setTurretPower(double power) {
        // TODO: negate?
//        Log.i("Hardware", String.format("set the power to %.2f", -power));
        servoTurret1.setPower(-power);
        servoTurret2.setPower(-power);
    }

    public double getTurretPower() {
        double pow1 = servoTurret1.getPower();
        double pow2 = servoTurret2.getPower();
        return (pow1 + pow2) / 2;
    }

    private void setupShooterVel1() {
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMode = true;
    }

    public void setupShooterVel() {
        setupShooterVel1();
        shoot1.setVelocity(0);
    }

    public void setShoot1Vel(double vel) {
        if (!shooterMode) setupShooterVel1();
        shoot1.setVelocity(vel);
    }
    public void copyShooterPower() {
        if (!shooterMode) setupShooterVel1();
        shoot2.setPower(shoot1.getPower());
    }

    public static class Locks {
        public static final Lock.StrLock DRIVE_MOTORS = new Lock.StrLock("drive_motors");
        public static final Lock.StrLock INTAKE_STORAGE = new Lock.StrLock("intake_storage");
    }

    /**
     * @param distance in inches
     * @return hood, speed
     */
    public static Pair<Double, Double> hoodAndSpeed(double distance) {
        if (distance > SHOOT_MAX_DIST) {
            return new Pair<>(HOOD_UP, SHOOT_FAR_RANGE_AUTO);
        }
        double speed = 6.81246 * distance + 1075.16505;
        double hood = 0.00492724 * distance + 0.0769453;
        if (hood > 0.5578) hood = 0.5578;
        else if (hood < 0.1817) hood = 0.1817;
        return new Pair<>(hood, speed);
    }

    //ROBOT VELX and VELY MUST BE IN METERS/SECOND
    public static Triple<Double, Double, Double> hoodAndSpeedAndTurret(double robotVelX, double robotVelY, REmover.RobotPose goalPose, Pose2D robotPose){
//        if (distance > SHOOT_MAX_DIST) {
//            return new Triple<>(HOOD_UP, SHOOT_FAR_RANGE_AUTO,????);

        double distance = Math.hypot((goalPose.x-robotPose.getX(DistanceUnit.INCH)),(goalPose.y-robotPose.getY(DistanceUnit.INCH)));

        //distance into speed and hood
        double speed = 6.81246 * distance + 1075.16505;
        double hood = 0.00492724 * distance + 0.0769453;

        //hood into percent of total range
        double hoodPercent = (hood-HOOD_DOWN)/(HOOD_UP-HOOD_DOWN)*100;

        //use model to convert speed and hoodPercent into a V and Theta
        double vB = 0.00331412 * speed + 0.426853;
        double thetaB = -0.181376 * hoodPercent + 71.29116;

        //thetaB into radians for sin and cos
        thetaB = thetaB * Math.PI/180;

        //azimuthal angle
        double alphaB = Math.atan2(goalPose.y - robotPose.getY(DistanceUnit.INCH), goalPose.x - robotPose.getX(DistanceUnit.INCH));

        //break up velocity of the ball with respect to the ground into components
        double vBz = vB*Math.sin(thetaB);
        double vBh = vB*Math.cos(thetaB);
        double vBx = vBh*Math.cos(alphaB);
        double vBy = vBh*Math.sin(alphaB);

        //velocity of ball with respect to robot (v)
        double vx = vBx - robotVelX;
        double vy = vBy - robotVelY;
        double vz = vBz;

        double alpha = Math.atan2(vy,vx);
        double vh = Math.hypot(vx,vy);
        double theta = Math.atan2(vz, vh);
        theta = theta * 180/Math.PI;
        double v = Math.sqrt((vx*vx)+(vy*vy)+(vz*vz));

        //use model to convert to hood and speed, alpha is turret angle
        double finalHood = (theta-71.29116)/(-0.181376);
        finalHood = (finalHood/100)*(HOOD_UP-HOOD_DOWN) + HOOD_DOWN;
        double finalSpeed = (v-0.426853)/(0.00331412);

        alpha = alpha * 180/Math.PI;
        alpha = alpha % 360;
        if (alpha > 180) {
            alpha -= 360;
        }
        if (alpha < -180) {
            alpha += 360;
        }
        double finalTurret = -180 + alpha - robotPose.getHeading(AngleUnit.DEGREES);
        finalTurret = finalTurret % 360;
        if (finalTurret > 180){
            finalTurret -= 360;
        }
        if (finalTurret < -180){
            finalTurret += 360;
        }

        //add something for the case that it is not possible
        if (finalHood > 0.5578) finalHood = 0.5578;
        else if (finalHood < 0.1817) finalHood = 0.1817;

        if (finalSpeed < 0){
            finalSpeed = 0;
        }
        //hood over 0.5578, negative speed

        alphaB = alphaB * 180/Math.PI;


        //return new Triple<>(alpha, alphaB, finalTurret);



        return new Triple<>(finalHood, finalSpeed, finalTurret);


    }
}


