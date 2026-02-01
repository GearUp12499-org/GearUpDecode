package org.firstinspires.ftc.teamcode.hardware;

import android.util.Pair;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.drivers.IGoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.drivers.NoOpPrism;
import org.firstinspires.ftc.teamcode.utilities.StaticStore;

import io.github.gearup12499.taskshark.Lock;

public class CompBot2Hardware extends HardwareMapper {
    public static final double DROP_DOWN_SWEET_SPOT = 0.49;
    public static final double DROP_DOWN_BOTTOM = 0.44;
    public static final double DROP_DOWN_TOP = 0.64;

    public static final double SLIDER_OUT = 0.10;
    public static final double SLIDER_MIDDLE = 0.25;
    public static final double SLIDER_IN = 0.95;

    public static final int TURRET_CW_90 = 230;
    public static final int TURRET_CW_STOP = 345;
    public static final int TURRET_CCW_90 = -230;
    public static final int TURRET_CCW_STOP = -345;

    public static final double BALL_STOP_STOWED = 0.37;
    public static final double BALL_STOP_MIDDLE = 0.47;

    public static final double FLIPPER_DOWN = 0.25;
    public static final double FLIPPER_MID = 0.50;
    public static final double FLIPPER_UP = 0.70;

    public static final double HOOD_UP = 0.5578;
    public static final double HOOD_50 = 0.3700;
    public static final double HOOD_DOWN = 0.1817;

    public static final double BOTTOM_BALL_STOP = 0.54;
    public static final double BOTTOM_STOP_STOWED = 0.40;
    public static final double BOTTOM_STOP_OUT = 0.15;

    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = -0.60;

    public static final double SHOOT_MID_RANGE = 1290.0;
    public static final double SHOOT_MID_RANGE2 = 1310.0;
    public static final double SHOOT_FAR_RANGE = 1840.0;

    public static final double SHOOT_HOOD_UP_DIST = 32.0;
    public static final double SHOOT_MIN_DIST = 20.0;

    // UP 0.13 DOWN 0.42
    public static final double SHOOTER_STOP_UP = 0.33;
    public static final double SHOOTER_STOP_DOWN = 0.66;

    @HardwareName("limelight")
    public Limelight3A limelight;

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

    @HardwareName("turret")
    @AutoClearEncoder
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx turret;

    @HardwareName("intake")
    @Reversed
    public DcMotorEx intake;

    @HardwareName("shoot1")
    @Reversed
    private DcMotorEx shoot1;

    @HardwareName("shoot2")
    private DcMotorEx shoot2;

    @HardwareName("ballStop")
    public ServoImplEx ballStop;

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

    public CompBot2Hardware(HardwareMap map) {
        super(map);

        pinpoint.setOffsets(-2.933, -5.020, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.REVERSED, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);

        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(380, 40, 20, 0));

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

    public double gethoodpos(){
        return hood.getPosition();
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
        boolean isUp = distance >= SHOOT_HOOD_UP_DIST;
        double hood = isUp ? HOOD_50 : HOOD_DOWN;
        double speed;
        if (isUp) speed = 8.0 * distance + 990;
        else speed = 8.92 * distance + 1014;
        return new Pair<>(hood, speed);
    }
}
