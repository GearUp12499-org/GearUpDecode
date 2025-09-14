package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;

public class DecodeHardware extends HardwareMapper {

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontLeft;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @HardwareName("backLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backLeft;

    @HardwareName("imu")
    public IMU imu;

    public DecodeHardware(HardwareMap map) {
        super(map);
    }
}
