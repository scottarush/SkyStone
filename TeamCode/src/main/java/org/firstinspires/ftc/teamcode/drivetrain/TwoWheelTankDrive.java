package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelTankDrive extends Drivetrain {

    //TODO - Hardware Here

    public  TwoWheelTankDrive(OpMode opMode) {
        super(opMode,7000,50);
    }

    @Override
    public void init(HardwareMap ahwMap) throws Exception {

    }

    @Override
    public void setPower(double lf, double rf, double lr, double rr) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void setMotorModes(DcMotor.RunMode mode) {

    }
    /**
     * @return true if the drivetrain is moving.  false if stopped.
     */
    public boolean isMoving(){
        return false;
    }

}
