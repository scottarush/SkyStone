package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.TwoWheelTankDrive;

public class Robot {

    public enum DriveTrainStyle{
        MECANUM(0),
        TWO_WHEEL_TANK(1);

        private final int id;

        DriveTrainStyle(int id) {
            this.id = id;
        }

        public int getId() { return id; }
    }

    private DriveTrainStyle activeDTS;

    private OpMode om;

    public Drivetrain drivetrain;

    public Robot(DriveTrainStyle activeDTS, OpMode om) {
        this.activeDTS = activeDTS;
        this.om = om;

        switch(activeDTS) {
            case MECANUM:
                drivetrain = new MecanumDrive();
                break;
            case TWO_WHEEL_TANK:
                drivetrain = new TwoWheelTankDrive();
                break;
            default:
                drivetrain = new MecanumDrive();
                break;
        }
        // This is for the other hardware for new stuff

    }

    public void init() {
        try {
            drivetrain.init(om.hardwareMap);
        } catch (Exception e) {
            om.telemetry.addData("Oh no! ","DriveTrain init failed");
            om.telemetry.update();
        }
    }

}
