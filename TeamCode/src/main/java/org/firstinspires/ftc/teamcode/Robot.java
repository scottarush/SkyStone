package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.TwoWheelTankDrive;

/**
 * Base class for a robot.
 *
 */
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

    protected OpMode opMode;

    private Drivetrain drivetrain;

    /**
     * Base constructor
     * @param activeDTS
     * @param om
     */
    public Robot(DriveTrainStyle activeDTS, OpMode om) {
        this.activeDTS = activeDTS;
        this.opMode = om;

        switch(activeDTS) {
            case MECANUM:
                drivetrain = new MecanumDrive(om);
                break;
            case TWO_WHEEL_TANK:
                drivetrain = new TwoWheelTankDrive(om);
                break;
            default:
                drivetrain = new MecanumDrive(om);
                break;
        }
        // This is for the other hardware for new stuff

    }

    /**
     * returns drivetrain
     */
    public Drivetrain getDrivetrain(){
        return drivetrain;
    }
    /**
     * Base initialization of drivetrain.
     * @throws Exception on any drivetrain init.
     */
    public void init() throws Exception {
        try {
            drivetrain.init(opMode.hardwareMap);
        } catch (Exception e) {
            throw new Exception("Drivetrain init err: "+e.getMessage());
        }
    }

}
