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
        MECANUM_HEX_BOT(0),
        MECANUM_REV_DEV_BOT(1),
        TWO_WHEEL_TANK(2);

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
            case MECANUM_HEX_BOT:
                drivetrain = new MecanumDrive(om,false);
                break;
            case MECANUM_REV_DEV_BOT:
                drivetrain = new MecanumDrive(om,true);
                break;
            case TWO_WHEEL_TANK:
                drivetrain = new TwoWheelTankDrive(om);
                break;
            default:
                drivetrain = new MecanumDrive(om,false);
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
