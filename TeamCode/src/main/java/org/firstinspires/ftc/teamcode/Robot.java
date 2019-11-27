package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.arm.FourBarArm;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.FrameDevBotMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.GrabberBotMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.TwoWheelTankDrive;

/**
 * Base class for a robot.
 *
 */
public abstract class Robot {

    public enum DriveTrainStyle {
        GRABBER_BOT_MECANUM_DRIVE(0),
        FRAME_DEV_BOT_MECANUM_DRIVE(1),
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
            case GRABBER_BOT_MECANUM_DRIVE:
                drivetrain = new GrabberBotMecanumDrive(om);
                break;
            case FRAME_DEV_BOT_MECANUM_DRIVE:
                drivetrain = new FrameDevBotMecanumDrive(om);
                break;
            case TWO_WHEEL_TANK:
                drivetrain = new TwoWheelTankDrive(om);
                break;
            default:
                drivetrain = new GrabberBotMecanumDrive(om);
                break;
        }
        // This is for the other hardware for new stuff

    }

    /**
     * Must be implemeented as real or dummy arm by subclasses
     * @return
     */
    public abstract FourBarArm getArm();

    /**
     * Must be implemeented as real or dummy grabber by subclasses
     * @return
     */
    public abstract Grabber getGrabber();
    /**
     * Must be implemeented as real or dummy hook by subclasses
     * @return
     */
    public abstract Hook getHook();


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
