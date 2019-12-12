package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="RedAllianceGetStone", group="Robot")
//@Disabled
public class AutoGetStoneRedAlliance extends OpMode {
    private SkystoneAutonomousOpMode mRealOpmode = null;
    public AutoGetStoneRedAlliance(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, false,AutonomousController.SEQUENCE_GET_STONE);
    }

    @Override
    public void init() {
        mRealOpmode.init();
    }

    @Override
    public void loop() {
        mRealOpmode.loop();
    }}


