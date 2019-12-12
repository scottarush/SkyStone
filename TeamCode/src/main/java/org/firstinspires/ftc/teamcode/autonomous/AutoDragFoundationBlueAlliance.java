package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="BlueAllianceDragFoundation", group="Robot")
//@Disabled
public class AutoDragFoundationBlueAlliance extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public AutoDragFoundationBlueAlliance(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, true,AutonomousController.SEQUENCE_DRAG_FOUNDATION);
    }

    @Override
    public void init() {
        mRealOpmode.init();
    }

    @Override
    public void loop() {
        mRealOpmode.loop();
    }


}


