/*
 * ex: set ro:
 * DO NOT EDIT.
 * generated by smc (http://smc.sourceforge.net/)
 * from file : GrabberBotAutoStateMachine.sm
 */

package org.firstinspires.ftc.teamcode.autonomous;


public class GrabberBotAutoStateMachineContext
    extends statemap.FSMContext
{
//---------------------------------------------------------------
// Member methods.
//

    public GrabberBotAutoStateMachineContext(AutonomousController owner)
    {
        this (owner, GrabberBotAutoStateMachine.Idle);
    }

    public GrabberBotAutoStateMachineContext(AutonomousController owner, AutonomousControllerState initState)
    {
        super (initState);

        _owner = owner;
    }

    @Override
    public void enterStartState()
    {
        getState().entry(this);
        return;
    }

    public void evDriveComplete()
    {
        _transition = "evDriveComplete";
        getState().evDriveComplete(this);
        _transition = "";
        return;
    }

    public void evDriveFail()
    {
        _transition = "evDriveFail";
        getState().evDriveFail(this);
        _transition = "";
        return;
    }

    public void evHookTimeout()
    {
        _transition = "evHookTimeout";
        getState().evHookTimeout(this);
        _transition = "";
        return;
    }

    public void evNoStoneFound()
    {
        _transition = "evNoStoneFound";
        getState().evNoStoneFound(this);
        _transition = "";
        return;
    }

    public void evRotationComplete()
    {
        _transition = "evRotationComplete";
        getState().evRotationComplete(this);
        _transition = "";
        return;
    }

    public void evSkystoneFound()
    {
        _transition = "evSkystoneFound";
        getState().evSkystoneFound(this);
        _transition = "";
        return;
    }

    public void evStartDragFoundation()
    {
        _transition = "evStartDragFoundation";
        getState().evStartDragFoundation(this);
        _transition = "";
        return;
    }

    public void evStartDriveToStones()
    {
        _transition = "evStartDriveToStones";
        getState().evStartDriveToStones(this);
        _transition = "";
        return;
    }

    public void evStoneFound()
    {
        _transition = "evStoneFound";
        getState().evStoneFound(this);
        _transition = "";
        return;
    }

    public AutonomousControllerState getState()
        throws statemap.StateUndefinedException
    {
        if (_state == null)
        {
            throw(
                new statemap.StateUndefinedException());
        }

        return ((AutonomousControllerState) _state);
    }

    protected AutonomousController getOwner()
    {
        return (_owner);
    }

    public void setOwner(AutonomousController owner)
    {
        if (owner == null)
        {
            throw (
                new NullPointerException(
                    "null owner"));
        }
        else
        {
            _owner = owner;
        }

        return;
    }

//---------------------------------------------------------------
// Member data.
//

    transient private AutonomousController _owner;

    //-----------------------------------------------------------
    // Constants.
    //

    private static final long serialVersionUID = 1L;

//---------------------------------------------------------------
// Inner classes.
//

    public static abstract class AutonomousControllerState
        extends statemap.State
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected AutonomousControllerState(String name, int id)
        {
            super (name, id);
        }

        protected void entry(GrabberBotAutoStateMachineContext context) {}
        protected void exit(GrabberBotAutoStateMachineContext context) {}

        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evDriveFail(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evHookTimeout(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evNoStoneFound(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evRotationComplete(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evSkystoneFound(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evStartDragFoundation(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evStartDriveToStones(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void evStoneFound(GrabberBotAutoStateMachineContext context)
        {
            Default(context);
        }

        protected void Default(GrabberBotAutoStateMachineContext context)
        {
            throw (
                new statemap.TransitionUndefinedException(
                    "State: " +
                    context.getState().getName() +
                    ", Transition: " +
                    context.getTransition()));
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
    // Constants.
    //

        private static final long serialVersionUID = 1L;
    }

    /* package */ static abstract class GrabberBotAutoStateMachine
    {
    //-----------------------------------------------------------
    // Member methods.
    //

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
        // Constants.
        //

        public static final GrabberBotAutoStateMachine_Idle Idle =
            new GrabberBotAutoStateMachine_Idle("GrabberBotAutoStateMachine.Idle", 0);
    }

    protected static class GrabberBotAutoStateMachine_Default
        extends AutonomousControllerState
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected GrabberBotAutoStateMachine_Default(String name, int id)
        {
            super (name, id);
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GrabberBotAutoStateMachine_Idle
        extends GrabberBotAutoStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GrabberBotAutoStateMachine_Idle(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void evStartDragFoundation(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.Start);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evStartDriveToStones(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.Start);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    /* package */ static abstract class GetStones
    {
    //-----------------------------------------------------------
    // Member methods.
    //

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
        // Constants.
        //

        public static final GetStones_Start Start =
            new GetStones_Start("GetStones.Start", 1);
        public static final GetStones_StrafeToStone StrafeToStone =
            new GetStones_StrafeToStone("GetStones.StrafeToStone", 2);
        public static final GetStones_DriveForwardToIntake DriveForwardToIntake =
            new GetStones_DriveForwardToIntake("GetStones.DriveForwardToIntake", 3);
        public static final GetStones_IntakeStone IntakeStone =
            new GetStones_IntakeStone("GetStones.IntakeStone", 4);
        public static final GetStones_BackupToBeginDrag BackupToBeginDrag =
            new GetStones_BackupToBeginDrag("GetStones.BackupToBeginDrag", 5);
        public static final GetStones_BlueAllianceRotateTowardBridge BlueAllianceRotateTowardBridge =
            new GetStones_BlueAllianceRotateTowardBridge("GetStones.BlueAllianceRotateTowardBridge", 6);
        public static final GetStones_RedAllianceRotateTowardBridge RedAllianceRotateTowardBridge =
            new GetStones_RedAllianceRotateTowardBridge("GetStones.RedAllianceRotateTowardBridge", 7);
        public static final GetStones_DragStone DragStone =
            new GetStones_DragStone("GetStones.DragStone", 8);
        public static final GetStones_ReleaseStone ReleaseStone =
            new GetStones_ReleaseStone("GetStones.ReleaseStone", 9);
        public static final GetStones_Complete Complete =
            new GetStones_Complete("GetStones.Complete", 10);
    }

    protected static class GetStones_Default
        extends AutonomousControllerState
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected GetStones_Default(String name, int id)
        {
            super (name, id);
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_Start
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_Start(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.openHook();
            ctxt.linearDrive(10d);
            return;
        }

        @Override
        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {
            AutonomousController ctxt = context.getOwner();

            AutonomousControllerState endState = context.getState();
            context.clearState();
            try
            {
                ctxt.checkStoneRecognition();
            }
            finally
            {
                context.setState(endState);
            }

            return;
        }

        @Override
        protected void evDriveFail(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.DriveForwardToIntake);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evNoStoneFound(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.DriveForwardToIntake);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evSkystoneFound(GrabberBotAutoStateMachineContext context)
        {
            AutonomousController ctxt = context.getOwner();

            (context.getState()).exit(context);
            context.clearState();
            try
            {
                ctxt.strafeToSkystone(10d, 2000);
            }
            finally
            {
                context.setState(GetStones.StrafeToStone);
                (context.getState()).entry(context);
            }

            return;
        }

        @Override
        protected void evStoneFound(GrabberBotAutoStateMachineContext context)
        {
            AutonomousController ctxt = context.getOwner();

            (context.getState()).exit(context);
            context.clearState();
            try
            {
                ctxt.strafeToStone(10d, 2000);
            }
            finally
            {
                context.setState(GetStones.StrafeToStone);
                (context.getState()).entry(context);
            }

            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_StrafeToStone
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_StrafeToStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.DriveForwardToIntake);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_DriveForwardToIntake
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_DriveForwardToIntake(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.linearDrive(10d);
            return;
        }

        @Override
        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.IntakeStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_IntakeStone
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_IntakeStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.closeHook();
            ctxt.startHookTimer();
            ctxt.startGrabber(true, 1000);
            return;
        }

        @Override
        protected void evHookTimeout(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.BackupToBeginDrag);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_BackupToBeginDrag
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_BackupToBeginDrag(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.linearDrive(-18d);
            return;
        }

        @Override
        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {
            AutonomousController ctxt = context.getOwner();

            if (ctxt.isBlueAlliance() == true)
            {
                (context.getState()).exit(context);
                // No actions.
                context.setState(GetStones.BlueAllianceRotateTowardBridge);
                (context.getState()).entry(context);
            }
            else if (ctxt.isBlueAlliance() == false)
            {
                (context.getState()).exit(context);
                // No actions.
                context.setState(GetStones.RedAllianceRotateTowardBridge);
                (context.getState()).entry(context);
            }            else
            {
                super.evDriveComplete(context);
            }

            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_BlueAllianceRotateTowardBridge
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_BlueAllianceRotateTowardBridge(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.rotate(+90);
            return;
        }

        @Override
        protected void evRotationComplete(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.DragStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_RedAllianceRotateTowardBridge
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_RedAllianceRotateTowardBridge(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.rotate(-90);
            return;
        }

        @Override
        protected void evRotationComplete(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.DragStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_DragStone
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_DragStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.linearDrive(72d);
            return;
        }

        @Override
        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.ReleaseStone);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_ReleaseStone
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_ReleaseStone(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.openHook();
            ctxt.startGrabber(false, 3000);
            ctxt.linearDrive(-37d);
            return;
        }

        @Override
        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(GetStones.Complete);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class GetStones_Complete
        extends GetStones_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private GetStones_Complete(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.stopGrabber();
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    /* package */ static abstract class DragFoundation
    {
    //-----------------------------------------------------------
    // Member methods.
    //

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
        // Constants.
        //

        public static final DragFoundation_Start Start =
            new DragFoundation_Start("DragFoundation.Start", 11);
    }

    protected static class DragFoundation_Default
        extends AutonomousControllerState
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected DragFoundation_Default(String name, int id)
        {
            super (name, id);
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class DragFoundation_Start
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_Start(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(GrabberBotAutoStateMachineContext context)
            {
                AutonomousController ctxt = context.getOwner();

            ctxt.openHook();
            ctxt.linearDrive(10d);
            return;
        }

        @Override
        protected void evDriveComplete(GrabberBotAutoStateMachineContext context)
        {
            AutonomousController ctxt = context.getOwner();

            AutonomousControllerState endState = context.getState();
            context.clearState();
            try
            {
                ctxt.checkStoneRecognition();
            }
            finally
            {
                context.setState(endState);
            }

            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }
}

/*
 * Local variables:
 *  buffer-read-only: t
 * End:
 */
