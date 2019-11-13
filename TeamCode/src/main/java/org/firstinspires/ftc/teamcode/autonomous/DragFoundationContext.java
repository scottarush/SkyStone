/*
 * ex: set ro:
 * DO NOT EDIT.
 * generated by smc (http://smc.sourceforge.net/)
 * from file : DragFoundation.sm
 */

package org.firstinspires.ftc.teamcode.autonomous;


public class DragFoundationContext
    extends statemap.FSMContext
{
//---------------------------------------------------------------
// Member methods.
//

    public DragFoundationContext(DragFoundationController owner)
    {
        this (owner, DragFoundation.Idle);
    }

    public DragFoundationContext(DragFoundationController owner, DragFoundationControllerState initState)
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

    public void evArmRetracted()
    {
        _transition = "evArmRetracted";
        getState().evArmRetracted(this);
        _transition = "";
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

    public void evRotationComplete()
    {
        _transition = "evRotationComplete";
        getState().evRotationComplete(this);
        _transition = "";
        return;
    }

    public void evStart()
    {
        _transition = "evStart";
        getState().evStart(this);
        _transition = "";
        return;
    }

    public DragFoundationControllerState getState()
        throws statemap.StateUndefinedException
    {
        if (_state == null)
        {
            throw(
                new statemap.StateUndefinedException());
        }

        return ((DragFoundationControllerState) _state);
    }

    protected DragFoundationController getOwner()
    {
        return (_owner);
    }

    public void setOwner(DragFoundationController owner)
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

    transient private DragFoundationController _owner;

    //-----------------------------------------------------------
    // Constants.
    //

    private static final long serialVersionUID = 1L;

//---------------------------------------------------------------
// Inner classes.
//

    public static abstract class DragFoundationControllerState
        extends statemap.State
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected DragFoundationControllerState(String name, int id)
        {
            super (name, id);
        }

        protected void entry(DragFoundationContext context) {}
        protected void exit(DragFoundationContext context) {}

        protected void evArmRetracted(DragFoundationContext context)
        {
            Default(context);
        }

        protected void evDriveComplete(DragFoundationContext context)
        {
            Default(context);
        }

        protected void evDriveFail(DragFoundationContext context)
        {
            Default(context);
        }

        protected void evHookTimeout(DragFoundationContext context)
        {
            Default(context);
        }

        protected void evRotationComplete(DragFoundationContext context)
        {
            Default(context);
        }

        protected void evStart(DragFoundationContext context)
        {
            Default(context);
        }

        protected void Default(DragFoundationContext context)
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

        public static final DragFoundation_Idle Idle =
            new DragFoundation_Idle("DragFoundation.Idle", 0);
        public static final DragFoundation_DriveForward DriveForward =
            new DragFoundation_DriveForward("DragFoundation.DriveForward", 1);
        public static final DragFoundation_DriveForwardSuccess DriveForwardSuccess =
            new DragFoundation_DriveForwardSuccess("DragFoundation.DriveForwardSuccess", 2);
        public static final DragFoundation_DriveForwardFail DriveForwardFail =
            new DragFoundation_DriveForwardFail("DragFoundation.DriveForwardFail", 3);
        public static final DragFoundation_DragFoundation DragFoundation =
            new DragFoundation_DragFoundation("DragFoundation.DragFoundation", 4);
        public static final DragFoundation_RotateToQuarry RotateToQuarry =
            new DragFoundation_RotateToQuarry("DragFoundation.RotateToQuarry", 5);
        public static final DragFoundation_PrepareDriveToQuarry PrepareDriveToQuarry =
            new DragFoundation_PrepareDriveToQuarry("DragFoundation.PrepareDriveToQuarry", 6);
        public static final DragFoundation_DriveToQuarry DriveToQuarry =
            new DragFoundation_DriveToQuarry("DragFoundation.DriveToQuarry", 7);
        public static final DragFoundation_RotateToBlock RotateToBlock =
            new DragFoundation_RotateToBlock("DragFoundation.RotateToBlock", 8);
        public static final DragFoundation_Success Success =
            new DragFoundation_Success("DragFoundation.Success", 9);
    }

    protected static class DragFoundation_Default
        extends DragFoundationControllerState
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

    private static final class DragFoundation_Idle
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_Idle(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void evStart(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.DriveForward);
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

    private static final class DragFoundation_DriveForward
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_DriveForward(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("DriveForward");
            ctxt.openHook();
            ctxt.driveToFoundation();
            return;
        }

        @Override
        protected void evDriveComplete(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.DriveForwardSuccess);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evDriveFail(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.DriveForwardFail);
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

    private static final class DragFoundation_DriveForwardSuccess
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_DriveForwardSuccess(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("DriveForwardSuccess");
            ctxt.closeHook();
            ctxt.startHookTimer();
            return;
        }

        @Override
        protected void evHookTimeout(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.DragFoundation);
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

    private static final class DragFoundation_DriveForwardFail
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_DriveForwardFail(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("DriveForwardFail");
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

    private static final class DragFoundation_DragFoundation
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_DragFoundation(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("DragFoundation");
            ctxt.dragFoundation();
            return;
        }

        @Override
        protected void evDriveComplete(DragFoundationContext context)
        {
            DragFoundationController ctxt = context.getOwner();

            DragFoundationControllerState endState = context.getState();
            context.clearState();
            try
            {
                ctxt.openHook();
                ctxt.startHookTimer();
            }
            finally
            {
                context.setState(endState);
            }

            return;
        }

        @Override
        protected void evHookTimeout(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.RotateToQuarry);
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

    private static final class DragFoundation_RotateToQuarry
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_RotateToQuarry(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("RotateToQuarry");
            ctxt.rotateToQuarry();
            return;
        }

        @Override
        protected void evRotationComplete(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.DriveToQuarry);
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

    private static final class DragFoundation_PrepareDriveToQuarry
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_PrepareDriveToQuarry(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("PrepareDriveToQuarry");
            ctxt.retractArm();
            return;
        }

        @Override
        protected void evArmRetracted(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.DriveToQuarry);
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

    private static final class DragFoundation_DriveToQuarry
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_DriveToQuarry(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("DriveToQuarry");
            ctxt.driveToQuarry();
            return;
        }

        @Override
        protected void evDriveComplete(DragFoundationContext context)
        {

            (context.getState()).exit(context);
            context.setState(DragFoundation.Success);
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

    private static final class DragFoundation_RotateToBlock
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_RotateToBlock(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("RotateToBlock");
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

    private static final class DragFoundation_Success
        extends DragFoundation_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private DragFoundation_Success(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(DragFoundationContext context)
            {
                DragFoundationController ctxt = context.getOwner();

            ctxt.setLogMessage("Success");
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
