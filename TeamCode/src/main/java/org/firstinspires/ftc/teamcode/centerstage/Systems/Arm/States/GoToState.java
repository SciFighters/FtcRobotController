package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

/**
 * The GoToState class represents the state where the arm moves to a target position using proportional control.
 */
public class GoToState extends State<Arm> {
    Arm arm;
    private static final double PROPORTIONAL_GAIN = 0.01; // Adjust this gain based on your requirements

    @Override
    public void enter(Arm owner) {
        this.arm = owner;
    }

    /**
     * Executes the proportional control to move the arm to the target position.
     */
    @Override
    public void execute() {
//        int currentPos = arm.getPos();
//        int targetPos = arm.getTargetPos();
//        int error = targetPos - currentPos;
//
//        // Use proportional control to adjust power based on position error
//        double power = PROPORTIONAL_GAIN * error;
        arm.lift1.setPower(0.1);
        arm.telemetry.addData("GOTO RUNNING", "");
//
//        if (Math.abs(error) < arm.getDeadZone()) {
//            // If close to the target position, transition to hold state
//            arm.stateMachine.changeState(arm.holdState);
//        }
    }
}
