package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

/**
 * The GoToState class represents the state where the arm moves to a target position using proportional control.
 */
public class GoToState extends State<Arm> {
    Arm arm;

    @Override
    public void enter(Arm owner) {
        this.arm = owner;
    }

    /**
     * Executes the proportional control to move the arm to the target position.
     */
    @Override
    public void execute() {
        this.arm.setManualMode(false, 1);
        if (arm.areMotorsBusy()) {
            arm.stateMachine.changeState(arm.holdState);
        }
    }
}
