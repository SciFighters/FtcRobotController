package org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States;

import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class ManualState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
        owner.setManualMode(true, owner.getHoldPower());
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {

    }
}
