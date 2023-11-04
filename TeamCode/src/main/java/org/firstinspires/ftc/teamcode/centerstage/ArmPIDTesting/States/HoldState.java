package org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States;

import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class HoldState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
        owner.setTargetPositions(owner.getCurrentHandPos(), owner.getCurrentRailPos());
        owner.setManualMode(false, owner.getHoldPower());
    }

    @Override
    public void execute() {
        owner.setManualMode(false, owner.getHoldPower());
    }

    @Override
    public void exit() {

    }
}
