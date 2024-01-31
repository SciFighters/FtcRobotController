package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class IdleState extends State<Arm> {
    @Override
    public void enter(Arm owner) {
        owner.setTargetPositions(owner.pos());
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {

    }
}
