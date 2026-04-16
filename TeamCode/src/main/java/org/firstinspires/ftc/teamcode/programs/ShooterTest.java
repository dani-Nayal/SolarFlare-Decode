package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_YAW_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_YAW_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

@TeleOp
public class ShooterTest extends LinearOpMode {
    public double targetYaw = 0;

    @Override
    public void runOpMode(){
        initialize(this,new Inferno(),true,true);
        executor.setCommands(
                turretPitch.command((Components.BotServo servo)->servo.triggeredDynamicTargetCommand(()->gamepad1.right_bumper,()->gamepad1.left_bumper,0.1)),
                Commands.triggeredDynamicCommand(()->gamepad1.right_trigger>0.3,()->gamepad1.left_trigger>0.3,new Commands.InstantCommand(()->targetYaw+=0.12),new Commands.InstantCommand(()->targetYaw-=0.12)),
                Commands.triggeredDynamicCommand(()->gamepad1.right_bumper,()->gamepad1.left_bumper,new Commands.InstantCommand(()->targetYaw+=0.005),new Commands.InstantCommand(()->targetYaw-=0.005)),
                new Commands.RunResettingLoop(new Commands.PressCommand(
                        new Commands.IfThen(()->gamepad1.x,
                                new Commands.SequentialCommand(
                                        new Commands.InstantCommand(()->targetYaw = 180),
                                        new Commands.SleepCommand(0.5),
                                        new Commands.InstantCommand(()->targetYaw = -110)
                                )
                        )
                )),
                new Commands.RunResettingLoop(new Commands.InstantCommand(()->turretYaw.call(servo->servo.setTarget(targetYaw*TURRET_YAW_RATIO+TURRET_YAW_OFFSET))))
        );
        executor.setWriteToTelemetry(()->{
            telemetry.addData("hood",turretPitch.get("turretPitchLeft").getTarget());
            telemetry.addData("yaw target",turretYaw.get("turretYawTop").getTargetMinusOffset());
            telemetry.addData("yaw angle",targetYaw);
        });
        waitForStart();
        Components.activateActuatorControl();
        flywheel.call((Components.BotMotor motor)->motor.switchControl("controlOff"));
        executor.runLoop(this::opModeIsActive);
    }
}
