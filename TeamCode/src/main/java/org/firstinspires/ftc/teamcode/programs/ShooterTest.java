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

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

@TeleOp
public class ShooterTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        initialize(this,new Inferno(),true,true);
        turretYaw.call(servo->servo.setTarget(50));
        executor.setCommands(
                turretPitch.command((Components.BotServo servo)->servo.triggeredDynamicTargetCommand(()->gamepad1.right_bumper,()->gamepad1.left_bumper,0.1)),
                turretYaw.command(servo->servo.triggeredDynamicTargetCommand(()->gamepad1.right_trigger>0.3,()->gamepad1.left_trigger>0.3,0.12))
        );
        executor.setWriteToTelemetry(()->{
            telemetry.addData("hood",turretPitch.get("turretPitchLeft").getTarget());
            telemetry.addData("yaw",turretYaw.get("turretYawTop").getTarget());
        });
        waitForStart();
        Components.activateActuatorControl();
        flywheel.call((Components.BotMotor motor)->motor.switchControl("controlOff"));
        executor.runLoop(this::opModeIsActive);
    }
}
