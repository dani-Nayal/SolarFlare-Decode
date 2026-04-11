package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_YAW_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_YAW_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setTargetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.sideRollers;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transferGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
@TeleOp
public class AlsoShooterTest extends LinearOpMode {
    public double velocityOffset = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(this, new Inferno(), false,false);
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pedro.createFollower(new Pose(72,72,0));
        waitForStart();
        turretPitch.call(servo->servo.switchControl("setPos"));
        flywheel.call(motor->motor.switchControl("VelocityPIDF"));
        turretYaw.call(servo->servo.switchControl("setPos"));
        frontIntake.setPower(1.0);
        backIntake.setPower(1.0);
        sideRollers.call(servo->servo.setPower(1.0));
        frontIntakeGate.setPosition(110.7);
        backIntakeGate.setPosition(133);
        transferGate.setPosition(148.5);
        executor.setCommands(turretPitch.command(servo->servo.triggeredDynamicTargetCommand(()->gamepad1.right_bumper,()->gamepad1.left_bumper,0.1)),
                Commands.triggeredDynamicCommand(()->gamepad1.dpad_left,()->gamepad1.dpad_right,new Commands.InstantCommand(()->velocityOffset+=2),new Commands.InstantCommand(()->velocityOffset+=2)),
                new Commands.ContinuousCommand(
                        ()->{
                            setTargetPoint();
                            Pose pos = follower.getPose();
                            targetFlywheelVelocity = Inferno.VelRegression.regressFormula(Math.sqrt((targetPoint[0]-pos.getX())*(targetPoint[0]-pos.getX()) + (targetPoint[1]-pos.getY())*(targetPoint[1]-pos.getY()))) + velocityOffset;
                            turretYaw.call(servo->servo.setTarget((Math.toDegrees(Math.atan2(targetPoint[1] - pos.getY(),targetPoint[0] - pos.getX())) - follower.getHeading())*TURRET_YAW_RATIO+TURRET_YAW_OFFSET));
                        }
                ),
        Pedro.updatePoseCommand());
        executor.setWriteToTelemetry(
                ()->{
                    Components.telemetry.addData("Hood Target",turretPitch.get("turretPitchLeft").getTarget());
                    Components.telemetry.addData("Flywheel Error",targetFlywheelVelocity-flywheel.get("flywheelLeft").getVelocity());
                    Components.telemetry.addData("Target Velocity",targetFlywheelVelocity);
                    Components.telemetry.addData("Velocity Offset",velocityOffset);
                }
        );
        executor.runLoop(this::opModeIsActive);
    }
}
