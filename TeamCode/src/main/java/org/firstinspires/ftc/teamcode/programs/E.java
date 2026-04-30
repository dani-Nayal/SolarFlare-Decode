package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class E extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretYawTop = hardwareMap.get(Servo.class,"turretYawTop");
        Servo turretYawBottom = hardwareMap.get(Servo.class,"turretYawBottom");
        turretYawTop.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        int count = 0;
        boolean aWasPressed = false;
        while (opModeIsActive()){
            if (gamepad1.a && !aWasPressed){
                count+=1;
                aWasPressed=true;
            } else if (!gamepad1.a && aWasPressed) aWasPressed = false;
            turretYawTop.setPosition(count*0.31716867489);
            turretYawBottom.setPosition(count*0.31716867489);
            telemetry.addData("e",turretYawTop.getPosition());
            telemetry.update();
        }
    }
}
