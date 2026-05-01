package org.firstinspires.ftc.teamcode.vision.testing;

import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.Artifact;

import java.util.List;

@TeleOp
public class TestArtifactsWithPanels extends OpMode {
    Vision vision;
    Pose3D cameraPose = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    double robotWidth = 15;
    double robotLength = 18;
    Pose initPosePedro = new Pose(72, 144 - (robotLength / 2), Math.toRadians(270));
    FieldManager panelsField = PanelsField.INSTANCE.getField();
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init(){
        vision = new Vision(hardwareMap, telemetry, cameraPose, Vision.CAMERA_ORIENTATION.NORMAL);

        initialize(this, new Inferno(),false,true);
        Pedro.createFollower(initPosePedro);

        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    public void loop() {
        follower.updatePose();

        Pose botPose = follower.getPose();

        vision.drawPoseOnPanels(panelsField, botPose, "blue");

        List<List<Artifact>> artifacts = vision.getArtifacts(botPose);

        telemetry.addData("artifacts size", artifacts.size());

        if (artifacts.size() < 2) return;

        List<Artifact> groundArtifacts = artifacts.get(0);

        List<Artifact> classifierArtifacts = artifacts.get(1);

        if (!groundArtifacts.isEmpty()) {
            Double intakingAngle = vision.intakingAngleArtifacts(groundArtifacts, botPose, 1);
            telemetry.addData("intaking angle", intakingAngle);
            panelsTelemetry.addData("intaking angle", intakingAngle);

            double lineLength = 10;

            double x2 = botPose.getX() + Math.cos(Math.toRadians(intakingAngle)) * lineLength;
            double y2 = botPose.getY() + Math.sin(Math.toRadians(intakingAngle)) * lineLength;

            panelsField.setStyle("red", "red", 1);
            panelsField.moveCursor(botPose.getX(), botPose.getY());
            panelsField.line(x2, y2);

            for (Artifact artifact : groundArtifacts) {
                double x = artifact.x;
                double y = artifact.y;
                String className = artifact.className;
                Artifact.ARTIFACT_TYPE artifactType = artifact.artifactType;

                panelsTelemetry.addData("x", x);
                panelsTelemetry.addData("y", y);
                panelsTelemetry.addData("className", className);
                panelsTelemetry.addData("artifactType", artifactType);

                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("className", className);
                telemetry.addData("artifactType", artifactType);

                panelsField.setStyle(className, className, 0);
                panelsField.moveCursor(x, y);
                panelsField.circle(2.5 );
            }
        }
        if (!classifierArtifacts.isEmpty()){
            panelsTelemetry.addData("count", classifierArtifacts.size());
            telemetry.addData("count", classifierArtifacts.size());

            for (Artifact artifact : classifierArtifacts) {
                double x = artifact.x;
                double y = artifact.y;
                double z = artifact.z;
                String className = artifact.className;
                Artifact.ARTIFACT_TYPE artifactType = artifact.artifactType;
                double zDiff = artifact.zDiff;

                panelsTelemetry.addData("x", x);
                panelsTelemetry.addData("y", y);
                panelsTelemetry.addData("z", z);
                panelsTelemetry.addData("className", className);
                panelsTelemetry.addData("artifactType", artifactType);
                panelsTelemetry.addData("zDiff", zDiff);

                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("z", z);
                telemetry.addData("className", className);
                telemetry.addData("artifactType", artifactType);
                telemetry.addData("zDiff", zDiff);

                panelsField.setStyle(className, className, 0);
                panelsField.moveCursor(x, y);
                panelsField.circle(2.5 + (z / 4));
            }
        }
        panelsField.update();
        panelsTelemetry.update();
        telemetry.update();
    }
}
