package org.firstinspires.ftc.teamcode.vision.testing;

import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class TestMotifWithPanels extends OpMode {
    public static boolean isBlueAlliance = true;
    Inferno.Alliance alliance = Inferno.Alliance.BLUE;
    Vision vision;
    Pose3D cameraPose = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    FieldManager panelsField = PanelsField.INSTANCE.getField();
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    double robotWidth = 15;
    double robotLength = 18;
    Pose initPosePedro = new Pose(72, 144 - (robotLength / 2), Math.toRadians(270));
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

        if (isBlueAlliance) alliance = Inferno.Alliance.BLUE;
        else alliance = Inferno.Alliance.RED;

        List<Artifact> classifierArtifacts = vision.getGroundAndClassifierArtifacts(botPose).get(0);

        if (!classifierArtifacts.isEmpty()){

            List<String> colors = vision.getClassifierArtifactColors(classifierArtifacts, alliance);

            Double points = vision.getPatternPoints(colors, Vision.MOTIF.PPG);

            panelsTelemetry.addData("colors", colors);
            telemetry.addData("colors", colors);

            panelsTelemetry.addData("points", points);
            telemetry.addData("points", points);

            for (int i = 0; i < classifierArtifacts.size(); i++){
                if (i == 0){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }

                }
                else if (i == 1){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                }
                else if (i == 2){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }

                }
                else if (i == 3){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }

                }
                else if (i == 4){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }

                }
                else if (i == 5){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }

                }
                else if (i == 6){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }

                }
                else if (i == 7){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                }
                else if (i == 8){
                    if (alliance == Inferno.Alliance.BLUE){
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                    else {
                        panelsField.setStyle(colors.get(0), colors.get(0), 0);
                        panelsField.moveCursor(0, 0);
                        panelsField.circle(2.5);
                    }
                }
            }
        }
        else {
            telemetry.addLine("no classifier artifacts found");
            panelsTelemetry.addLine("no classifier artifacts found");
        }

        panelsField.update();
        panelsTelemetry.update();
        telemetry.update();
    }
}
