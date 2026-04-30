package org.firstinspires.ftc.teamcode.vision;
import com.bylazar.field.FieldManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    public final double INTAKING_WIDTH_INCHES = 10;
    public CAMERA_ORIENTATION cameraOrientation = CAMERA_ORIENTATION.NORMAL;
    public final int NN_PIPELINE_INDEX = 0;
    public final int APRIL_TAGS_PIPELINE_INDEX = 1;
    public final double HORIZONTAL_FOV = 54.4;
    public Pose3D cameraPoseOnRobot = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    public Limelight3A limelight;
    public Telemetry telemetry;
    List<Integer> localizationAprilTags = new ArrayList<>();

    public enum CAMERA_ORIENTATION {
        NORMAL,
        UPSIDE_DOWN,
        CLOCKWISE_90,
        COUNTER_CLOCKWISE_90
    }

    public enum MOTIF {
        GPP,
        PGP,
        PPG
    }

    public Vision(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        localizationAprilTags.add(20);
        localizationAprilTags.add(24);
    }

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, Pose3D cameraPoseOnRobot, CAMERA_ORIENTATION cameraOrientation){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        this.cameraPoseOnRobot = cameraPoseOnRobot;
        this.cameraOrientation = cameraOrientation;

        localizationAprilTags.add(20);
        localizationAprilTags.add(24);
    }

    public List<List<Artifact>> getArtifacts(Pose botPosePedro){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != NN_PIPELINE_INDEX) limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        LLResult result = limelight.getLatestResult();

        List<Artifact> groundArtifacts = new ArrayList<>();
        List<Artifact> classifierArtifacts = new ArrayList<>();

        if (result == null || !result.isValid()){
            List<List<Artifact>> out = new ArrayList<>();

            out.add(groundArtifacts);
            out.add(classifierArtifacts);

            return out;
        }

        for (LLResultTypes.DetectorResult detectorResult : result.getDetectorResults()) {
            Artifact artifact = new Artifact(detectorResult, cameraOrientation, botPosePedro);

            if (artifact.artifactType == Artifact.ARTIFACT_TYPE.GROUND){
                groundArtifacts.add(artifact);
            }
            else if (artifact.artifactType == Artifact.ARTIFACT_TYPE.CLASSIFIER){
                classifierArtifacts.add(artifact);
            }
        }

        if (classifierArtifacts.size() > 9){
            classifierArtifacts.subList(9, classifierArtifacts.size()).clear();
        }

        List<List<Artifact>> out = new ArrayList<>();

        out.add(groundArtifacts);
        out.add(classifierArtifacts);

        return out;
    }

    public Pose getBotPoseMT1(Pose odometryPose){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != APRIL_TAGS_PIPELINE_INDEX) limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        double odometryHeading = normalizeHeading360Degrees(Math.toDegrees(odometryPose.getHeading()));

        Pose out = null;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult aprilTag : aprilTags){
                if (localizationAprilTags.contains(aprilTag.getFiducialId())){
                    Pose aprilTagPose = limelightToPedroPose(result.getBotpose());

                    double aprilTagHeading = Math.toDegrees(aprilTagPose.getHeading());

                    double diff = (aprilTagHeading - odometryHeading + 540) % 360 - 180;
                    double headingDiff = Math.abs(diff);

                    double dist = Math.hypot(aprilTagPose.getX() - odometryPose.getX(), aprilTagPose.getY() - odometryPose.getY());

                    if ((headingDiff < 15) && (dist < 10)){
                        out = aprilTagPose;
                    }
                }
            }
        }
        else return null;

        return out;
    }

    public Pose getBotPoseMT2(Pose odometryPose){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != APRIL_TAGS_PIPELINE_INDEX) limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        double odometryHeading = normalizeHeading360Degrees(Math.toDegrees(odometryPose.getHeading()));

        limelight.updateRobotOrientation(standardToLimelightYaw(odometryHeading));

        Pose out = null;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult aprilTag : aprilTags){
                if (localizationAprilTags.contains(aprilTag.getFiducialId())){
                    Pose aprilTagPose = limelightToPedroPose(result.getBotpose());

                    double aprilTagHeading = Math.toDegrees(aprilTagPose.getHeading());

                    double diff = (aprilTagHeading - odometryHeading + 540) % 360 - 180;
                    double headingDiff = Math.abs(diff);

                    double dist = Math.hypot(aprilTagPose.getX() - odometryPose.getX(), aprilTagPose.getY() - odometryPose.getY());

                    if ((headingDiff < 15) && (dist < 10)){
                        out = aprilTagPose;
                    }
                }
            }
        }
        else return null;

        return out;
    }
    public Pose getBotPoseMT2WithMT1(Pose odometryPose) {
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != APRIL_TAGS_PIPELINE_INDEX) limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        double odometryHeading = normalizeHeading360Degrees(Math.toDegrees(odometryPose.getHeading()));

        Pose botPoseMT1 = getBotPoseMT1(odometryPose);

        if (botPoseMT1 != null) {
            double llYaw = standardToLimelightYaw(Math.toDegrees(botPoseMT1.getHeading()));

            limelight.updateRobotOrientation(llYaw);

            Pose out = null;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult aprilTag : aprilTags){
                    if (localizationAprilTags.contains(aprilTag.getFiducialId())){
                        Pose aprilTagPose = limelightToPedroPose(result.getBotpose());

                        double aprilTagHeading = Math.toDegrees(aprilTagPose.getHeading());

                        double diff = (aprilTagHeading - odometryHeading + 540) % 360 - 180;
                        double headingDiff = Math.abs(diff);

                        double dist = Math.hypot(aprilTagPose.getX() - odometryPose.getX(), aprilTagPose.getY() - odometryPose.getY());

                        if ((headingDiff < 15) && (dist < 10)){
                            out = aprilTagPose;
                        }
                    }
                }
            }
            return out;
        }
        return null;
    }

    public Pose limelightToPedroPose(Pose3D llPose){
        double llX = llPose.getPosition().toUnit(DistanceUnit.INCH).x;
        double llY = llPose.getPosition().toUnit(DistanceUnit.INCH).y;
        double llYaw = llPose.getOrientation().getYaw(AngleUnit.DEGREES);

        double xTransformed = llY + 72;
        double yTransformed = 72 - llX;
        double yawTransformed = limelightToStandardYaw(llYaw);

        yawTransformed = Math.toRadians(yawTransformed);

        return new Pose(xTransformed, yTransformed, yawTransformed);
    }

    public double standardToLimelightYaw(double standardYawDegrees){
        double yawTransformed = standardYawDegrees + 90;

        if (yawTransformed >= 180.0) {
            yawTransformed -= 360.0;
        }
        return yawTransformed;
    }

    public double limelightToStandardYaw(double llYawDegrees){
        double yawTransformed = (llYawDegrees + 270) % 360;

        if (yawTransformed < 0) {
            yawTransformed += 360;
        }
        return yawTransformed;
    }
    public void stopLimelight(){
        limelight.stop();
    }
    public Integer getObeliskID(){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != APRIL_TAGS_PIPELINE_INDEX) limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult aprilTag : aprilTags){
                int id = aprilTag.getFiducialId();
                if (id == 21 || id == 22 || id == 23){
                    return id;
                }
            }
        }

        telemetry.addLine("ID not in sight");
        return null;
    }

    public Double intakingAngleArtifacts(List<Artifact> artifacts, Pose botPose, int stepDeg) {
        if (artifacts.isEmpty()) return null;

        double bestAngle = -1;

        double bestScore = Double.NEGATIVE_INFINITY;

        final double MAX_RELEVANT_DISTANCE = 70;

        double w1 = 10;
        double w2 = 4;
        double w3 = 1;

        double horizontalLeftBound = Math.toDegrees(botPose.getHeading()) + (HORIZONTAL_FOV / 2);
        double horizontalRightBound = Math.toDegrees(botPose.getHeading()) - (HORIZONTAL_FOV / 2);

        for (double angleDeg = horizontalLeftBound; angleDeg >= horizontalRightBound; angleDeg -= stepDeg) {

            double theta = Math.toRadians(angleDeg);
            double cos = Math.cos(theta);
            double sin = Math.sin(theta);

            int count = 0;
            double totalPerpendicularDist = 0;
            double totalDist = 0;

            for (Artifact artifact : artifacts) {
                double rx = artifact.x - botPose.getX();
                double ry = artifact.y - botPose.getY();

                double perpendicularDist = Math.abs(rx * sin - ry * cos);
                double distance = Math.hypot(rx, ry);

                if (perpendicularDist < INTAKING_WIDTH_INCHES / 2) {
                    count++;
                    totalPerpendicularDist += perpendicularDist;
                    totalDist += distance;
                }
            }

            if (count > 0) {
                double avgPerpendicularDist = totalPerpendicularDist / count;
                double avgDist = totalDist / count;

                double normCount = count / (double) artifacts.size();
                double normPerpendicular = avgPerpendicularDist / (INTAKING_WIDTH_INCHES / 2);
                double normDist = avgDist / MAX_RELEVANT_DISTANCE;

                double score = w1 * normCount - w2 * normPerpendicular - w3 * normDist;

                if (score > bestScore) {
                    bestScore = score;
                    bestAngle = theta;
                }
            }
        }

        double bestAngleDeg = Math.toDegrees(bestAngle);
        if (bestAngleDeg < 0) bestAngleDeg += 360;

        return bestAngleDeg;
    }

    public void drawPoseOnPanels(FieldManager panelsField, Pose pose, String color) {
        if (!(pose == null || panelsField == null)) {
            panelsField.setStyle(color, color, 0);
            panelsField.moveCursor(pose.getX(), pose.getY());
            panelsField.circle(3);

            double headingLineLength = 3;
            double x2 = pose.getX() + Math.cos(pose.getHeading()) * headingLineLength;
            double y2 = pose.getY() + Math.sin(pose.getHeading()) * headingLineLength;

            panelsField.setStyle("black", "black", 1);
            panelsField.line(x2, y2);
        }
    }
    public double normalizeHeading360Degrees(double heading) {
        return (heading + 360) % 360;
    }

    public Pose getLoadingZoneIntakingPosition(List<Artifact> artifacts, Pose botPose, double yConstraint){
        if (botPose == null) return null;
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != NN_PIPELINE_INDEX) limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        double bestScore = Double.NEGATIVE_INFINITY;
        double bestValue = 0;

        double botX = botPose.getX();
        double botY = botPose.getY();

        for (double i = INTAKING_WIDTH_INCHES / 2; i < yConstraint; i++) {

            double score = 0;
            int count = 0;
            double totalHorizontalDistance = 0;
            double totalDistanceToBot = 0;

            for (Artifact artifact : artifacts){
                double artifactX = artifact.x;
                double artifactY = artifact.y;

                double horizontalDistance = Math.abs(i - artifactY);

                if (horizontalDistance <= (INTAKING_WIDTH_INCHES / 2)){
                    totalHorizontalDistance += horizontalDistance;
                    double distanceToBot = Math.sqrt(Math.pow(botX - artifactX, 2) + Math.pow(botY - artifactY, 2));
                    totalDistanceToBot += distanceToBot;
                    count++;
                }
            }

            if (count == 0) continue;

            final double MAX_RELEVANT_DISTANCE = 35;

            double w1 = 10;
            double w2 = 4;
            double w3 = 1;

            double averageHorizontalDistance = totalHorizontalDistance / count;
            double averageDistanceToBot = totalDistanceToBot / count;

            double normPerpendicular = averageHorizontalDistance / (INTAKING_WIDTH_INCHES / 2);
            double normDist = averageDistanceToBot / MAX_RELEVANT_DISTANCE;

            score += w1 * count - w2 * normPerpendicular - w3 * normDist;

            if (count >= 3) score += 100;

            if (score > bestScore){
                bestScore = score;
                bestValue = i;
            }
        }
        return new Pose(botX, bestValue);
    }
}