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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class Vision {
    public final double INTAKING_WIDTH_INCHES = 10;
    public CAMERA_ORIENTATION cameraOrientation = CAMERA_ORIENTATION.NORMAL;
    public final double fx = 1218.145;
    public final double fy = 1219.418;
    public final double cx = 621.829;
    public final double cy = 500.362;
    public final double fxNN = fx / 4;
    public final double fyNN = fy / 4;
    public final double cxNN = cx / 4;
    public final double cyNN = cy / 4;
    public final int NN_PIPELINE_INDEX = 0;
    public final int APRIL_TAGS_PIPELINE_INDEX = 1;
    public final int CLASSIFIER_RED_PIPELINE_INDEX = 3;
    public final int CLASSIFIER_BLUE_PIPELINE_INDEX = 4;
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

    public List<Artifact> getArtifacts(Pose botPosePedro){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != NN_PIPELINE_INDEX) limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        List<Artifact> artifacts = new ArrayList<>();
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return artifacts;

        for (LLResultTypes.DetectorResult detectorResult : result.getDetectorResults()) {
            String className = detectorResult.getClassName();

            List<List<Double>> corners = detectorResult.getTargetCorners();

            double targetX = getBottomCenterXPixels(cameraOrientation, corners);
            double targetY = getBottomCenterYPixels(cameraOrientation, corners);

            double xOffset = targetX - cxNN;
            double yOffset = cyNN - targetY;

            double tx = Math.toDegrees(Math.atan(xOffset / fxNN));
            double ty = Math.toDegrees(Math.atan(yOffset / fyNN));

            double verticalAngleDeg = ty + 90 + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

            double depth = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z * Math.tan(Math.toRadians(verticalAngleDeg));

            double horizontal = depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES)));

            depth += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
            horizontal += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

            double x = botPosePedro.getX();
            double y = botPosePedro.getY();
            double theta = botPosePedro.getHeading();

            double cos = Math.cos(theta);
            double sin = Math.sin(theta);

            double artifactX = x + depth * cos - (-horizontal) * sin;

            double artifactY = y + depth * sin + (-horizontal) * cos;

            double targetXTopCenter = getTopCenterXPixels(cameraOrientation, corners);
            double targetYTopCenter = getTopCenterYPixels(cameraOrientation, corners);

            double xTopOffset = targetXTopCenter - cxNN;
            double yTopOffset = cyNN - targetYTopCenter;

            double txTopCenter = Math.toDegrees(Math.atan(xTopOffset / fxNN));
            double tyTopCenter = Math.toDegrees(Math.atan(yTopOffset / fyNN));

            Artifact artifact = new Artifact(artifactX, artifactY, className, detectorResult.getTargetXPixels(), detectorResult.getTargetYPixels(), targetX, targetY, getTopCenterXPixels(cameraOrientation, corners), getTopCenterYPixels(cameraOrientation, corners), txTopCenter, tyTopCenter);

            artifacts.add(artifact);

        }
        return artifacts;
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

    public Pose3D pedroToLimelightPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double yaw = Math.toDegrees(pedroPose.getHeading());

        double llX = 72 - y;
        double llY = x - 72;
        double llYaw = standardToLimelightYaw(yaw);

        return new Pose3D(new Position(DistanceUnit.INCH, llX, llY, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, llYaw, 0, 0, 0));
    }

    public Pose2D pedroToStandardPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double yaw = pedroPose.getHeading();

        double xTransformed = 72 - y;
        double yTransformed = x - 72;
        double yawTransformed = Math.toDegrees(yaw);

        return new Pose2D(DistanceUnit.INCH, xTransformed, yTransformed, AngleUnit.DEGREES, yawTransformed);
    }

    public Pose standardToPedroPose(Pose2D standardPose){
        double x = standardPose.getX(DistanceUnit.INCH);
        double y = standardPose.getY(DistanceUnit.INCH);
        double yaw = standardPose.getHeading(AngleUnit.DEGREES);

        double xTransformed = y + 72;
        double yTransformed = 72 - x;
        double yawTransformed = Math.toRadians(yaw);

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

    public List<Artifact> pedroToStandardPoseArtifacts(List<Artifact> artifacts){
        if (artifacts.isEmpty()) return artifacts;

        List<Artifact> out = new ArrayList<>();
        for (Artifact artifact : artifacts){
            double x = artifact.getX();
            double y = artifact.getY();
            String className = artifact.getClassName();

            double xTransformed = 72 - y;
            double yTransformed = x - 72;

            out.add(new Artifact(xTransformed, yTransformed, className));
        }
        return out;
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
                double rx = artifact.getX() - botPose.getX();
                double ry = artifact.getY() - botPose.getY();

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

    public Double getBottomCenterXPixels(CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetXPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == CAMERA_ORIENTATION.NORMAL){
            targetXPixels = (corner2.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetXPixels = (corner0.get(0) + corner1.get(0)) / 2;
        }
        else if (orientation == CAMERA_ORIENTATION.CLOCKWISE_90){
            targetXPixels = (corner1.get(0) + corner2.get(0)) / 2;
        }
        else if (orientation == CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetXPixels = (corner3.get(0) + corner0.get(0)) / 2;
        }

        return targetXPixels;
    }

    public Double getBottomCenterYPixels(CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetYPixels;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == CAMERA_ORIENTATION.NORMAL){
            targetYPixels = corner3.get(1);
        }
        else if (orientation == CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetYPixels = corner1.get(1);
        }
        else if (orientation == CAMERA_ORIENTATION.CLOCKWISE_90){
            targetYPixels = corner2.get(1);
        }
        else if (orientation == CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetYPixels = corner0.get(1);
        }
        else return null;

        return targetYPixels;
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

    public Double getTopCenterXPixels(CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetXPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == CAMERA_ORIENTATION.NORMAL){
            targetXPixels = (corner1.get(0) + corner0.get(0)) / 2;
        }
        else if (orientation == CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetXPixels = (corner2.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == CAMERA_ORIENTATION.CLOCKWISE_90){
            targetXPixels = (corner0.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetXPixels = (corner2.get(0) + corner1.get(0)) / 2;
        }

        return targetXPixels;
    }

    public Double getTopCenterYPixels(CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetYPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == CAMERA_ORIENTATION.NORMAL){
            targetYPixels = corner1.get(1);
        }
        else if (orientation == CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetYPixels = corner3.get(1);
        }
        else if (orientation == CAMERA_ORIENTATION.CLOCKWISE_90){
            targetYPixels = corner0.get(1);
        }
        else if (orientation == CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetYPixels = corner2.get(1);
        }

        return targetYPixels;
    }

    public boolean artifactsEvenlySpaced(List<Artifact> artifacts, double distTolerance){
        if (artifacts.size() < 3) return true;

        artifacts.sort(Comparator.comparing(Artifact::getTopCenterXPixels));

        List<Double> distances = new ArrayList<>();

        for (int i = 1; i < artifacts.size(); i++){
            Artifact artifact1 = artifacts.get(i - 1);
            Artifact artifact2 = artifacts.get(i);

            double distance = Math.sqrt(Math.pow(artifact1.getCenterXPixels() - artifact2.getCenterXPixels(), 2) + Math.pow(artifact1.getCenterYPixels() - artifact2.getCenterYPixels(), 2));
            distances.add(distance);
        }

        for (int i = 1; i < distances.size(); i++){
            if (Math.abs(distances.get(i) - distances.get(i - 1)) > distTolerance){
                return false;
            }
        }
        return true;
    }

    public double getNumArtifactsInClassifier(List<Artifact> classifierArtifacts){
        return classifierArtifacts.size();
    }

    public List<String> getClassifierArtifactColors(List<Artifact> classifierArtifacts, Inferno.Alliance alliance){
        if (classifierArtifacts.isEmpty() || alliance == null) return new ArrayList<>();

        List<String> colors = new ArrayList<>();

        if (alliance == Inferno.Alliance.BLUE) classifierArtifacts.sort(Comparator.comparing(Artifact::getTopCenterXPixels));
        else if (alliance == Inferno.Alliance.RED) classifierArtifacts.sort(Comparator.comparing(Artifact::getTopCenterXPixels).reversed());

        for (Artifact artifact : classifierArtifacts){
            colors.add(artifact.getClassName());
        }

        return colors;
    }

    public Double getPatternPoints(List<String> colors, MOTIF motif){
        if (colors.isEmpty() || motif == null) return null;

        double points = 0;
        String[] correctColors;

        if (motif == MOTIF.GPP) correctColors = new String[]{"green", "purple", "purple", "green", "purple", "purple", "green", "purple", "purple"};
        else if (motif == MOTIF.PGP) correctColors = new String[]{"purple", "green", "purple", "purple", "green", "purple", "purple", "green", "purple"};
        else correctColors = new String[]{"purple", "purple", "green", "purple", "purple", "green", "purple", "purple", "green"};

        int len = Math.min(colors.size(), correctColors.length);
        for (int i = 0; i < len; i++){
            if (colors.get(i).equals(correctColors[i])){
                points += 2;
            }
        }

        return points;
    }

    public List<List<Artifact>> getGroundAndClassifierArtifacts(Pose botPose){
        if (botPose == null) return null;
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != NN_PIPELINE_INDEX) limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        List<Artifact> artifacts = getArtifacts(botPose);

        List<Artifact> classifierArtifacts = new ArrayList<>();

        for (Artifact artifact : artifacts){
            double ty = artifact.getTyTopCenter();

            if (ty > 0){
                classifierArtifacts.add(artifact);
            }
        }

        artifacts.removeAll(classifierArtifacts);

        List<List<Artifact>> output = new ArrayList<>();

        if (classifierArtifacts.size() > 9){
            classifierArtifacts.subList(9, classifierArtifacts.size()).clear();
        }

        output.add(artifacts);
        output.add(classifierArtifacts);

        return output;
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
                double artifactX = artifact.getX();
                double artifactY = artifact.getY();

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


