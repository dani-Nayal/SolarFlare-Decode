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
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

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

    public List<ArtifactDescriptor> getRelativeArtifactDescriptors(){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        List<ArtifactDescriptor> artifactDescriptors = new ArrayList<>(); // Output array
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return artifactDescriptors;
        }

        for (LLResultTypes.DetectorResult detectorResult : result.getDetectorResults()) {
            String className = detectorResult.getClassName();
            List<List<Double>> corners = detectorResult.getTargetCorners();

            telemetry.addData("corner 0", corners.get(0));
            telemetry.addData("corner 1", corners.get(1));
            telemetry.addData("corner 2", corners.get(2));
            telemetry.addData("corner 3", corners.get(3));

            double targetX = getBottomCenterXPixels(cameraOrientation, corners);
            double targetY = getBottomCenterYPixels(cameraOrientation, corners);

            double xOffset = targetX - cxNN;
            double yOffset = cyNN - targetY;

            double tx = Math.toDegrees(Math.atan(xOffset / fxNN));
            double ty = Math.toDegrees(Math.atan(yOffset / fyNN));

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("cx", cxNN);
            telemetry.addData("cy", cyNN);
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("xOffset", xOffset);
            telemetry.addData("yOffset", yOffset);

            double verticalAngleDeg = 90 + ty + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

            telemetry.addData("vertical angle", verticalAngleDeg);

            double depth = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z * Math.tan(Math.toRadians(verticalAngleDeg));

            double horizontal = depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES)));

            depth += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
            horizontal += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

            ArtifactDescriptor artifact = new ArtifactDescriptor(horizontal, depth, className);
            artifact.setBottomCenterXPixels(targetX);
            artifact.setBottomCenterYPixels(targetY);
            artifactDescriptors.add(artifact);

        }
        return artifactDescriptors;
    }

    public List<ArtifactDescriptor> getArtifactDescriptors(Pose botPosePedro){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != NN_PIPELINE_INDEX) limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        List<ArtifactDescriptor> artifactDescriptors = new ArrayList<>(); // Output array
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return artifactDescriptors;

        for (LLResultTypes.DetectorResult detectorResult : result.getDetectorResults()) {
            String className = detectorResult.getClassName();

            List<List<Double>> corners = detectorResult.getTargetCorners();

            double targetX = getBottomCenterXPixels(cameraOrientation, corners);
            double targetY = getBottomCenterYPixels(cameraOrientation, corners);

            double xOffset = targetX - cxNN;
            double yOffset = cyNN - targetY;

            double tx = Math.toDegrees(Math.atan(xOffset / fxNN));
            double ty = Math.toDegrees(Math.atan(yOffset / fyNN));

            double verticalAngleDeg = 90 + ty + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

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

            ArtifactDescriptor artifact = new ArtifactDescriptor(artifactX, artifactY, className, detectorResult.getTargetXPixels(), detectorResult.getTargetYPixels(), targetX, targetY, getTopCenterXPixels(cameraOrientation, corners), getTopCenterYPixels(cameraOrientation, corners));

            artifactDescriptors.add(artifact);

        }
        return artifactDescriptors;
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

    public List<ArtifactDescriptor> pedroToStandardPoseArtifacts(List<ArtifactDescriptor> artifacts){
        if (artifacts.isEmpty()) return artifacts;

        List<ArtifactDescriptor> out = new ArrayList<>();
        for (ArtifactDescriptor artifactDescriptor : artifacts){
            double x = artifactDescriptor.getX();
            double y = artifactDescriptor.getY();
            String className = artifactDescriptor.getClassName();

            double xTransformed = 72 - y;
            double yTransformed = x - 72;

            out.add(new ArtifactDescriptor(xTransformed, yTransformed, className));
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

    public Double intakingAngleArtifacts(List<ArtifactDescriptor> artifacts, Pose botPose, int stepDeg) {
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
            double totalPerpDist = 0;
            double totalDist = 0;

            for (ArtifactDescriptor artifact : artifacts) {
                double rx = artifact.getX() - botPose.getX();
                double ry = artifact.getY() - botPose.getY();

                double perpDist = Math.abs(rx * sin - ry * cos);
                double distance = Math.hypot(rx, ry);

                if (perpDist < INTAKING_WIDTH_INCHES / 2) {
                    count++;
                    totalPerpDist += perpDist;
                    totalDist += distance;
                }
            }

            if (count > 0) {
                double avgPerpDist = totalPerpDist / count;
                double avgDist = totalDist / count;

                double normCount = count / (double) artifacts.size();
                double normPerp = avgPerpDist / (INTAKING_WIDTH_INCHES / 2);
                double normDist = avgDist / MAX_RELEVANT_DISTANCE;

                double score = w1 * normCount - w2 * normPerp - w3 * normDist;

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

    public boolean artifactsEvenlySpaced(List<ArtifactDescriptor> artifacts, double distTolerance){
        if (artifacts.size() < 3) return true;

        artifacts.sort(Comparator.comparing(ArtifactDescriptor::getTopCenterXPixels));

        List<Double> distances = new ArrayList<>();

        for (int i = 1; i < artifacts.size(); i++){
            ArtifactDescriptor artifact1 = artifacts.get(i - 1);
            ArtifactDescriptor artifact2 = artifacts.get(i);

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

    public double getAngleToArtifactDegrees(ArtifactDescriptor firstArtifact, ArtifactDescriptor secondArtifact){
        double x1 = firstArtifact.getTopCenterXPixels();
        double y1 = firstArtifact.getTopCenterYPixels();
        double x2 = secondArtifact.getTopCenterXPixels();
        double y2 = secondArtifact.getTopCenterYPixels();

        double deltaY = -(y2 - y1); // Y is flipped in image coordinates
        double deltaX = x2 - x1;

        double angle = Math.atan2(deltaY, deltaX);

        return normalizeHeading360Degrees(angle);
    }

    public double pointToLineDistance(
            double x1, double y1,
            double x2, double y2,
            double px, double py) {

        double dx = x2 - x1;
        double dy = y2 - y1;

        double numerator = Math.abs(dx * (y1 - py) - dy * (x1 - px));
        double denominator = Math.hypot(dx, dy);

        if (denominator < 1e-6) return Double.POSITIVE_INFINITY;

        return numerator / denominator;
    }

    public List<ArtifactDescriptor> getClassifierArtifacts(List<ArtifactDescriptor> artifacts){
        if (artifacts.isEmpty()) return artifacts;

        double lineDistTolerance = 30;
        double distTolerance = 30;

        List<ArtifactDescriptor> bestArtifacts = new ArrayList<>();
        double bestAverageY = Double.POSITIVE_INFINITY;

        for (int i = 0; i < artifacts.size(); i++){
            for (int j = i + 1; j < artifacts.size(); j++) {
                ArtifactDescriptor artifact1 = artifacts.get(i);
                ArtifactDescriptor artifact2 = artifacts.get(j);

                List<ArtifactDescriptor> collinearArtifacts = new ArrayList<>();

                for (ArtifactDescriptor artifact : artifacts) {
                    double dist = pointToLineDistance(artifact1.getTopCenterXPixels(), artifact1.getTopCenterYPixels(), artifact2.getTopCenterXPixels(), artifact2.getTopCenterYPixels(), artifact.getTopCenterXPixels(), artifact.getTopCenterYPixels());

                    if (dist < lineDistTolerance){
                        collinearArtifacts.add(artifact);
                    }
                }

                if (collinearArtifacts.isEmpty()) continue;

                double sumY = 0;

                for (ArtifactDescriptor artifact : collinearArtifacts){
                    sumY += artifact.getCenterYPixels();
                }

                double averageY = sumY / collinearArtifacts.size();

                if (averageY < bestAverageY && artifactsEvenlySpaced(collinearArtifacts, distTolerance)){ // Less than because in camera y increases downward, we want artifacts that are highest in the image
                    bestArtifacts = collinearArtifacts;
                    bestAverageY = averageY;
                }

            }
        }
        return bestArtifacts;
    }

    public List<ArtifactDescriptor> getGroundArtifacts(List<ArtifactDescriptor> artifacts){
        if (artifacts.isEmpty()) return artifacts;

        List<ArtifactDescriptor> classifierArtifacts = getClassifierArtifacts(artifacts);

        artifacts.removeAll(classifierArtifacts);

        return artifacts;

    }

    public double getNumArtifactsInClassifier(List<ArtifactDescriptor> classifierArtifacts){
        return classifierArtifacts.size();
    }

    public List<String> getClassifierArtifactColors(List<ArtifactDescriptor> classifierArtifacts, Inferno.Alliance alliance){
        if (classifierArtifacts.isEmpty() || alliance == null) return new ArrayList<>();

        List<String> colors = new ArrayList<>();

        if (alliance == Inferno.Alliance.BLUE) classifierArtifacts.sort(Comparator.comparing(ArtifactDescriptor::getTopCenterXPixels));
        else if (alliance == Inferno.Alliance.RED) classifierArtifacts.sort(Comparator.comparing(ArtifactDescriptor::getTopCenterXPixels).reversed());

        for (ArtifactDescriptor artifact : classifierArtifacts){
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
}
