package org.firstinspires.ftc.teamcode.programs;

import static org.apache.commons.math3.util.FastMath.atan2;
import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.gamepad1;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.telemetry;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_YAW_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_YAW_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.YAW_FIGHT;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.findMotif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.hoodDesired;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.readBallStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transfer;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.useVelFeedforward;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.*;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

import static org.apache.commons.math3.util.FastMath.tan;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.yawDesired;

public class FarPrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.9;
    public static final double SHOT_TIME = 0.7;
    public static final double PRE_SHOT_TIME = 0;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.0;
    public static final double stopIntakeT = 0.17;
    public static final double slowDownAmount = 1.0;
    public static final double loadingSlowDownT = 0.8;
    public static final double loadingSlowDownAmount = 0.7;
    public static Double angle;
    public static Command shoot = new SequentialCommand(new SleepCommand(PRE_SHOT_TIME),
            setState(Inferno.RobotState.SHOOTING),
            new SleepCommand(SHOT_TIME),
            new ConditionalCommand(
                    new IfThen(
                        ()->{
                            readBallStorage();
                            int ballsPresent = 0;

                            for (Inferno.Color color : ballStorage){
                                if (!Objects.isNull(color)) ballsPresent+=1;
                            }
                            return ballsPresent!=0;
                        },
                        new SequentialCommand(
                                new InstantCommand(transfer::reset),
                                new SleepCommand(SHOT_TIME)
                        )
                    )
            ),
            setState(Inferno.RobotState.INTAKE_FRONT));
    public static final HashMap<String, Pose> poses = new HashMap<>();
    public static double visionHeading;
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return Math.PI - input;}
    static {
        poses.put("start",new Pose(60,8.25, Math.toRadians(90)));
        poses.put("preloadShoot",new Pose(60,20.9,Math.toRadians(130)));
        poses.put("firstSpikeLead",new Pose(42.6,30.2,Math.toRadians(130)));
        poses.put("firstSpikeCtrl",new Pose(37.4,33.6,Math.toRadians(180)));
        poses.put("firstSpike",new Pose(17.3,33.0,Math.toRadians(180)));
        poses.put("firstShoot",new Pose(49.9, 11.8,Math.toRadians(180)));
        poses.put("loadingZone",new Pose(11.9, 9.6,Math.toRadians(185)));
        poses.put("shoot",new Pose(54.7, 17.6,Math.toRadians(170)));
        poses.put("park",new Pose(40.7,17.6,Math.toRadians(180)));
    }
    public static Pose getPose(String input){if (alliance==Inferno.Alliance.BLUE) return poses.get(input); else return mirrorPose(Objects.requireNonNull(poses.get(input)));}
    public static double getHeading(String input){if (alliance==Inferno.Alliance.BLUE) return Objects.requireNonNull(poses.get(input)).getHeading(); else return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
    public static double wallX = 0;
    public static double scan1X;
    public static double scan2X;
    public static double wallXOffset = 14;
    public static int flag = 0;
    public static Command preloadShoot = new SequentialCommand(
            new SleepCommand(INITIAL_WAIT),
            new PedroCommand(b->
                b.addPath(new BezierLine(getPose("start"),getPose("preloadShoot")))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0,0.3,HeadingInterpolator.constant(getHeading("start"))),
                                HeadingInterpolator.PiecewiseNode.linear(0.3,0.65, getHeading("start"), getHeading("preloadShoot")),
                                new HeadingInterpolator.PiecewiseNode(0.65,1.0,HeadingInterpolator.constant(getHeading("preloadShoot")))
                        )),true),
            shoot
    );
    public static Command firstSpike = new SequentialCommand(
            new PedroCommand(
                    b->b.addPath(new BezierLine(getPose("preloadShoot"), getPose("firstSpikeLead")))
                            .setTangentHeadingInterpolation()
                        .addPath(new BezierCurve(getPose("firstSpikeLead"), getPose("firstSpikeCtrl"), getPose("firstSpike")))
                            .setTangentHeadingInterpolation()
                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                        .addPath(new BezierLine(getPose("firstSpike"), getPose("firstShoot")))
                            .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0.0,0.4,HeadingInterpolator.tangent.reverse()),
                                    HeadingInterpolator.PiecewiseNode.linear(0.4,0.7, getHeading("firstSpike"), getHeading("firstShoot")),
                                    new HeadingInterpolator.PiecewiseNode(0.7,1.0,HeadingInterpolator.constant(getHeading("firstShoot")))
                            ))
                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),
                    true
            ), shoot
    );
    public static Command loadingZone = new SequentialCommand(
            new PedroCommand(
                    b->b.addPath(new BezierLine(getPose("firstShoot"), getPose("loadingZone")))
                            .setConstantHeadingInterpolation(getHeading("loadingZone"))
                            .addParametricCallback(loadingSlowDownT,()->follower.setMaxPower(loadingSlowDownAmount)), true),
            new SleepCommand(0.15),
            new PedroCommand(
                    b->b.addPath(new BezierLine(getPose("loadingZone"), getPose("shoot")))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(0.0,0.1,HeadingInterpolator.constant(getHeading("loadingZone"))),
                            new HeadingInterpolator.PiecewiseNode(0.1,0.4,HeadingInterpolator.tangent.reverse()),
                            HeadingInterpolator.PiecewiseNode.linear(0.4,0.7, Math.atan2(getPose("loadingZone").getY() - getPose("shoot").getY(), getPose("loadingZone").getX() - getPose("shoot").getX()), getHeading("shoot")),
                            new HeadingInterpolator.PiecewiseNode(0.7,1.0,HeadingInterpolator.constant(getHeading("shoot")))
                        ))
                        .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                        .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()), true),
            shoot
    );
    public static Pose getClusterPose(double x, double xOffset){
        Pose pos = follower.getPose();
        Double angle = vision.intakingAngleArtifacts(vision.getGroundAndClassifierArtifacts(pos).get(0), pos,1);
        FarPrimeSigmaConstants.angle = angle;
        if (Objects.isNull(angle)) angle = visionHeading;
        else angle = Math.toRadians(angle);
        double y = Math.max(7.5,tan(angle)*(x-pos.getX()) + pos.getY());
        return new Pose(x+xOffset,y);
    }
    public static class VisionIntake extends CompoundCommand{
        public static double firstPathStartTime;
        public static boolean failsafe = false;
        public VisionIntake(){
            setGroup(
                    new SequentialCommand(
                            new InstantCommand(()->{firstPathStartTime=timer.time(); failsafe = false;}),
                            new ConditionalCommand(
                                    new IfThen(()->!failsafe, new PedroCommand(
                                            b->b.addPath(new BezierLine(follower::getPose,getClusterPose(scan1X,0)))
                                                            .setTangentHeadingInterpolation()
                                                        .addPath(new BezierLine(follower::getPose,()->getClusterPose(scan2X,0)))
                                                            .setTangentHeadingInterpolation()
                                                        .addPath(new BezierLine(follower::getPose,()->getClusterPose(wallX,wallXOffset)))
                                                            .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                                                    new HeadingInterpolator.PiecewiseNode(0.0,0.7,HeadingInterpolator.tangent),
                                                                    new HeadingInterpolator.PiecewiseNode(0.7,1.0,HeadingInterpolator.constant(visionHeading))
                                                            ))
                                                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                                                            .addCallback(()->(follower.getChainIndex()==2&&timer.time()-firstPathStartTime>3),()->failsafe=true)
                                                        .addPath(new BezierLine(follower::getPose, getPose("shoot")))
                                                            .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                                                    new HeadingInterpolator.PiecewiseNode(0.0,0.1,HeadingInterpolator.constant(visionHeading)),
                                                                    new HeadingInterpolator.PiecewiseNode(0.1,0.4,HeadingInterpolator.tangent.reverse()),
                                                                    new HeadingInterpolator.PiecewiseNode(0.4,0.7,HeadingInterpolator.linearFromPoint(()->follower.getHeading(), ()->getHeading("shoot"),0.7)),
                                                                    new HeadingInterpolator.PiecewiseNode(0.7,1.0,HeadingInterpolator.constant(getHeading("shoot")))
                                                            ))
                                                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),
                                            true
                                    )),
                                    new IfThen(()->failsafe,new PedroCommand(
                                            b->b.addPath(new BezierLine(follower::getPose, getPose("shoot")))
                                                    .setConstantHeadingInterpolation(getHeading("shoot"))
                                                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),true
                                    ))
                            ),
                            shoot
                    )
            );
        }
    }
    public static Command visionIntake = new VisionIntake();
    public static Command park = new ParallelCommand(
            setState(null),
            new PedroCommand(
                    b->b.addPath(new BezierLine(getPose("shoot"),getPose("park")))
                            .setConstantHeadingInterpolation(getHeading("park")),true
            )
    );
    public static void runOpMode(Inferno.Alliance alliance, LinearOpMode opMode){
        initialize(opMode,new Inferno(),true,true);
        Inferno.useTurretSOTM = false;
        useVelFeedforward = false;
        findMotif.reset();
        turretOffsetFromAuto = 0;
        Inferno.motifDetected = false;
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flag = 0;
        Inferno.alliance = alliance;
        if (alliance == Inferno.Alliance.BLUE) {wallX = 2.5; wallXOffset = 14; visionHeading = Math.toRadians(180);} else {wallX = 141.5; wallXOffset = -14; visionHeading = Math.toRadians(0);}
        scan1X = getPose("shoot").getX()*2.0/3.0+ wallX /3.0;
        scan2X = getPose("shoot").getX()/3.0+wallX*2.0/3.0;

        gamePhase = Inferno.GamePhase.AUTO;
        Pedro.createFollower(getPose("start"));
        executor.setWriteToTelemetry(()->{
            telemetry.addData("offset",turretYaw.get("turretYawTop").getOffset());
            telemetry.addData("Target Flywheel Velocity",0);
            telemetry.addData("Flywheel Velocity",0);
            telemetry.addData("botX",follower.getPose().getX());
            telemetry.addData("botY",follower.getPose().getY());
            telemetry.addData("botHeading",follower.getPose().getHeading());
            telemetry.addLine("");
            telemetry.addLine("Please, Speed, we need this.");
        });
        double targetX = (alliance == Inferno.Alliance.RED) ? 141.5 : 2.5;
        executor.setCommands(
                new SequentialCommand(new SleepCommand(1.0),new InstantCommand(Inferno::initTurretYawPosition)),
                turretYaw.command(servo->servo.instantSetTargetCommand(0*TURRET_YAW_RATIO+TURRET_YAW_OFFSET)),
                turretYaw.command(servo->servo.triggeredDynamicOffsetCommand(()->gamepad1.left_trigger>0.2,()->gamepad1.right_trigger>0.2,0.05))
        );
        turretYaw.call(servo->servo.switchControl("setPos"));
        executor.runLoop(opMode::opModeInInit);
        turretOffsetFromAuto = turretYaw.get("turretYawTop").getOffset() + YAW_FIGHT;
        Components.activateActuatorControl();
        executor.setWriteToTelemetry(()->{
            telemetry.addData("Motif",Arrays.asList(motif));
            telemetry.addLine("");
            telemetry.addData("Target Flywheel Velocity", targetFlywheelVelocity);
            telemetry.addData("Flywheel Velocity", flywheel.get("flywheelLeft").getVelocity());
            telemetry.addData("Flywheel Error", targetFlywheelVelocity - flywheel.get("flywheelLeft").getVelocity());
            telemetry.addLine("");
            Components.telemetry.addData("Yaw Pos",turretYaw.get("turretYawTop").getCurrentPosition());
            Components.telemetry.addData("Yaw Target",turretYaw.get("turretYawTop").getTarget());
            Components.telemetry.addData("Yaw Angle",yawDesired);
            Components.telemetry.addData("Yaw Error", turretYaw.get("turretYawTop").getTarget() - turretYaw.get("turretYawTop").getCurrentPosition());
            telemetry.addLine("");
            telemetry.addData("Hood Angle",(turretPitch.get("turretPitchLeft").getTarget()-TURRET_PITCH_OFFSET)/TURRET_PITCH_RATIO);
            telemetry.addData("Hood Desired",hoodDesired);
            telemetry.addLine("");
            telemetry.addData("Ball Storage", Arrays.asList(ballStorage));
            telemetry.addLine("");
            telemetry.addData("Robot State",robotState);
            telemetry.addLine("");
            telemetry.addData("Shot Type",shotType);
            telemetry.addLine("");
            telemetry.addData("Classifier Count",classifierBallCount);
            telemetry.addData("Current Shot Height",currentBallPath);
            telemetry.addLine("");
            telemetry.addData("PoseX",follower.getPose().getX());
            telemetry.addData("PoseY",follower.getPose().getY());
            telemetry.addData("PoseHeading",Math.toDegrees(follower.getHeading()));
            telemetry.addLine("");
            telemetry.addData("Flywheel Left Power",flywheel.get("flywheelLeft").getPower());
            telemetry.addData("Flywheel Right Power",flywheel.get("flywheelRight").getPower());
        });
        Command mainPath = new SequentialCommand(
                preloadShoot,
                new InstantCommand(()->flag=1),
                firstSpike,
                new InstantCommand(()->flag=2),
                loadingZone,
                new InstantCommand(()->flag=3),
                visionIntake,
                visionIntake,
                visionIntake,
                visionIntake,
                visionIntake
        );
        executor.setCommands(
                new SequentialCommand(new ParallelCommand(findMotif, new SequentialCommand(new SleepCommand(5),new InstantCommand(findMotif::stop))), new InstantCommand(()->vision.stopLimelight())),
                new SequentialCommand(
                        new ParallelCommand(
                                mainPath,
                                new SequentialCommand(
                                        new SleepUntilTrue(mainPath::isFinished,29.3),
                                        new InstantCommand(mainPath::stop)
                                )
                        ),
                        park
                ),
                Pedro.updateCommand(),
                loopFSM,
                new RunResettingLoop(
                    new InstantCommand(
                            ()->{if (flag==0) targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("preloadShoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));
                                else if (flag==1) targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("firstShoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));
                                else targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("shoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));}
                    )
                )
        );
        executor.runLoop(opMode::opModeIsActive);
    }
}
