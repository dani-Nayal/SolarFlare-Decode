package org.firstinspires.ftc.teamcode.programs;

import static org.apache.commons.math3.util.FastMath.atan2;
import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.gamepad1;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ShotType.AIRSORT;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ShotType.NORMAL;
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
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setShotType;
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
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Commands.Command;
import org.firstinspires.ftc.teamcode.base.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.base.Commands.IfThen;
import org.firstinspires.ftc.teamcode.base.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.base.Commands.ParallelCommand;
import org.firstinspires.ftc.teamcode.base.Commands.PressCommand;
import org.firstinspires.ftc.teamcode.base.Commands.RunResettingLoop;
import org.firstinspires.ftc.teamcode.base.Commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SleepCommand;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroCommand;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class ClosePrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.001;
    public static final double PRE_SHOT_TIME = 0;
    public static final double SHOT_TIME = 0.3;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.05;
    public static final double stopIntakeT = 0.17;
    public static final double slowDownAmount = 1.0;
    public static double gateIntakeTimeout = 1.7;
    public static final double secondShootSlowT = 0.75;
    public static final double fourthShootSlowT = 0.75;
    public static final double shootSlowT = 0.8;
    public static final double shootSlowAmount = 1.0;
    public static boolean airSorting = false;
    public static Command shoot = new SequentialCommand(
            new SleepCommand(PRE_SHOT_TIME),
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
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return Math.PI - input;}
    public static final double[] SHOOT = new double[]{59.85,79.42,Math.toRadians(180)};
    public static final double[] FINAL_SHOOT = new double[]{65,105,Math.toRadians(180)};
    public static final double[] MOTIF_SHOOT = new double[]{65.5,72.5,Math.toRadians(163)};
    static {
        poses.put("start",new Pose(20.29, 121.28, Math.toRadians(144)));
        poses.put("preloadShoot",new Pose(SHOOT[0],SHOOT[1],Math.toRadians(144)));
        poses.put("preloadMotifShoot",new Pose(SHOOT[0],SHOOT[1],Math.toRadians(100)));
        poses.put("shoot",new Pose(SHOOT[0],SHOOT[1],SHOOT[2]));
        poses.put("secondSpikeCtrl",new Pose(41.07, 58.06));
        poses.put("secondSpike",new Pose(18.829, 58.805));
        poses.put("secondSpikeGateOpen",new Pose(16.80, 60.54,Math.toRadians(180)));
        poses.put("gateOpen",new Pose(16.3, 57.5,Math.toRadians(160)));
        poses.put("firstSpike",new Pose(22.773, 81,Math.toRadians(180)));
        poses.put("thirdSpikeLead",new Pose(42.90, 45.23));
        poses.put("thirdSpikeCtrl",new Pose(35.36, 35.64));
        poses.put("thirdSpike",new Pose(18.66, 35.69,Math.toRadians(180)));
        poses.put("park",new Pose(45,79,Math.toRadians(180)));
    }
    public static Pose getPose(String input){if (alliance==Inferno.Alliance.BLUE) return poses.get(input); else return mirrorPose(Objects.requireNonNull(poses.get(input)));}
    public static double getHeading(String input){if (alliance==Inferno.Alliance.BLUE) return Objects.requireNonNull(poses.get(input)).getHeading(); else return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
    public static Command preloadShoot = new SequentialCommand(new SleepCommand(INITIAL_WAIT),
            new PedroCommand(b->
                    b.addPath(new BezierLine(follower::getPose,getPose("preloadShoot")))
                    .setHeadingInterpolation(HeadingInterpolator.linear(getHeading("start"),getHeading("preloadShoot"),0.5)),
    true),
    shoot);
    public static Command preloadMotifShoot = new SequentialCommand(new SleepCommand(INITIAL_WAIT),
            new PedroCommand(b->
                    b.addPath(new BezierLine(follower::getPose,getPose("preloadMotifShoot")))
                            .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0.0,0.4,HeadingInterpolator.constant(getHeading("start"))),
                                    HeadingInterpolator.PiecewiseNode.linear(0.4,0.7,getHeading("start"),getHeading("preloadMotifShoot")),
                                    new HeadingInterpolator.PiecewiseNode(0.7,1.0,HeadingInterpolator.constant(getHeading("preloadMotifShoot")))
                            )),
                    true),
            shoot);
    public static Command secondSpike = new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                    new BezierCurve(
                            follower::getPose,
                            getPose("secondSpikeCtrl"),
                            getPose("secondSpike")
                    ))
                .setConstantHeadingInterpolation(getHeading("shoot"))
                .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
            .addPath(
                    new BezierLine(
                            follower::getPose,
                            getPose("shoot")
                    )
            ).setConstantHeadingInterpolation(getHeading("shoot"))
                .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                .addParametricCallback(secondShootSlowT,()->follower.setMaxPower(shootSlowAmount))
                .addParametricCallback(0.93,()->follower.setMaxPower(1.0)), true), shoot);
    public static Command secondSpikeGateOpen = new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                            new BezierCurve(
                                    follower::getPose,
                                    getPose("secondSpikeCtrl"),
                                    getPose("secondSpikeGateOpen")
                            ))
                    .setConstantHeadingInterpolation(getHeading("shoot"))
                    .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("shoot")
                            )
                    ).setConstantHeadingInterpolation(getHeading("shoot"))
                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                    .addParametricCallback(secondShootSlowT,()->follower.setMaxPower(shootSlowAmount))
                    .addParametricCallback(0.93,()->follower.setMaxPower(1.0)), true), shoot);
    public static Command gate = new SequentialCommand(new PedroCommand(
                (PathBuilder b)->b.addPath(
                        new BezierLine(
                                follower::getPose,
                                getPose("gateOpen")
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0,0.5,HeadingInterpolator.tangent),
                        HeadingInterpolator.PiecewiseNode.linear(0.5,0.8, Math.atan2(getPose("gateOpen").getY() - getPose("shoot").getY(), getPose("gateOpen").getX() - getPose("shoot").getX()), getHeading("gateOpen")),
                        new HeadingInterpolator.PiecewiseNode(0.8,1.0,HeadingInterpolator.constant(getHeading("gateOpen")))
                )),
    true).setTimeout(2),
            new Inferno.CheckFull(gateIntakeTimeout),
            new PedroCommand(
                    (PathBuilder b)->b.addPath(
                                    new BezierLine(
                                            follower::getPose,
                                            getPose("shoot")
                                    )
                            )
                            .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0.0,0.1,HeadingInterpolator.constant(getHeading("gateOpen"))),
                                    new HeadingInterpolator.PiecewiseNode(0.1,1.0, HeadingInterpolator.tangent.reverse())
                            ))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                            .addParametricCallback(shootSlowT,()->follower.setMaxPower(shootSlowAmount))
                            .addParametricCallback(0.93,()->classifierBallCount = vision.getGroundAndClassifierArtifacts(follower.getPose()).get(1).size())
                            .addParametricCallback(0.93,()->follower.setMaxPower(1.0)),
    true), shoot
    );
    public static Command firstSpike = new SequentialCommand(
            new PedroCommand((PathBuilder b)->b
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("firstSpike")
                            )
                    ).setTangentHeadingInterpolation()
                    .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("shoot")
                            )
                    ).setTangentHeadingInterpolation().setReversed()
                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                    .addParametricCallback(fourthShootSlowT,()->follower.setMaxPower(shootSlowAmount))
                    .addParametricCallback(0.93,()->classifierBallCount = vision.getGroundAndClassifierArtifacts(follower.getPose()).get(1).size())
                    .addParametricCallback(0.93,()->follower.setMaxPower(1.0)),true), shoot
    );
    public static Command thirdSpike =  new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("thirdSpikeLead")
                            )
                    ).setTangentHeadingInterpolation()
                    .addPath(new BezierCurve(
                            getPose("thirdSpikeLead"),
                            getPose("thirdSpikeCtrl"),
                            getPose("thirdSpike")
                    )).setTangentHeadingInterpolation()
                    .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("shoot")
                            )
                    ).setTangentHeadingInterpolation().setReversed()
                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                    .addParametricCallback(0.93,()->classifierBallCount = vision.getGroundAndClassifierArtifacts(follower.getPose()).get(1).size())
                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),true), shoot);
    public static Command park = new SequentialCommand(setState(null),
            new PedroCommand(b->
                    b.addPath(new BezierLine(follower::getPose,getPose("park")))
                            .setLinearHeadingInterpolation(getHeading("shoot"),getHeading("park")),
    true));
    public static ArrayList<Command> pathList = new ArrayList<>(Arrays.asList(
            preloadShoot,
            secondSpike,
            gate,
            gate,
            gate,
            gate,
            firstSpike
    ));
    public static ArrayList<String> pathListDisplay = new ArrayList<>(Arrays.asList(
            "preloadShoot",
            "secondSpike",
            "gate",
            "gate",
            "gate",
            "gate",
            "firstSpike"
    ));
    public static int selectionIndex = 0;
    public static boolean isInserting = true;
    public static void addAction(Command command, String label){
        if (isInserting) {
            pathListDisplay.add(selectionIndex,label);
            pathList.add(selectionIndex,command);
            selectionIndex+=1;
        } else {
            pathListDisplay.set(selectionIndex,label);
            pathList.set(selectionIndex,command);
        }
    }
    public static void removeAction(){
        if (!isInserting){
            pathList.remove(selectionIndex);
            pathListDisplay.remove(selectionIndex);
            if (selectionIndex>0) selectionIndex-=1;
        }
    }
    public static void runOpMode(Inferno.Alliance alliance, LinearOpMode opMode){
        airSorting = false;
        poses.put("shoot",new Pose(SHOOT[0],SHOOT[1],SHOOT[2]));
        initialize(opMode,new Inferno(),true,true);
        Inferno.useTurretSOTM = false;
        useVelFeedforward = false;
        Inferno.motifDetected = false;
        turretOffsetFromAuto = 0;
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        preloadShoot.reset();
        Inferno.alliance = alliance;
        gamePhase = Inferno.GamePhase.AUTO;
        Pedro.createFollower(getPose("start"));
        pathList = new ArrayList<>(Arrays.asList(
                preloadShoot,
                secondSpike,
                gate,
                gate,
                gate,
                gate,
                firstSpike
        ));
        pathListDisplay = new ArrayList<>(Arrays.asList(
                "preloadShoot",
                "secondSpike",
                "gate",
                "gate",
                "gate",
                "gate",
                "firstSpike"
        ));
        executor.setWriteToTelemetry(()->{
                telemetry.addData("Airsorting", airSorting);
                telemetry.addData("Offset", turretYaw.get("turretYawTop").getOffset());
                telemetry.addLine("");
                telemetry.addLine("A: First Spike");
                telemetry.addLine("B: Second Spike");
                telemetry.addLine("X: Third Spike");
                telemetry.addLine("Y: Gate");
                telemetry.addLine("LEFT_BUMPER: Preload Shoot");
                telemetry.addLine("RIGHT_BUMPER: Preload Motif Shoot");
                telemetry.addLine("BACK: Delete");
                telemetry.addLine("START: Toggle Airsort");
                telemetry.addLine("DPAD_LEFT: Second Spike Gate Open");
                telemetry.addLine("");
                for (int i=0;i<pathListDisplay.size();i++){
                    if (i==selectionIndex && isInserting){
                        telemetry.addLine("> [    ]");
                        telemetry.addLine(pathListDisplay.get(i));
                    } else if (i==selectionIndex){
                        telemetry.addLine("> ["+pathListDisplay.get(i)+"]");
                    } else telemetry.addLine(pathListDisplay.get(i));
                }
                if (selectionIndex==pathListDisplay.size()) telemetry.addLine("> [    ]");
                telemetry.addLine("");
                telemetry.addLine("Please, Speed, we need this.");
                telemetry.addLine("");
                telemetry.addData("Target Flywheel Velocity",targetFlywheelVelocity);
                telemetry.addData("Flywheel Velocity",0);
        });
        double targetX = (alliance == Inferno.Alliance.RED) ? 141.5 : 2.5;
        executor.setCommands(
                new SequentialCommand(new SleepCommand(1.0),new InstantCommand(Inferno::initTurretYawPosition)),
                Commands.triggeredToggleCommand(()->gamepad1.start, new InstantCommand(()->airSorting=true), new InstantCommand(()->airSorting=false)),
                turretYaw.command(servo->servo.instantSetTargetCommand(0*TURRET_YAW_RATIO+TURRET_YAW_OFFSET)),
                turretYaw.command(servo->servo.triggeredDynamicOffsetCommand(()->gamepad1.left_trigger>0.2,()->gamepad1.right_trigger>0.2,0.05)),
                new RunResettingLoop(new PressCommand(
                        new IfThen(()->gamepad1.dpad_up,new InstantCommand(()->{
                                if (!isInserting){
                                    isInserting=true;
                                } else {
                                    if (selectionIndex>0) {
                                        selectionIndex -= 1;
                                        isInserting = false;
                                    }
                                }
                        })),
                        new IfThen(()->gamepad1.dpad_down,new InstantCommand(()->{
                                if (isInserting && selectionIndex!=pathList.size()) {
                                    isInserting = false;
                                } else {
                                    if (selectionIndex < pathList.size()) {
                                        selectionIndex += 1;
                                        isInserting = true;
                                    }
                                }
                        })),
                        new IfThen(()->gamepad1.a,new InstantCommand(()->addAction(firstSpike,"firstSpike"))),
                        new IfThen(()->gamepad1.b,new InstantCommand(()->addAction(secondSpike,"secondSpike"))),
                        new IfThen(()->gamepad1.x,new InstantCommand(()->addAction(thirdSpike,"thirdSpike"))),
                        new IfThen(()->gamepad1.y,new InstantCommand(()->addAction(gate,"gate"))),
                        new IfThen(()->gamepad1.left_bumper,new InstantCommand(()->addAction(preloadShoot,"preloadShoot"))),
                        new IfThen(()->gamepad1.right_bumper,new InstantCommand(()->addAction(preloadMotifShoot,"preloadMotifShoot"))),
                        new IfThen(()->gamepad1.dpad_left,new InstantCommand(()->addAction(secondSpikeGateOpen,"secondSpikeGateOpen"))),
                        new IfThen(()->gamepad1.back,new InstantCommand(ClosePrimeSigmaConstants::removeAction))
                ))
        );
        turretYaw.call(servo->servo.switchControl("setPos"));
        executor.runLoop(opMode::opModeInInit);
        turretOffsetFromAuto = turretYaw.get("turretYawTop").getOffset() + YAW_FIGHT;
        Components.activateActuatorControl();
        executor.setWriteToTelemetry(()->{
            telemetry.addData("Motif",Arrays.asList(motif));
            telemetry.addData("Yaw Target", turretYaw.get("turretYawTop").getTarget());
            telemetry.addData("Target Point", List.of(targetPoint));
            telemetry.addData("Pose Heading", follower.getHeading());
            telemetry.addLine("");
            telemetry.addData("Target Flywheel Velocity", targetFlywheelVelocity);
            telemetry.addData("Flywheel Velocity", flywheel.get("flywheelLeft").getVelocity());
            telemetry.addData("Flywheel Error", targetFlywheelVelocity - flywheel.get("flywheelLeft").getVelocity());
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
        if (pathList.size()>4) {
            if (airSorting) {pathList.add(pathList.size()-3,new ParallelCommand(
                    new InstantCommand(()->poses.put("shoot",new Pose(MOTIF_SHOOT[0], MOTIF_SHOOT[1], MOTIF_SHOOT[2]))),
                    setShotType(AIRSORT))
            );}
            else {pathList.add(pathList.size()-3,setShotType(NORMAL));}
        }
        if (!pathList.isEmpty() && !airSorting) pathList.add(pathList.size()-1,new InstantCommand(()->poses.put("shoot",new Pose(FINAL_SHOOT[0], FINAL_SHOOT[1], FINAL_SHOOT[2]))));
        SequentialCommand mainPath = new SequentialCommand(pathList.toArray(new Command[0]));
        executor.setCommands(
                new ParallelCommand(findMotif, new SequentialCommand(new SleepCommand(5),new InstantCommand(findMotif::stop))),
                new SequentialCommand(
                        new ParallelCommand(
                                mainPath,
                                new SequentialCommand(
                                        new SleepCommand(29.3),
                                        new ConditionalCommand(new IfThen(
                                                ()->follower.getPose().distanceFrom(getPose("shoot"))>5 || airSorting,
                                                new SequentialCommand(
                                                        new InstantCommand(mainPath::stop),
                                                        park
                                                )
                                        ))
                                )
                        )
                ),
                Pedro.updateCommand(),
                loopFSM,
                new RunResettingLoop(
                    new InstantCommand(
                            ()->{if (!(shotType==AIRSORT)) targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("shoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));}
                    )
                )
        );
        executor.runLoop(opMode::opModeIsActive);
    }
}
