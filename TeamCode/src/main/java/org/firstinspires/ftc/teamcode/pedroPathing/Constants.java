package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.7)
            .forwardZeroPowerAcceleration(-31.9940432459363933)
            .lateralZeroPowerAcceleration(-68.69912366702141)
            .headingPIDFCoefficients(new PIDFCoefficients(1.1,0.001,0.00001,0))
            .centripetalScaling(0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.04,0.112767110976556211,0.0014754081917193876961));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(67.55921082984744)
            .yVelocity(53.24798439806841);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.76925)
            .strafePodX(-2.625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            0.1,
            0.1,
            0.004,
            7,
            1.0,
            10,
            0.85
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        followerConstants.usePredictiveBraking = true;
        Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
        return follower;
    }
}
