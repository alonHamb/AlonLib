package org.firstinspires.ftc.pedropathing;

import static org.firstinspires.ftc.teamcode.RobotMap.Drive.LEFT_BACK_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.RobotMap.Drive.LEFT_FRONT_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.RobotMap.Drive.RIGHT_BACK_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.RobotMap.Drive.RIGHT_FRONT_MOTOR_ID;

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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(15);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RIGHT_FRONT_MOTOR_ID)
            .rightRearMotorName(RIGHT_BACK_MOTOR_ID)
            .leftFrontMotorName(LEFT_FRONT_MOTOR_ID)
            .leftRearMotorName(LEFT_BACK_MOTOR_ID)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(.5).distanceUnit(DistanceUnit.METER).hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants).pinpointLocalizer(localizerConstants)
                .build();
    }
}
