package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("rl");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.411859287051704);
        c.yPodOffset.set(5.413429080031989);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18591782169151252);
                Controller secondaryTranslationalForward = Controller.proportional(0.06869166729932472);
                Controller primaryTranslationalLateral = Controller.proportional(0.30410229581897497);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11235767254211265);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011115948064045797));
                c.brake.set(Controller.proportionalFeedforward(0.009448555854438927));

                c.headingFeedback.set(Controller.proportional(728.5585366900535));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(-0.04262935252820299, 0.076916659918455));

                c.linearBrakeCoefficients.set(Matrix.diag(19.769062389351014, -0.6282632527562289));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.28094353773406583, 0.03925858952434869));

                c.maxAchievableForwardVelocity.set(85.75658003424422);
                c.maxAchievableStrafeVelocity.set(57.16565718842405);
                c.naturalForwardDeceleration.set(32.34505947763211);
                c.naturalStrafeDeceleration.set(49.812086747122635);
            }
    );
}