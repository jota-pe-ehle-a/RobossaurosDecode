package org.firstinspires.ftc.teamcode.Constantes;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = .002965;
        ThreeWheelIMUConstants.strafeTicksToInches = .002965;
        ThreeWheelIMUConstants.turnTicksToInches = .002965;
        ThreeWheelIMUConstants.leftY = 4.25;
        ThreeWheelIMUConstants.rightY = -4.25;
        ThreeWheelIMUConstants.strafeX = 1.25;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "motorFD"; // ctrlHub 01
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "motorTE"; // ctrlHub 02
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "motorTD"; // ctrlHub 03
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    }
}




