package org.firstinspires.ftc.teamcode.DECODE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Teste Motores")
public class TesteMotores extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("motorFE");
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                motor.setPower(0.6);
            }
            else {
                motor.setPower(0);
            }
            telemetry.addData("Porta do motor:","ControlHub 00");
            telemetry.addData("Botão a ser pressionado:", "gamepad1.a");
            telemetry.addData("Está sendo pressionado: ",gamepad1.a);
            telemetry.update();
        }
    }
}
