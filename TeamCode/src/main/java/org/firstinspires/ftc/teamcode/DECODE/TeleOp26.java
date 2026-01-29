package org.firstinspires.ftc.teamcode.DECODE;


import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constantes.FConstants;
import org.firstinspires.ftc.teamcode.Constantes.LConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;


@TeleOp(name="TeleOp26")
public class TeleOp26 extends LinearOpMode {

    Follower follower;

    // --- MOTORES DOS MECANISMOS ---
    private DcMotor motorColetor;
    private DcMotor motorColetor2;
    private DcMotor motorLancador;
    private double voltagem;



    @Override
    public void runOpMode() {
        // --- MAPEAMENTO DOS MOTORES ---

        motorColetor           = hardwareMap.get(DcMotor.class, "motorCO");
        motorColetor2          = hardwareMap.get(DcMotor.class, "motorCO2");
        motorLancador          = hardwareMap.get(DcMotor.class, "motorLancador");

        motorLancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLancador.setDirection(DcMotorSimple.Direction.REVERSE);


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(carregarPose());

        telemetry.addData("Status", "Robô Pronto!");
        telemetry.update();
        boolean fezEndGame = false;
        boolean acabouEndGame = false;
        Pose endGamePoseAzul = new Pose(105,36,Math.toRadians(90));
        Pose endGamePoseVermeho = new Pose(39,36,Math.toRadians(90));
        Path pathFinal = null;
        waitForStart();
        follower.startTeleopDrive();
            while (opModeIsActive()) {
                voltagem = hardwareMap.voltageSensor.iterator().next().getVoltage();
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x);
                salvarPosicaoDoRobo(follower);
                telemetry.update();
                follower.update();
                if(gamepad1.a && !fezEndGame && estaEntre(new Pose(48,0),new Pose(96,36))){
                    fezEndGame = true;
                    pathFinal = new Path(
                            new BezierLine(
                                    follower.getPose(), endGamePoseAzul
                            )
                    );
                    pathFinal.setConstantHeadingInterpolation(Math.toRadians(90));
                }
                else if(gamepad1.b && !fezEndGame && estaEntre(new Pose(48,0),new Pose(96,36))){
                    fezEndGame = true;
                    pathFinal = new Path(
                            new BezierLine(
                                    follower.getPose(), endGamePoseVermeho
                            )
                    );
                    pathFinal.setConstantHeadingInterpolation(Math.toRadians(90));
                }
                else if(pathFinal != null && !acabouEndGame){
                    follower.followPath(pathFinal);
                    if(follower.atParametricEnd()){
                        follower.breakFollowing();
                        follower.startTeleopDrive();
                        acabouEndGame = true;
                        fezEndGame = false;
                        pathFinal = null;
                    }
                }
                // ====================================================================
                // ========= CONTROLE DOS EFETUADORES (gamepad2) =========
                // ====================================================================
                // (Seu código de controle dos mecanismos, que está correto)
                motorColetor.setPower((gamepad2.right_trigger-gamepad2.left_trigger)*0.7);
                if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                    motorColetor2.setPower(1);
                }
                else if(gamepad2.left_bumper && !gamepad2.right_bumper){
                    motorColetor2.setPower(-1);
                }
                else if(!gamepad2.right_bumper && !gamepad2.left_bumper){
                    motorColetor2.setPower(0);
                }
                else if (gamepad2.x) {
                    lancarArtefato(.55);
                }
                else if (gamepad2.y) {
                    lancarArtefato(.7);
                }
                else if (gamepad2.b) {
                    lancarArtefato(.85);
                }
                else {
                    motorLancador.setPower(0);
                }
            }

    }
    Pose carregarPose(){
        String fileName = "current_robot_position.cvc";
        File internalDir = hardwareMap.appContext.getFilesDir();

        File file = new File(internalDir,fileName);

        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));
            String line = reader.readLine();
            String[] parts = line.split(",");
            if(parts.length == 3){
                double xAtual = Double.parseDouble(parts[0]);
                double yAtual = Double.parseDouble(parts[1]);
                double hAtual = Double.parseDouble(parts[2]);

                return new Pose(xAtual,yAtual,hAtual);
            }
        } catch (IOException | NumberFormatException e) {
            telemetry.addData("ERRO Pose", "Não foi possível carregar a Pose: " + e.getMessage());
        }
        return new Pose(0,0,0);
    }
    boolean estaEntre(@NonNull Pose ponto1, Pose ponto2){
        boolean Xverdadeiro = false;
        boolean Yverdadeiro = false;
        if(follower.getPose().getX() > ponto1.getX() && follower.getPose().getX() < ponto2.getX()){
            Xverdadeiro = true;
        }
        if(follower.getPose().getY() > ponto1.getY() && follower.getPose().getY() < ponto2.getY()){
            Yverdadeiro = true;
        }
        return Xverdadeiro && Yverdadeiro;
    }
    void salvarPosicaoDoRobo(@NonNull Follower follower){
        double xAgora = follower.getPose().getX();
        double yAgora = follower.getPose().getY();
        double hAgora = follower.getPose().getHeading();

        String fileName = "current_robot_position.cvc";

        File internalDir = hardwareMap.appContext.getFilesDir();

        File file = new File(internalDir,fileName);
        String dataToSave = xAgora + "," + yAgora + "," + hAgora;

        try{
            FileWriter writer = new FileWriter(file);
            writer.write(dataToSave);
            writer.close();

            telemetry.addData("Pose a ser salva: ", "X: %.2f , Y: %.2f , Heading: %.2f",xAgora,yAgora,hAgora);
        } catch (IOException e){
            telemetry.addData("Não foi possível salvar a pose do robô","ERROR:" + e.getMessage());
        }
    }
    void lancarArtefato(double potencia){
        motorLancador.setPower(potencia*(13.5/voltagem));
    }
}