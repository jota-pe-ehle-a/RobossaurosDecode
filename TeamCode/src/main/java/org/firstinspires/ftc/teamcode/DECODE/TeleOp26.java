package org.firstinspires.ftc.teamcode.DECODE;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

    // --- MOTORES DAS RODAS ---
    DcMotor motorDF , motorDT, motorEF, motorET;

    // --- MOTORES DOS MECANISMOS ---
    private DcMotor motorColetor;
    private DcMotor motorColetor2;
    private DcMotor motorLancador;
    private DcMotor motorSuporte;
    private CameraDecode camera;
    private double voltagem;

    String mensagemBlobVerde;
    String areaBlobVerde;
    String posicaoBlobVerde;
    String mensagemBlobRoxo;
    String areaBlobRoxo;
    String posicaoBlobRoxo;


    @Override
    public void runOpMode() {
        // --- MAPEAMENTO DOS MOTORES ---
        motorDF                = hardwareMap.get(DcMotor.class,"motorFD");
        motorDT                = hardwareMap.get(DcMotor.class,"motorTD");
        motorEF                = hardwareMap.get(DcMotor.class,"motorFE");
        motorET                = hardwareMap.get(DcMotor.class,"motorTE");

        motorColetor           = hardwareMap.get(DcMotor.class, "motorCO");
        motorColetor2          = hardwareMap.get(DcMotor.class, "motorCO2");
        motorLancador          = hardwareMap.get(DcMotor.class, "motorLancador");
        motorSuporte           = hardwareMap.get(DcMotor.class,"motorSuporte");

        motorET.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSuporte.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSuporte.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        camera = new CameraDecode(hardwareMap);
        voltagem = hardwareMap.voltageSensor.iterator().next().getVoltage();

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

        try { // Boa prática para garantir que a câmera seja desligada
            while (opModeIsActive()) {
                // --- MOVIMENTAÇÃO ---
                double potenciaDir = -gamepad1.left_stick_y - gamepad1.left_stick_x;
                double potenciaEsq = -gamepad1.left_stick_y + gamepad1.left_stick_x;

                salvarPosicaoDoRobo(follower);
                verArtefato();
                if(camera.getDetection() != null){
                    telemetry.addData("Distância até o GOAL: ",camera.getDetection().ftcPose.range-31+" cm");
                }
                telemetry.addData("Artefato Roxo: ",mensagemBlobRoxo);
                telemetry.addData("Área Roxo: ",areaBlobRoxo);
                telemetry.addData("Posição Roxo: ",posicaoBlobRoxo);
                telemetry.addData("Artefato Verde: ",mensagemBlobVerde);
                telemetry.addData("Área Verde: ",areaBlobVerde);
                telemetry.addData("Posição verde: ",posicaoBlobVerde);
                telemetry.update();
                follower.update();
                if(gamepad1.a && !fezEndGame && estaEntre(new Pose(48,0),new Pose(96,36))){
                    fezEndGame = true;
                    pathFinal = new Path(
                            new BezierLine(
                                    follower.getPose(), endGamePoseAzul
                            )
                    );
                    pathFinal.setLinearHeadingInterpolation(follower.getPose().getHeading(), endGamePoseAzul.getHeading());
                }
                else if(gamepad1.b && !fezEndGame && estaEntre(new Pose(48,0),new Pose(96,36))){
                    fezEndGame = true;
                    pathFinal = new Path(
                            new BezierLine(
                                    follower.getPose(), endGamePoseVermeho
                            )
                    );
                    pathFinal.setLinearHeadingInterpolation(follower.getPose().getHeading(), endGamePoseVermeho.getHeading());
                }
                else if(pathFinal != null && !acabouEndGame){
                    follower.followPath(pathFinal);
                    if(follower.atParametricEnd()){
                        follower.breakFollowing();
                        follower.startTeleopDrive();
                        acabouEndGame = true;
                        fezEndGame = false;
                    }
                }
                else if(gamepad1.x && poseAjustada() != null){
                    follower.followPath(new Path(
                            new BezierLine(
                                    follower.getPose(),poseAjustada()
                            )
                    ));
                    if(follower.atParametricEnd()){
                        follower.breakFollowing();
                        follower.startTeleopDrive();
                    }
                }
                // ====================================================================
                // ========= CONTROLE DOS EFETUADORES (gamepad2) =========
                // ====================================================================
                // (Seu código de controle dos mecanismos, que está correto)
                motorColetor.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
                if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                    motorColetor2.setPower(1);
                } else if(gamepad2.left_bumper && !gamepad2.right_bumper){
                    motorColetor2.setPower(-1);
                } else if(!gamepad2.right_bumper && !gamepad2.left_bumper){
                    motorColetor2.setPower(0);
                }
                if (gamepad2.x) {
                    lancarArtefato(.5);
                } else if (gamepad2.y) {
                    lancarArtefato(.6);
                } else if (gamepad2.b) {
                    lancarArtefato(.7);
                } else {
                    motorLancador.setPower(0);
                    motorSuporte.setPower(0);

                }
            }
        } finally  {
            // Garante que a câmera seja desligada ao final.
            if (camera != null) {
                camera.close();
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
    void verArtefato(){
        if(camera.blobRoxo() != null){
            mensagemBlobRoxo = "Detectado";
            areaBlobRoxo = String.valueOf(camera.blobRoxo().getContourArea());
            posicaoBlobRoxo = String.valueOf(camera.blobRoxo().getBoxFit().center);
        } else if(camera.blobRoxo() == null){
            mensagemBlobRoxo = "Não detectado";
            areaBlobRoxo = "Indefinida";
            posicaoBlobRoxo = "Indefinida";
        }
        if(camera.blobVerde() != null){
            mensagemBlobVerde = "Detectado";
            areaBlobVerde = String.valueOf(camera.blobVerde().getContourArea());
            posicaoBlobVerde = String.valueOf(camera.blobVerde().getBoxFit().center);
        } else if(camera.blobVerde() == null){
            mensagemBlobVerde = "Não detectado";
            areaBlobVerde = "Indefinida";
            posicaoBlobVerde = "Indefinida";
        }
    }
    boolean estaEntre(Pose ponto1, Pose ponto2){
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
    void salvarPosicaoDoRobo(Follower follower){
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
    Pose poseAjustada(){
        AprilTagDetection detection = camera.getDetection();
        if (detection != null && (detection.id == 20 || detection.id == 24)) {
            double novoAngulo = follower.getPose().getHeading() + Math.toRadians(detection.ftcPose.bearing);
            return new Pose(follower.getPose().getX()+0.01,follower.getPose().getY()+0.01, novoAngulo);
        } else {
            return null;
        }
    }
    void lancarArtefato(double potencia){
        motorLancador.setPower(potencia*(13.5/voltagem));
        motorSuporte.setPower(potencia*(13.5/voltagem));
    }
}