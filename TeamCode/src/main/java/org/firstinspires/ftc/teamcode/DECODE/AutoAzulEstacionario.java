package org.firstinspires.ftc.teamcode.DECODE;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constantes.FConstants;
import org.firstinspires.ftc.teamcode.Constantes.LConstants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


@Autonomous(name = "Azul Estacionar")
public class AutoAzulEstacionario extends LinearOpMode {
    Follower follower;
    private DcMotor lancador, coletor, coletor2;

    int etapaAtual   = 0;
    double voltagem  = 0;

    Pose startPose           =   new Pose(64, 7.72,Math.toRadians(90));
    Pose poseDeLancamento    =   new Pose(58,13);
    Pose poseFinal           =   new Pose(30,15);
    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        lancador = hardwareMap.dcMotor.get("motorLancador");
        coletor = hardwareMap.dcMotor.get("motorCO");
        coletor2 = hardwareMap.dcMotor.get("motorCO2");
        lancador.setDirection(DcMotorSimple.Direction.REVERSE);
        PathChain pathStart = new PathBuilder()
                .addPath(criarLinha(startPose,poseDeLancamento))
                .setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(116))
                .build();
        PathChain pathFinal = new PathBuilder()
                .addPath(criarLinha(poseDeLancamento,poseFinal))
                .setConstantHeadingInterpolation(90)
                .build();

        waitForStart();
        salvarAlianca("azul");
        while (opModeIsActive()){
            follower.update();
            voltagem = hardwareMap.voltageSensor.iterator().next().getVoltage();
            salvarPosicaoDoRobo(follower);
            switch (etapaAtual){
                case 0:
                    follower.followPath(pathStart);
                    if(follower.atParametricEnd()){
                        lancarArtefato(calcularPotencia());
                        etapaAtual++;
                    }
                    break;
                case 1:
                    follower.followPath(pathFinal);
                    if(follower.atParametricEnd()){
                        etapaAtual = -1;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    Path criarLinha(Pose startPose, Pose finalPose){
        return new Path(
                new BezierLine(
                        startPose,finalPose
                )
        );
    }
    void lancarArtefato(double potencia){
        lancador.setPower(potencia);
        sleep(3000);
        coletor2.setPower(.8);
        coletor.setPower(.8);
        sleep(500);
        coletor2.setPower(0);
        coletor.setPower(0);
        sleep(1000);
        coletor2.setPower(.8);
        coletor.setPower(.8);
        sleep(1500);
        coletor2.setPower(0);
        coletor.setPower(0);
        lancador.setPower(0);
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
    void salvarAlianca(String corDaAlianca){
        corDaAlianca = corDaAlianca.toLowerCase();
        String fileName = "cor_da_alianca.cvc";

        File internalDir = hardwareMap.appContext.getFilesDir();

        File file = new File(internalDir,fileName);

        try{
            FileWriter writer = new FileWriter(file);
            writer.write(corDaAlianca);
            writer.close();

            telemetry.addData("Aliança a ser salva: ",corDaAlianca);
        }
        catch (IOException e){
            telemetry.addData("Não foi possível salvar a  aliança", "ERROR:" + e.getMessage());
        }
    }
    double calcularPotencia(){
        return 0.9*(13.5/voltagem);
    }
}
