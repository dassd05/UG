package org.firstinspires.ftc.teamcode.legacycode.util;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Logs {
    public String fileName;
    public String absFilePath;
    File file;

    public enum type {
        NEW_VERS,
        LAST_VERS,
        FIRST_VERS
    }

    public Logs(String fileName) {
        this.fileName = fileName;
    }

    public static String filePathConvert(String fileName) {
        return String.format("%s/FIRST/data/"+fileName+".txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    }
    public static String filePathConvert(String fileName, int fileVers) {
        return String.format("%s/FIRST/data/"+fileName+fileVers+".txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    }
    public static String filePathConvert(String fileName, String fileVers) {
        return String.format("%s/FIRST/data/"+fileName+fileVers+".txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    }

    public boolean fileCreate() {
        try {
            if (file.createNewFile()) {

            } else {
                return false;
            }
        } catch (IOException e) {

        }
        return true;
    }
    public void writeFile(String filePath, String data) {
        try {
            FileWriter myWriter = new FileWriter(filePath, true);
            myWriter.write(data + "\n");
            myWriter.close();
//            telemetry.addData("File Writing", "Success");
        } catch (IOException e) {
//            telemetry.addData("File Writing", "Fail");
//            telemetry.addData("Error", e.getStackTrace());
//            telemetry.addData("Error", e.getLocalizedMessage());
//            telemetry.addData("Error", e.getMessage());
//            telemetry.addData("Error", e.getSuppressed());
        }
    }

}
