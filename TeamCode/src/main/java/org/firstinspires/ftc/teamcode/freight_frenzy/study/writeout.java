package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class writeout {
    public static writeout instance;

    public writeout() {

    }

    public boolean createFile(String filepath) {
        boolean returning = false;
        File tempFile = new File(System.getProperty("user.dir") + filepath);
        try {
            if(tempFile.createNewFile()) {
                returning = true;   
            }
        } catch (IOException e) {
            e.printStackTrace(); }
        return returning;
    }



    public static writeout getInstance() {
        if(instance == null) instance = new writeout();
        return instance;
    }

    public static void setInstance(writeout instance) {
        writeout.instance = instance;
    }
}
