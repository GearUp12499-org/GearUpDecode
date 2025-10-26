package org.firstinspires.ftc.teamcode.hardware;

import android.os.Environment;

import java.io.File;

public class FileUtil {

    public static File getfile(){
        String dirPath = Environment.getExternalStorageDirectory().getPath() + "/data";
        File dir = new File(dirPath);
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        int i = 0;
        File f;
        do {
            i++;
            f = new File(dirPath + "/" + i + ".csv");
        } while (f.exists());
        return f;
    }
}