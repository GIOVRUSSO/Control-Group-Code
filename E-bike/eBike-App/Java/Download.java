/*
 * Class Name: Download.java
 * Corresponding layout: activity_view_data.xml
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
 * Description: Downloads current assistance level from Dropbox to implement
 * */

package ie.ucd.smartrideRT;

import android.content.Intent;
import android.os.Environment;
import android.util.Log;


import com.dropbox.core.DbxException;
import com.dropbox.core.DbxRequestConfig;
import com.dropbox.core.v2.DbxClientV2;
import com.dropbox.core.v2.files.FileMetadata;
import com.dropbox.core.v2.files.ListFolderResult;
import com.dropbox.core.v2.files.Metadata;
import com.dropbox.core.v2.users.FullAccount;
import com.dropbox.core.v2.files.WriteMode;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.ArrayList;
import java.util.List;


public class Download {

    public static int alPercentage;
    private static final String ACCESS_TOKEN = "Dropbox key here";




    public static void download() throws DbxException, IOException {


        DbxRequestConfig config = new DbxRequestConfig("dropbox/SmartRide2");
        DbxClientV2 client = new DbxClientV2(config, ACCESS_TOKEN);
        Log.d("DBX", "download");



        try {
            List<String> files = new ArrayList<String>();
            ListFolderResult listFolderResult = client.files().listFolder("/ALvalue");
            for (Metadata metadata : listFolderResult.getEntries()) {
                String name = metadata.getName();
                if (name.endsWith(".csv")) {
                    alPercentage = Integer.parseInt(name.substring(0, name.lastIndexOf('.')));
                }
            }
            Log.d("DBX", "download assistance level = "+alPercentage);
            BLEService.wirteToBLE(-1);
            BLEService.wirteToBLE(alPercentage);
            Log.d("BLE", "Sent assistance level = "+alPercentage);

        }catch(DbxException ex) {
                    System.out.println(ex.getMessage());
        }
    }
}
