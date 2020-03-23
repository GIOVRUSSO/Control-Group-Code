/*
 * Class Name: Upload.java
 * Corresponding layout: activity_view_data.xml
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
 * Description: Uploads data to dropbox
 * */

package ie.ucd.smartrideRT;

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

public class Upload {
    private static final String ACCESS_TOKEN = "Dropbox key here";


    public static void upload(String dataLabel) throws DbxException, IOException {

        /**
         * Create Dropbox client
         */
        DbxRequestConfig config = new DbxRequestConfig("dropbox/SmartRide2");
        DbxClientV2 client = new DbxClientV2(config, ACCESS_TOKEN);
        Log.d("DBX","client");


        /**
         * Get current account info
         */

        /*
        FullAccount account = client.users().getCurrentAccount();
        System.out.println(account.getName().getDisplayName());
        Log.d("DBX","name");
        */


        /**
         * Get files and folder metadata from Dropbox root directory
        */
        /*
        ListFolderResult result = client.files().listFolder("");
        while (true) {
            for (Metadata metadata : result.getEntries()) {
                System.out.println(metadata.getPathLower());
            }

            if (!result.getHasMore()) {
                break;
            }

            result = client.files().listFolderContinue(result.getCursor());
        }
        */

        /**
         * Upload data file to Dropbox
         */

        try (InputStream in = new FileInputStream(Environment.getDataDirectory().getPath()+"/data/ie.ucd.smartrideRT/databases/data.db");
        ) {
            Log.d("UP","ADD");
            FileMetadata metadata = client.files().uploadBuilder("/"+dataLabel+".db")
                    .withMode(WriteMode.OVERWRITE)
                    .uploadAndFinish(in);
        }
        catch (DbxException ex) {
            System.out.println(ex.getMessage());
        }

    }
}
