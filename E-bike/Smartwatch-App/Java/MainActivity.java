package com.empatica.E4Link;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.provider.Settings;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;


import com.empatica.empalink.ConnectionNotAllowedException;
import com.empatica.empalink.EmpaDeviceManager;
import com.empatica.empalink.EmpaticaDevice;
import com.empatica.empalink.config.EmpaSensorStatus;
import com.empatica.empalink.config.EmpaSensorType;
import com.empatica.empalink.config.EmpaStatus;
import com.empatica.empalink.delegate.EmpaDataDelegate;
import com.empatica.empalink.delegate.EmpaStatusDelegate;


import java.io.FileOutputStream;
import java.io.File;

import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


public class MainActivity extends AppCompatActivity implements EmpaDataDelegate, EmpaStatusDelegate {

    private static final String TAG = "MainActivity";

    private final ExecutorService executorService = Executors.newFixedThreadPool(10);

    private static final int REQUEST_ENABLE_BT = 1;

    private static final int REQUEST_PERMISSION_ACCESS_COARSE_LOCATION = 1;

    private static final int REQUEST_PERMISSION_ACCESS_STORAGE = 2;

    private static final int REQUEST_PERMISSION = 3;

    // TODO insert your API Key here
    private static final String EMPATICA_API_KEY = "";

    private EmpaDeviceManager deviceManager = null;

    private TextView accel_xLabel;

    private TextView accel_yLabel;

    private TextView accel_zLabel;

    private TextView bvpLabel;

    private TextView edaLabel;

    private TextView ibiLabel;

    private TextView temperatureLabel;

    private TextView batteryLabel;

    private TextView statusLabel;

    private TextView deviceNameLabel;

    private LinearLayout dataCnt;



    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        /**
         * Permissions
         */
        if (checkAndRequestPermissions()) {
            // carry on the normal flow, as the case of  permissions  granted.
        }


        /**
         * Initialize vars that reference UI components E4
         */
        statusLabel = (TextView) findViewById(R.id.status);

        dataCnt = (LinearLayout) findViewById(R.id.dataArea);

        accel_xLabel = (TextView) findViewById(R.id.accel_x);

        accel_yLabel = (TextView) findViewById(R.id.accel_y);

        accel_zLabel = (TextView) findViewById(R.id.accel_z);

        bvpLabel = (TextView) findViewById(R.id.bvp);

        edaLabel = (TextView) findViewById(R.id.eda);

        ibiLabel = (TextView) findViewById(R.id.ibi);

        temperatureLabel = (TextView) findViewById(R.id.temperature);

        batteryLabel = (TextView) findViewById(R.id.battery);

        deviceNameLabel = (TextView) findViewById(R.id.deviceName);

        final Button disconnectButton = findViewById(R.id.disconnectButton);
        disconnectButton.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {

                if (deviceManager != null) {

                    deviceManager.disconnect();
                }
            }

        });

        initEmpaticaDeviceManager();
    }

    /**
     * Permissions Request
     */

    String[] permissions = new String[]{Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.READ_EXTERNAL_STORAGE
    };

    private boolean checkAndRequestPermissions() {
        int result;
        List<String> mPermissionList = new ArrayList<>();
        for (String p : permissions) {
            result = ContextCompat.checkSelfPermission(this, p);
            if (result != PackageManager.PERMISSION_GRANTED) {
                mPermissionList.add(p);
            }
        }
        if (!mPermissionList.isEmpty()) {
            ActivityCompat.requestPermissions(this, mPermissionList.toArray(new String[mPermissionList.size()]), REQUEST_PERMISSION);
            return false;
        }
        return true;
    }


    @Override
    public void onRequestPermissionsResult(int requestCode, String permissionsList[], @NonNull int[] grantResults) {
        switch (requestCode) {
            //STORAGE
            case REQUEST_PERMISSION:
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // Permission was granted, yay!
                    initEmpaticaDeviceManager();
                } else {
                    // Permission denied, boo!
                    final boolean needRationale1 = ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.WRITE_EXTERNAL_STORAGE);
                    new AlertDialog.Builder(this)
                            .setTitle("Permission required")
                            .setMessage("Without this permission application cannot be started, allow it in order to start the appllication.")
                            .setPositiveButton("Retry", new DialogInterface.OnClickListener() {
                                public void onClick(DialogInterface dialog, int which) {
                                    // try again
                                    if (needRationale1) {
                                        // the "never ask again" flash is not set, try again with permission request
                                        initEmpaticaDeviceManager();
                                    } else {
                                        // the "never ask again" flag is set so the permission requests is disabled, try open app settings to enable the permission
                                        Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                                        Uri uri = Uri.fromParts("package", getPackageName(), null);
                                        intent.setData(uri);
                                        startActivity(intent);
                                    }
                                }
                            })
                            .setNegativeButton("Exit application", new DialogInterface.OnClickListener() {
                                public void onClick(DialogInterface dialog, int which) {
                                    // without permission exit is the only way
                                    finish();
                                }
                            })
                            .show();
                }
                break;
        }
            /*
            //LOCATION
            case REQUEST_PERMISSION_ACCESS_COARSE_LOCATION:
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // Permission was granted, yay!
                    initEmpaticaDeviceManager();
                } else {
                    // Permission denied, boo!
                    final boolean needRationale = ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.ACCESS_COARSE_LOCATION);
                    new AlertDialog.Builder(this)
                            .setTitle("Permission required")
                            .setMessage("Without this permission bluetooth low energy devices cannot be found, allow it in order to connect to the device.")
                            .setPositiveButton("Retry", new DialogInterface.OnClickListener() {
                                public void onClick(DialogInterface dialog, int which) {
                                    // try again
                                    if (needRationale) {
                                        // the "never ask again" flash is not set, try again with permission request
                                        initEmpaticaDeviceManager();
                                    } else {
                                        // the "never ask again" flag is set so the permission requests is disabled, try open app settings to enable the permission
                                        Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                                        Uri uri = Uri.fromParts("package", getPackageName(), null);
                                        intent.setData(uri);
                                        startActivity(intent);
                                    }
                                }
                            })
                            .setNegativeButton("Exit application", new DialogInterface.OnClickListener() {
                                public void onClick(DialogInterface dialog, int which) {
                                    // without permission exit is the only way
                                    finish();
                                }
                            })
                            .show();
                }
                break;

            //STORAGE
            case REQUEST_PERMISSION_ACCESS_STORAGE:
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // Permission was granted, yay!
                    initEmpaticaDeviceManager();
                } else {
                    // Permission denied, boo!
                    final boolean needRationale1 = ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.WRITE_EXTERNAL_STORAGE);
                    new AlertDialog.Builder(this)
                            .setTitle("Permission required")
                            .setMessage("Without this permission storage cannot be accessed, allow it in order to access the storage.")
                            .setPositiveButton("Retry", new DialogInterface.OnClickListener() {
                                public void onClick(DialogInterface dialog, int which) {
                                    // try again
                                    if (needRationale1) {
                                        // the "never ask again" flash is not set, try again with permission request
                                        initEmpaticaDeviceManager();
                                    } else {
                                        // the "never ask again" flag is set so the permission requests is disabled, try open app settings to enable the permission
                                        Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                                        Uri uri = Uri.fromParts("package", getPackageName(), null);
                                        intent.setData(uri);
                                        startActivity(intent);
                                    }
                                }
                            })
                            .setNegativeButton("Exit application", new DialogInterface.OnClickListener() {
                                public void onClick(DialogInterface dialog, int which) {
                                    // without permission exit is the only way
                                    finish();
                                }
                            })
                            .show();
                }
                break;
        }
        */

    }


    private void initEmpaticaDeviceManager() {
        // Android 6 (API level 23) now require ACCESS_COARSE_LOCATION permission to use BLE
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_COARSE_LOCATION}, REQUEST_PERMISSION_ACCESS_COARSE_LOCATION);
        } else {

            if (TextUtils.isEmpty(EMPATICA_API_KEY)) {
                new AlertDialog.Builder(this)
                        .setTitle("Warning")
                        .setMessage("Please insert your API KEY")
                        .setNegativeButton("Close", new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int which) {
                                // without permission exit is the only way
                                finish();
                            }
                        })
                        .show();
                return;
            }

            // Create a new EmpaDeviceManager. MainActivity is both its data and status delegate.
            deviceManager = new EmpaDeviceManager(getApplicationContext(), this, this);

            // Initialize the Device Manager using your API key. You need to have Internet access at this point.
            deviceManager.authenticateWithAPIKey(EMPATICA_API_KEY);
        }
    }


    @Override
    protected void onPause() {
        super.onPause();
        if (deviceManager != null) {
            deviceManager.stopScanning();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (deviceManager != null) {
            deviceManager.cleanUp();
        }
    }

    @Override
    public void didDiscoverDevice(EmpaticaDevice bluetoothDevice, String deviceName, int rssi, boolean allowed) {
        // Check if the discovered device can be used with your API key. If allowed is always false,
        // the device is not linked with your API key. Please check your developer area at
        // https://www.empatica.com/connect/developer.php
        if (allowed) {
            // Stop scanning. The first allowed device will do.
            deviceManager.stopScanning();
            try {
                // Connect to the device
                deviceManager.connectDevice(bluetoothDevice);
                updateLabel(deviceNameLabel, "To: " + deviceName);
            } catch (ConnectionNotAllowedException e) {
                // This should happen only if you try to connect when allowed == false.
                Toast.makeText(MainActivity.this, "Sorry, you can't connect to this device", Toast.LENGTH_SHORT).show();
            }
        }
    }

    @Override
    public void didRequestEnableBluetooth() {
        // Request the user to enable Bluetooth
        Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // The user chose not to enable Bluetooth
        if (requestCode == REQUEST_ENABLE_BT && resultCode == Activity.RESULT_CANCELED) {
            // You should deal with this
            return;
        }
        super.onActivityResult(requestCode, resultCode, data);

    }

    @Override
    public void didUpdateSensorStatus(@EmpaSensorStatus int status, EmpaSensorType type) {

        didUpdateOnWristStatus(status);
    }


    /**
     * Control UI accoring to the status to BLE connection
     */

    @Override
    public void didUpdateStatus(EmpaStatus status) {
        // Update the UI
        updateLabel(statusLabel, status.name());

        // The device manager is ready for use
        if (status == EmpaStatus.READY) {
            updateLabel(statusLabel, status.name() + " - Turn on your device");
            // Start scanning
            deviceManager.startScanning();

            // The device manager has established a connection
            hide();

        } else if (status == EmpaStatus.CONNECTED) {

            show();

            // The device manager disconnected from a device
        } else if (status == EmpaStatus.DISCONNECTED) {

            updateLabel(deviceNameLabel, "Waiting for a new device");

            hide();
        }
    }


    //List<Float> AccX = new ArrayList<>();
    //List<Float> AccY = new ArrayList<>();
    //List<Float> AccZ = new ArrayList<>();

    //List<Float> BVPs = new ArrayList<>();
    //List<Float> EDAs = new ArrayList<>();
    //List<Float> IBIs = new ArrayList<>();
    //List<Float> TEMPs = new ArrayList<>();
    //List<Float> BATTERYs = new ArrayList<>();

    //int lastSavedMinute = -1;


    int lastSaveMinAcc = -1;
    List<Float> AccX = new ArrayList<>();
    List<Integer> AccY = new ArrayList<>();
    List<Integer> AccZ = new ArrayList<>();
    /**
     * ACC
     */
    @Override
    public void didReceiveAcceleration(final int x, final int y, final int z, double timestamp) {
        updateLabel(accel_xLabel, "" + x);
        updateLabel(accel_yLabel, "" + y);
        updateLabel(accel_zLabel, "" + z);

/*
        Calendar acc =  Calendar.getInstance();
        int currentMinute = acc.get(Calendar.MINUTE);
        if (lastSaveMinAcc == -1)
            lastSaveMinAcc = currentMinute;
        if (currentMinute != lastSaveMinAcc) {

            save(AccX, "AccX");
            AccX.clear();
            lastSaveMinAcc = currentMinute;
        }

        AccX.add();
*/
        // save ACC data to the local memory
        //save(x, "accel_xLabel");
        //save(y, "accel_yLabel");
        //save(z, "accel_zLabel");
        //Log.d("debug", "ACC "+ System.currentTimeMillis() + "");
    }

    /**
     * BVP
     */
    int lastSaveMinBvp = -1;
    List<Float> BVPs = new ArrayList<>();
    @Override
    public void didReceiveBVP(final float bvp, double timestamp) {
        updateLabel(bvpLabel, "" + bvp);

        //save(bvp, "BVP");
        //Log.d("debug", "BVP "+System.currentTimeMillis() + "");
        //Log.d("debug", "BVP ");
        //Calendar c_bvp =  Calendar.getInstance();
        //int currentMinute = c_bvp.get(Calendar.MINUTE);
        //if (lastSaveMinBvp == -1)
        //    lastSaveMinBvp = currentMinute;
        //if (currentMinute != lastSaveMinBvp) {
        //    save(BVPs, "BVP");
        //    BVPs.clear();
        //    lastSaveMinBvp = currentMinute;
        //}

        BVPs.add(bvp);

    }

    /**
     * IBI
     */

    int lastSaveMinIbi = -1;
    List<Float> IBIs = new ArrayList<>();
    @Override
    public void didReceiveIBI(final float ibi, double timestamp) {
        updateLabel(ibiLabel, "" + ibi);

        Calendar c =  Calendar.getInstance();
        int currentMinute = c.get(Calendar.MINUTE);
        if (lastSaveMinIbi == -1)
            lastSaveMinIbi = currentMinute;
        if (currentMinute != lastSaveMinIbi) {
            save(IBIs, "IBI");
            IBIs.clear();
            lastSaveMinIbi = currentMinute;
        }

        IBIs.add(ibi);

//
       Log.d("debug", "IBI ");
//        executorService.execute(new Runnable() {
//            @Override
//            public void run() {
//                try{
//                    Thread.sleep(30);
//
//                    save(ibi, "IBI");
//                } catch (InterruptedException e) {
//                    System.out.println("Interrupted (IBI)");
//                }
//
//            }
//        });
//
    }


    /**
     * EDA
     */
    int lastSaveMinGSR = -1;
    List<Float> GSRs = new ArrayList<>();
    @Override
    public void didReceiveGSR (final float gsr, double timestamp){
        updateLabel(edaLabel, "" + gsr);

        Calendar c_gsr =  Calendar.getInstance();
        int currentMinute = c_gsr.get(Calendar.MINUTE);
        if (lastSaveMinGSR == -1)
            lastSaveMinGSR = currentMinute;
        if (currentMinute != lastSaveMinGSR) {
            save(GSRs, "EDA");
            GSRs.clear();
            lastSaveMinGSR = currentMinute;
        }

        GSRs.add(gsr);
    }



    /**
     * TEMP
     */
    int lastSaveMinTEMP = -1;
    List<Float> TEMPs = new ArrayList<>();
    @Override
    public void didReceiveTemperature (final float temp, double timestamp){
        updateLabel(temperatureLabel, "" + temp);

        //Log.d("debug", "TEMP ");

        //Calendar c_temp =  Calendar.getInstance();
        //int currentMinute = c_temp.get(Calendar.MINUTE);
        //if (lastSaveMinTEMP == -1)
        //    lastSaveMinTEMP = currentMinute;
        //if (currentMinute != lastSaveMinTEMP) {
        //    save(TEMPs, "TEMP");
        //   TEMPs.clear();
        //    lastSaveMinTEMP = currentMinute;
        //}

        TEMPs.add(temp);

    }

    /**
     * Battery
     */
    int lastSaveMinBatt= -1;
    List<Float> BATTs = new ArrayList<>();
    @Override
    public void didReceiveBatteryLevel (final float battery, double timestamp){
        updateLabel(batteryLabel, String.format("%.0f %%", battery * 100));

            //Log.d("debug", "Battery "+System.currentTimeMillis() + "");

        //Log.d("debug", "BATTERY");

        //Calendar c_temp =  Calendar.getInstance();
        //int currentMinute = c_temp.get(Calendar.MINUTE);
        //if (lastSaveMinBatt == -1)
        //    lastSaveMinBatt = currentMinute;
        //if (currentMinute != lastSaveMinBatt) {
        //    save(BATTs, "BATTERY");
        //    BATTs.clear();
        //    lastSaveMinBatt = currentMinute;
        //}

        BATTs.add(battery);


    }

    /**
     * Update a label with some text, making sure this is run in the UI thread
     */
    private void updateLabel ( final TextView label, final String text){
        runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    label.setText(text);
                }
            });
    }


    /**
     * Tag
     */
    @Override
    public void didReceiveTag ( double timestamp){
    }

    @Override
    public void didEstablishConnection () {
        show();
    }

    @Override
    public void didUpdateOnWristStatus ( @EmpaSensorStatus final int status){

        runOnUiThread(new Runnable() {

                @Override
                public void run() {
                    if (status == EmpaSensorStatus.ON_WRIST) {

                        ((TextView) findViewById(R.id.wrist_status_label)).setText("ON WRIST");
                    } else {

                        ((TextView) findViewById(R.id.wrist_status_label)).setText("NOT ON WRIST");
                    }
                }
            });
        }

    void show () {

       runOnUiThread(new Runnable() {

                @Override
                public void run() {

         dataCnt.setVisibility(View.VISIBLE);
         // dataCnt IS A LINEAR lAYOUT
            }
            });
    }


    void hide () {
        // textView and disconnect button invisible
        runOnUiThread(new Runnable() {

                @Override
                public void run() {
                    dataCnt.setVisibility(View.INVISIBLE);
                }
            });
    }

    /**
     * Save file at internal storage -> Empa
     *
     * @param saveData dataLabel
     */
    void save (List<Float> saveData, String dataLabel){
        try {
            File directory = new File(Environment.getExternalStorageDirectory().getPath() + "/Empa");
            if (!directory.exists()) {
                directory.mkdir();
            }
            SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd-HH-mm");
            String time = format.format(new Date(System.currentTimeMillis()-1));
            String fileName = dataLabel + "Data" + time + ".txt";
            String file_path = Environment.getExternalStorageDirectory().getPath() + "/Empa/" + dataLabel + "/" + fileName;

            File file = new File(file_path);
            //file.setExecutable(true);
            if (!file.exists()) {
                File dir = new File(file.getParent());
                dir.mkdirs();
                file.createNewFile();
            }
            Log.d("Debug", "Save()" + dataLabel);
            FileOutputStream outStream = new FileOutputStream(file, true);
            String str = String.valueOf(saveData);
            str = str.replaceAll(",","");
            str = str.substring(str.indexOf('[') + 1, str.lastIndexOf(']') - 1);
            outStream.write(str.getBytes());
            outStream.close();

            //System.out.println("New " + dataLabel + " Data Saved");

            /**
             * Call Upload()
             */

            Upload.upload(file_path, fileName, dataLabel);
            Log.d("Debug","Upload()" + dataLabel);
            //System.out.println("New " + dataLabel + " File Uploaded");

            } catch (Exception e) {
                e.printStackTrace();
            }


        }


    }
