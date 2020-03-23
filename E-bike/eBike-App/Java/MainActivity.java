/*
* Class Name: MainActivity.java
* Corresponding layout: activity_main.xml
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
* Description: MainActivity is the launcher activity. It displays a combined list of bluetooth devices
* that are within range and devices that have already been paired with on the home screen. It also
* has a number of buttons which are used to launch other activities.
* */


package ie.ucd.smartrideRT;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.provider.Settings;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends Activity implements OnItemClickListener {

    private static final String tag = "MADebugging";

    //private LeDeviceListAdapter mLeDeviceListAdapter;
    private BluetoothAdapter mBluetoothAdapter;
    private boolean mScanning;
    private Handler mHandler;

    private static Context mContext;// 上下文

    private static final int REQUEST_ENABLE_BT = 1;

    private static final int REQUEST_PERMISSION = 1;
    // Stops scanning after 10 seconds.
    private static final long SCAN_PERIOD = 10000;
    private ListView mListView = null;
    private BLEAdapter mAdapter = null;


    public static BLEService mBLEService;
    DatabaseService mdatabaseService;
    boolean bluetoothIsBound=false;
    ArrayAdapter<String> activityListAdapter;
    ListView listView;
    IntentFilter filter;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        /**
         * Permissions
         */
        if (checkAndRequestPermissions()) {
            // carry on the normal flow, as the case of  permissions  granted.
        }

        init();

        /**
         * Start service for BLE connection to retrieve BLE devices
         */
        //start service for bluetooth connection to retrieve bluetooth devices
        Intent i = new Intent(this, BLEService.class);
        bindService(i, BLEServiceConnection, Context.BIND_AUTO_CREATE);

        //start thread to start syncing data from bike
        Intent j = new Intent(this, DatabaseService.class);
        bindService(j, databaseServiceConnection, Context.BIND_AUTO_CREATE);
        //register broadcast receiver to produce list of all available devices for Bluetooth connection
        registerDeviceReceiver();
    }
        /**
         * Permissions Request
         */

        String[] permissions = new String[]{Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.WRITE_EXTERNAL_STORAGE,
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.READ_PHONE_STATE
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
                } else {
                    // Permission denied, boo!
                    final boolean needRationale1 = ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.WRITE_EXTERNAL_STORAGE);
                    new AlertDialog.Builder(this)
                            .setTitle("Permission required")
                            .setMessage("Without this permission application cannot be started, allow it in order to start the sppllication.")
                            .setPositiveButton("Retry", new DialogInterface.OnClickListener() {
                                public void onClick(DialogInterface dialog, int which) {
                                    // try again
                                    if (needRationale1) {
                                        // the "never ask again" flash is not set, try again with permission request
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
    }
    //init initialises variables especially the Adapter to store all devices found by Bluetooth
    private void init() {
        listView = (ListView) findViewById(R.id.listView);
        listView.setOnItemClickListener(this);
        activityListAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1, 0);
        listView.setAdapter(activityListAdapter);
    }

    //this method registers the BroadcastReceiver to listen to BluetoothService for the devices
    // that are available/paired so they can be printed to the screen for user selection
    private void registerDeviceReceiver() {
        filter = new IntentFilter("ie.ucd.smartrideRT");
        registerReceiver(MyReceiver, filter);
        filter = new IntentFilter("ie.ucd.smartrideRT.message");
        registerReceiver(stateReceiver, filter);
        filter = new IntentFilter("ie.ucd.smartrideRT.connection");
        registerReceiver(connectionReceiver, filter);
    }

    //BroadcastReceiver listens for available devices from BluetoothService
    private final BroadcastReceiver MyReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            Log.d("debuggg", "bt onreceive");
            String deviceDataReceived = intent.getStringExtra("device");
            if (activityListAdapter.getCount() == 0 ||!activityListAdapter.getItem(0).equals(deviceDataReceived))
                activityListAdapter.add(deviceDataReceived);
        }
    };
    // Toast for indicate state
    private final BroadcastReceiver stateReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            String message = intent.getStringExtra("message");
            Toast.makeText(MainActivity.this, message, Toast.LENGTH_SHORT).show();
        }
    };

    private final BroadcastReceiver connectionReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            boolean connected = intent.getBooleanExtra("connected", false);
            TextView tv = findViewById(R.id.tvConnection);
            tv.setText(connected ? "c" : "d");
        }
    };

    //bluetoothServiceConnection is required for Bluetooth to work in the background
    private ServiceConnection BLEServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            mBLEService = ((BLEService.LocalBinder) service).getService();
            bluetoothIsBound = true;
        }

        @Override
        public void onServiceDisconnected(ComponentName name){
            mBLEService = null;
            bluetoothIsBound = false;
        }

    };

    //databaseServiceConnection is needed to use DatabaseService methods
    private ServiceConnection databaseServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            DatabaseService.DatabaseMyLocalBinder binder  = (DatabaseService.DatabaseMyLocalBinder) service;
            mdatabaseService = binder.getService();
        }

        @Override
        public void onServiceDisconnected(ComponentName name){

        }
    };


//    @Override
//    public void onDestroy(){
//        super.onDestroy();
//
//        //bluetoothService.onDestroy();
//    }

    //method to manage what happens when user clicks one of the devices that have been found by bluetooth
    public void onItemClick(AdapterView<?> arg0, View arg1, int arg2,
                            long arg3) {
        //mBLEService.tryRead();
//
//        if (activityListAdapter.getItem(arg2).contains("Paired")) {
//            //Log.i(tag, "Checking if device " + activityListAdapter.getItem(arg2) + " is paired.");
//            // Take the address out of the string
//            String deviceAddress = activityListAdapter.getItem(arg2).split("[\\r\\n]")[1];
//            Log.i(tag, "Device address clicked is " + deviceAddress);
//
//            //bluetoothService.connectToPairedDevice(deviceAddress);
//            bluetoothService.checkifPaired(arg2);
//        } else {
//            Toast.makeText(getApplicationContext(), "device is not paired", Toast.LENGTH_SHORT).show();
//        }
    }

    /* The following methods launch different activities depending on the user selection*/

    // Method to launch activity to send manual command to bike
    public void goToSendCommand(View view){
        Log.i(tag, "About to launch SendCommand");
        Intent i = new Intent(this, SendCommand.class);
        startActivity(i);
    }




/*
    // Method to launch activity to view data saved in database
    public void goToDb(View view){
        Log.i(tag, "Launching database activity");
        Intent j = new Intent(this, ViewData.class);
        startActivity(j);
    }

 */

}