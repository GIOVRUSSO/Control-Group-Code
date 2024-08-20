package ie.ucd.smartrideRT;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.UUID;


import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.Binder;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;

import android.widget.Toast;

import com.google.android.gms.common.util.ArrayUtils;


/**
 * Service for managing connection and data communication with a GATT server hosted on a
 * given Bluetooth LE device.
 */

public class BLEService extends Service {
    private static byte[] testData = {1, 2, 3};
    private static final String tag = "debuggBLE";

    private Context mContext;
    private static final String TAG = BLEService.class.getSimpleName();// TAG


    private BluetoothManager mBluetoothManager = null;
    private BluetoothAdapter mBluetoothAdapter = null;
    private String mBluetoothDeviceAddress = null;
    private BluetoothGatt mBluetoothGatt = null;
    private BluetoothGattCharacteristic mCharacteristic = null;

    private ArrayList<BluetoothDevice> listDevice;
    private List<BluetoothGattService> serviceList;
    private List<BluetoothGattCharacteristic> characterList;

    MyDBHandler dbHandler;
    //bufferedReader bufferedReader;

    private Handler scanhandler;

    private static final boolean AUTO_CONNECT = true;
    private static final boolean NOTIFICATION_ENABLED = true;


    private int mConnectionState = STATE_DISCONNECTED;
    private static final int STATE_DISCONNECTED = 0;
    private static final int STATE_CONNECTING = 1;
    private static final int STATE_CONNECTED = 2;

    public final static String ACTION_GATT_CONNECTED =
            "com.example.bluetooth.le.ACTION_GATT_CONNECTED";
    public final static String ACTION_GATT_DISCONNECTED =
            "com.example.bluetooth.le.ACTION_GATT_DISCONNECTED";
    public final static String ACTION_GATT_SERVICES_DISCOVERED =
            "com.example.bluetooth.le.ACTION_GATT_SERVICES_DISCOVERED";
    public final static String ACTION_DATA_AVAILABLE =
            "com.example.bluetooth.le.ACTION_DATA_AVAILABLE";
    public final static String EXTRA_DATA =
            "com.example.bluetooth.le.EXTRA_DATA";

    // TODO insert your own UUID numbers
    public static final UUID Service_UUID = UUID.fromString("00003eb0-0000-1000-8000-00805f9b34fb");
    public static final UUID Characteristic_UUID = UUID.fromString("00003eb2-0000-1000-8000-00805f9b34fb");

    private boolean found = false;


    @Override
    public void onCreate() {
// TODO Auto-generated method stub
        super.onCreate();
        //initialize();
    }


    // Bluetooth is set up as a service which allows it to run in the background across multiple activities
    public class LocalBinder extends Binder {
        BLEService getService() {
            return BLEService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        initialize();
        //Get ability to pull data from database
        dbHandler = new MyDBHandler(this, null, null, 1);
        return mBinder;
    }

    @Override
    public boolean onUnbind(Intent intent) {
        // After using a given device, you should make sure that BluetoothGatt.close() is called
        // such that resources are cleaned up properly.  In this particular example, close() is
        // invoked when the UI is disconnected from the Service.
        close();
        return super.onUnbind(intent);
    }

    private final IBinder mBinder = new LocalBinder();

    /**
     * Initializes a reference to the local Bluetooth adapter.
     *
     * @return Return true if the initialization is successful.
     */

    public boolean initialize() {

        scanhandler = new Handler();
        Log.d("debuggg", "ble initiallize");

        // If the phone support BLE
        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            toast("BLE not support this device");
            ((Activity) mContext).finish(); // If not, turn off the app
        }
        // For API level 18 and above, get a reference to BluetoothAdapter through
        // BluetoothManager.
        if (mBluetoothManager == null) {
            mBluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
            if (mBluetoothManager == null) {
                Log.e(TAG, "Unable to initialize BluetoothManager.");
                return false;
            }
        }
        //turn on bluetooth
        mBluetoothAdapter = mBluetoothManager.getAdapter();
        if (mBluetoothAdapter == null || !mBluetoothAdapter.isEnabled()) {
            mBluetoothAdapter.enable();
            // request for turn on the Bluetooth
            //startActivityForResult(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE));
            //Log.e(TAG, "Unable to obtain a BluetoothAdapter.");
            //return false;
        }


        //Stop BLE devices scanning after 10 seconds
        scanhandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                mBluetoothAdapter.stopLeScan(new BluetoothAdapter.LeScanCallback() {
                    @Override
                    public void onLeScan(BluetoothDevice bluetoothDevice, int i, byte[] bytes) {

                    }
                });
            }
        }, 10000);

        // Scan for particular BLE devices
        mBluetoothAdapter.startLeScan(new BluetoothAdapter.LeScanCallback() {
            @Override
            public void onLeScan(BluetoothDevice bluetoothDevice, int i, byte[] bytes) {
//                Log.d("debuggg", bluetoothDevice.getAddress() + " " + bluetoothDevice.getName());
                if ("EBIKE".equals(bluetoothDevice.getName()) && !found) {
                    Intent intent = new Intent();
                    intent.setAction("ie.ucd.smartrideRT");
                    intent.putExtra("device", bluetoothDevice.getAddress());
                    sendBroadcast(intent);
                    connect(bluetoothDevice.getAddress());
                    mBluetoothAdapter.stopLeScan(null);
                    found = true;
                }
            }
        });

        return true;
    }


    @Override
    public void onDestroy() {

        super.onDestroy();
        suicide();
    }


    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {

        // if address not null then connect,otherwise let user choose the device

        if (mBluetoothDeviceAddress != null) {
            if (connect(mBluetoothDeviceAddress)) {
            } else {
            }
        } else {
            Log.d("debuggg", "onStartCommand: ");
            Intent bleIntent = new Intent();
            bleIntent.setClass(mContext, MainActivity.class);
            bleIntent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(bleIntent);
        }
        // return super.onStartCommand(intent, flags, startId);
        return Service.START_STICKY;
    }


    /**
     * Connects to the GATT server hosted on the Bluetooth LE device.
     *
     * @param address The device address of the destination device.
     * @return Return true if the connection is initiated successfully. The connection result
     * is reported asynchronously through the
     * {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     * callback.
     */
    private boolean connect(final String address) {
        if (mBluetoothAdapter == null || address == null) {
            Log.w(TAG, "BluetoothAdapter not initialized or unspecified address.");
            return false;
        }

        // Previously connected device.  Try to reconnect.
        if (mBluetoothDeviceAddress != null
                && address.equals(mBluetoothDeviceAddress)
                && mBluetoothGatt != null) {
            if (mBluetoothGatt.connect()) { // connect a device,change the connection state
                mConnectionState = STATE_CONNECTING;
                return true;
            } else {
                return false;
            }
        }


        final BluetoothDevice device = mBluetoothAdapter
                .getRemoteDevice(address);
        if (device == null) {
            Log.w(TAG, "Device not found.  Unable to connect.");
            toast("Device not connected");
            return false;
        }
        // We want to directly connect to the device, so we are setting the autoConnect
        // parameter to false.
        mBluetoothGatt = device.connectGatt(this, AUTO_CONNECT, mGattCallback);
        Log.d(TAG, "Trying to create a new connection.");
        //address of current connected device
        mBluetoothDeviceAddress = address;
        //state of connection
        mConnectionState = STATE_CONNECTING;
        return true;
    }


    /**
     * suicide
     */
    private void suicide() {
        disconnect();
        close();
        try {
            if (mBluetoothAdapter != null) {
                mBluetoothAdapter.disable();
                mBluetoothAdapter = null;
                mBluetoothManager = null;
            }
        } catch (Exception e) {
            toast("Please turn off the Bluetooth yourself");
        }
    }

    /**
     * Disconnects an existing connection or cancel a pending connection. The disconnection result
     * is reported asynchronously through the
     * {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     * callback.
     */
    private void disconnect() {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }
        mBluetoothGatt.disconnect();
    }


    /**
     * After using a given BLE device, the app must call this method to ensure resources are
     * released properly.
     */
    private void close() {
        if (mBluetoothGatt == null) {
            return;
        }
        mBluetoothGatt.close();
        mBluetoothGatt = null;
    }


    /**
     * GATT communication callback
     */
    // Implements callback methods for GATT events that the app cares about.  For example,
    // connection change and services discovered.

    private final BluetoothGattCallback mGattCallback = new BluetoothGattCallback() {

        // Call this method when the connection state changed
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status,
                                            int newState) {

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                if (mBluetoothGatt != null)
                    mBluetoothGatt.discoverServices(); //search the services support by the connected devices
                mConnectionState = STATE_CONNECTED;
                System.out.println("state connected");
                toast("Successfully connected to bike!");
                connectionBroadcast(true);
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                // connect(mBluetoothDeviceAddress);
                mConnectionState = STATE_DISCONNECTED;
                System.out.println("state disconnected");
                toast("failed to connect - please try again.");
                connectionBroadcast(false);
            }
        }

        public UUID convertFromInteger(int i) {
            final long MSB = 0x0000000000001000L;
            final long LSB = 0x800000805f9b34fbL;
            long value = i & 0xFFFFFFFF;
            return new UUID(MSB | (value << 32), LSB);
        }
        // Call this method when the BLE Services have been found
        // triggered by mBluetoothGatt.discoverServices()
        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            Log.d(TAG, "onServicesDiscovered");

            BluetoothGattService service = gatt.getService(convertFromInteger(0x180D));

            BluetoothGattCharacteristic characteristic = service.getCharacteristic(convertFromInteger(0x2A37));

            gatt.setCharacteristicNotification(characteristic, true);

            BluetoothGattDescriptor descriptor = characteristic.getDescriptor(convertFromInteger(0x2902));

            descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);

            gatt.writeDescriptor(descriptor);



            if  (true) return;

            Log.d(TAG, "onServicesDiscovered");

            toast("Bike Service Discovered！");
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d("debuggg2", "Gatt success");

                Log.d("debuggg2", "find operation characteristics");
                if (mBluetoothGatt != null) {
                    Log.d("debuggg2", "Gatt not null");
                    List<BluetoothGattService> services = mBluetoothGatt.getServices();
                    for (BluetoothGattService s : services) { // characteristics in services
                        Log.d("debuggg2", "service: " + s.getUuid());
                        for (BluetoothGattCharacteristic c : s.getCharacteristics()) {
                            Log.d("debuggg2", "\tchar: " + c.getUuid());
                        }
                    }
                    BluetoothGattService mGattService = mBluetoothGatt
                            .getService(Service_UUID);
                    mCharacteristic = mGattService
                            .getCharacteristic(Characteristic_UUID);
                    List<BluetoothGattDescriptor> mDescriptors = mCharacteristic
                            .getDescriptors();

                    Log.d("debuggg2", mCharacteristic.toString());
                    for (BluetoothGattDescriptor mDescriptor : mDescriptors) {
                        System.out.println(mDescriptor.getUuid().toString());
                        Log.d("debuggg2", "uuids: " + mDescriptor.getUuid().toString());
                    }

                    Log.d("debuggg2", mCharacteristic.toString());

                    setCharacteristicNotification(mCharacteristic,
                            NOTIFICATION_ENABLED);

//
//                    Log.d("debuggg2", "start write");
//                    wirteToBLE(testData);
//                    Log.d("debuggg2", "end write");
//                    String sss = new String(readFromBLE());
//                    System.out.println(sss);
//                    Log.d("debuggg2", sss);
//                    wirteToBLE(testData);
//                    System.out.println(new String(readFromBLE()));
                }
            } else { // no services been found
                Log.w(TAG, "onServicesDiscovered received:" + "status");
            }
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
            Log.d("debuggg2", descriptor + " " + status);
        }

        List<byte []> btyeArrays = new ArrayList<>();
        int count = 0;
        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt,
                                            BluetoothGattCharacteristic characteristic) {
            //broadcastUpdate(ACTION_DATA_AVAILABLE, characteristic);

                btyeArrays.add(characteristic.getValue());
                if (btyeArrays.size() == 5) {
                    byte[] fullPacket = ArrayUtils.concatByteArrays(btyeArrays.get(0), btyeArrays.get(1), btyeArrays.get(2), btyeArrays.get(3),btyeArrays.get(4));

                    writeToDatabase(fullPacket);

                    Log.d("debuggg2", new String(fullPacket).substring(0, 63));
                    btyeArrays.clear();
                    count++;
                }



        }

        // Callback of read Data
        @Override
        public void onCharacteristicRead(BluetoothGatt gatt,
                                         BluetoothGattCharacteristic characteristic, int status) {

            if (mBluetoothAdapter == null || mBluetoothGatt == null) {
                Log.w(TAG, "BluetoothAdapter not initialized");
                return;
            }
            Log.e(TAG, "onCharacteristicRead");
            Log.d("debuggg2", "onCharacteristicRead" + characteristic.getValue());
            super.onCharacteristicRead(gatt, characteristic, status);

        }

        // Callback of write Data
        @Override
        public void onCharacteristicWrite(BluetoothGatt gatt,
                                          BluetoothGattCharacteristic characteristic, int status) {

            Log.e(TAG, "onCharacteristicWrite");
            super.onCharacteristicWrite(gatt, characteristic, status);
        }

        //following code can be modified for your own requirement
        /*
        @Override
        public void onDescriptorRead(BluetoothGatt gatt,
                                     BluetoothGattDescriptor descriptor, int status) {

            super.onDescriptorRead(gatt, descriptor, status);
        }


        @Override
        public void onDescriptorWrite(BluetoothGatt gatt,
                                      BluetoothGattDescriptor descriptor, int status) {

            super.onDescriptorWrite(gatt, descriptor, status);
        }


        @Override
        public void onReadRemoteRssi(BluetoothGatt gatt, int rssi, int status) {

            super.onReadRemoteRssi(gatt, rssi, status);
        }


        @Override
        public void onReliableWriteCompleted(BluetoothGatt gatt, int status) {

            super.onReliableWriteCompleted(gatt, status);
        }
        */
    };

    /**
     * Enables or disables notification on a give characteristic.
     *
     * @param characteristic Characteristic to act on.
     * @param enabled        If true, enable notification.  False otherwise.
     */
    // It's common for BLE apps to ask to be notified when a particular characteristic changes
    // on the device.
    private void setCharacteristicNotification(BluetoothGattCharacteristic characteristic,
                                               boolean enabled) {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }
        mBluetoothGatt.setCharacteristicNotification(characteristic, enabled);

        //Get the features in the specified feature of the device, where it is monitored,
        // setCharacteristicNotification and the above callback onCharacteristicChanged one by one

    }


    /**
     * toast
     * @param text text will be shown on UI Screen
     */
    private void toast(String text) {
        //  Toast.makeText(mContext, text, Toast.LENGTH_SHORT).show();
        Intent intent = new Intent();
        intent.setAction("ie.ucd.smartrideRT.message");
        intent.putExtra("message", text);
        sendBroadcast(intent);
    }

    private void connectionBroadcast(boolean connected) {
        Intent intent = new Intent();
        intent.setAction("ie.ucd.smartrideRT.connection");
        intent.putExtra("connected", connected);
        sendBroadcast(intent);
    }

    /*
     * 
     * @return
     */
    /*
    public static BLEService self() {
        if (mContext != null)
            return (BLEService) mContext;
        return null;
    }
    */


    /**
     * Communication API, call this method for writing data to BLE device
     *  
     *
     * @param valueOut
     * @return
     */
    public boolean wirteToBLE(byte[] valueOut) {


        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return false;
        }

        String commandSent;
        commandSent = new String(valueOut);
        // Need to remove the '!' character from this string before entering in database
        commandSent = commandSent.replace("!", "");

        mCharacteristic.setValue(commandSent);

        CommandSentData commandSentData = new CommandSentData(commandSent);
        dbHandler.addCommandSentRow(commandSentData);

        boolean isSuccess = mBluetoothGatt.writeCharacteristic(mCharacteristic);
        return isSuccess;
    }


    /**
     * Receive data from BLE device
     *  
     *
     * @return valueIn
     */
    public byte[] readFromBLE() {


        byte[] valueIn = null;


        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return null;
        }

        boolean isSuccess = mBluetoothGatt.readCharacteristic(mCharacteristic);

        if (isSuccess) {
            Log.d("debuggg2", "read success" + " " + isSuccess);
            valueIn = mCharacteristic.getValue();

        }

        return valueIn;
    }

    /**
     * Database service API, send a broadcast to database service for storing the received data
     *
     * @return data
     */
     public void writeToDatabase(byte[] data) {
         Log.i(TAG, "save data to database");
         String s;

         s = new String(data, StandardCharsets.UTF_8);
         //s = s.substring(0, 63);
         String databaseEntry = s;
         Log.i(TAG, databaseEntry);
         Intent database_intent = new Intent();
         database_intent.addFlags(Intent.FLAG_INCLUDE_STOPPED_PACKAGES);
         database_intent.setAction("ie.ucd.smartrideRT.database");
         database_intent.putExtra("database", databaseEntry);
         sendBroadcast(database_intent);
         Log.i(TAG, "success sendBroadcast");
             /*
         } catch (Exception e) {

             e.printStackTrace();
             Log.i(TAG, "some kind of exception");
         }*/
    }

}