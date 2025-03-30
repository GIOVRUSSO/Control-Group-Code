package ie.ucd.smartrideRT;

import android.Manifest;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentSender;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.location.Location;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.provider.Settings;
import androidx.annotation.NonNull;
import com.google.android.material.snackbar.Snackbar;
import androidx.core.app.ActivityCompat;
import androidx.appcompat.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.common.api.ApiException;
import com.google.android.gms.common.api.ResolvableApiException;
import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationCallback;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationResult;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.location.LocationSettingsRequest;
import com.google.android.gms.location.LocationSettingsResponse;
import com.google.android.gms.location.LocationSettingsStatusCodes;
import com.google.android.gms.location.SettingsClient;
import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.gms.tasks.Task;

import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

import bikemessaging.SimpleLocation;

public class TrafficLightNudgingControl extends AppCompatActivity {
    // Has a handler for messages for the BluetoothService messages
    // (for both the OnBike bluetooth and the TrafficLight bluetooth connections)
    // Location
    private final String TAG = "TLNudging";

    // UI
    private TextView mLatitudeTextView;
    private TextView mLongitudeTextView;

    private String mLatitudeLabel;
    private String mLongitudeLabel;

    // Control algorithm
    private Timer controlTimer;


    // Location
    private static final int REQUEST_CHECK_SETTINGS = 0x1;
    private static final int REQUEST_PERMISSIONS_REQUEST_CODE = 34;
    private FusedLocationProviderClient mFusedLocationClient;
    private LocationRequest mLocationRequest;
    private LocationCallback mLocationCallback;
    private Location currentLocation;
    private boolean mRequestingLocationUpdates;
    private LocationSettingsRequest mLocationSettingsRequest;
    private SettingsClient mSettingsClient;

    public static final int TL_DATA = 1;

    // Bluetooth
    private BluetoothService bluetoothService;
    private boolean isBound;

    // Database
    MyDBHandler dbHandler;
    DatabaseService databaseService;

    Handler handler = new Handler(new IncomingHandlerCallback());

    /**
     * Class which is used to take messages from the message queue of this activity???
     */
    class IncomingHandlerCallback implements Handler.Callback {
        @Override
        public boolean handleMessage(Message msg) {
            switch (msg.what) {
                //CALORIES_DATA case is for when an updated assistance level should be sent to the bike
                //based on feedback
                case TL_DATA:
                    Integer requestToBikeInt = (Integer) msg.obj;
                    String requestToSendToBike = requestToBikeInt + "!";
                    //Log.i(TAG, "Request to send to bike is: " + requestToSendToBike); //TODO: remove
                    write(requestToSendToBike);
                    break;
                default:
                    Log.i(TAG, "PPC: default case");
                    break;
            }
            return true;
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_traffic_light_nudging_control);

        mLatitudeTextView = findViewById(R.id.latitude_text);
        mLongitudeTextView = findViewById(R.id.longitude_text);

        mLatitudeLabel = "Latitude";
        mLongitudeLabel = "Longitude";

        mLocationRequest = LocationRequest.create();
        mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);

        mRequestingLocationUpdates = true;

        mFusedLocationClient = LocationServices.getFusedLocationProviderClient(this);
        mSettingsClient = LocationServices.getSettingsClient(this);

        createLocationCallback();
        createLocationRequest();
        buildLocationSettingsRequest();

        if(mRequestingLocationUpdates && checkPermissions()){
            startLocationUpdates();
        }
        else if(!checkPermissions()){
            requestPermissions();
        }

        // Start service for bluetooth connection to use BluetoothService methods
        Intent i = new Intent(this, BluetoothService.class);
        bindService(i, bluetoothServiceConnection, Context.BIND_AUTO_CREATE);

        dbHandler = new MyDBHandler(this, null, null, 1);

        float samplingPeriod=6;

        // Start thread to start syncing data from bike
        Intent j = new Intent(this, DatabaseService.class);
        bindService(j, databaseServiceConnection, Context.BIND_AUTO_CREATE);

        Log.i(TAG, "About to start control task.");
        //start closed loop control with feedback
        controlTimer = new Timer();
        controlTimer.schedule(new NudgingControlTask(samplingPeriod), 0, (int)samplingPeriod);
    }

    @Override
    protected void onDestroy(){
        super.onDestroy();

        stopLocationUpdates();
    }

   private class NudgingControlTask extends TimerTask {
        private float samplingPeriod;
        private boolean rampingDown = false;
        private boolean isApproaching = false;
        private boolean isMovingAway = false;
        private TrafficLightData previousTrafficLightData;
        private float initialEncounteredDistance;

        public NudgingControlTask(float samplingPeriod){
            Log.i(TAG, "In nudging control task constructor");
            this.samplingPeriod = samplingPeriod;
        }

        public void run(){
            // Must look up the database each time
            TrafficLightData currentTrafficLightData = dbHandler.getLatestTrafficLightData();
            Log.i(TAG, "currentTrafficLightData is null?" + (currentTrafficLightData == null));

            SimpleLocation currentBikeLocation = dbHandler.getLatestBikeLocation();
            Log.i(TAG, "currentBikeLocation is null?" + (currentBikeLocation == null));

            // Need to check if the data is stale...

            // Need to check if we are moving away from the traffic light - need two objects to compare
            if(previousTrafficLightData == null || currentTrafficLightData == null){
                return;
            }

            // Are we approaching
            //if(previousDistanceToLight < currentDistanceToLight){
                // Moving away, so can increase the value to some preset, and disconnect from this light
                // Log.i(TAG, "Moving away from the light");
                // increaseMotorContribution();
                // return;
            }

            // Approaching - need to keep checking current light status
            //if(currentTrafficLightData.getTrafficLightStatus() == TrafficLightStatus.Red || currentTrafficLightData.getTrafficLightStatus() == TrafficLightStatus.Amber){
                // Need to initiate/continue rampdown
               // rampDownAssist(initialDistanceToLight, currentDistanceToLight);
            }

            // Check if the traffic light state warrants start up of ramp-down/continuation of ramp-down
            //if(mostRecentTrafficLightData != null){
                //controlBasedOnTrafficLightSignal(mostRecentTrafficLightData.getTrafficLightStatus());
//                if(mostRecentTrafficLightData.getTrafficLightStatus() == TrafficLightStatus.Amber ||
//                        mostRecentTrafficLightData.getTrafficLightStatus() == TrafficLightStatus.Red &&
//                        notMovingAway){
//                    if(rampingDown){
//                        // What parameters would this require?
//                        continueRampdown();
//                    }
//                    else {
//                        SimpleLocation trafficLightLocation = new SimpleLocation(mostRecentTrafficLightData.getLatitude(), mostRecentTrafficLightData.getLongitude());
//                        // Will need to check stale data using a timestamp at some point
//                        float currentSpeed = dbHandler.getLatestSpeed();
//                        initialEncounteredDistance = getDistance(currentBikeLocation, trafficLightLocation);
//                        initiateRampdown(currentBikeLocation, trafficLightLocation, currentSpeed);
//                        rampingDown = true;
//                    }
//                }
//                else {
//                    // Want to let the user have full control over the throttle level now,
//                    // but must ramp up to whatever value they are using
//                    rampingDown = false;
//                }

                //previousTrafficLightData = currentTrafficLightData;
            //}
        //}

        private void controlBasedOnTrafficLightSignal(TrafficLightStatus currentTrafficLightStatus){
            switch(currentTrafficLightStatus){
                case Red:
                    handler.obtainMessage(TL_DATA, 80).sendToTarget();
                    break;
                case Amber:
                    handler.obtainMessage(TL_DATA, 100).sendToTarget();
                    break;
                case Green:
                    handler.obtainMessage(TL_DATA, 120).sendToTarget();
                    break;
            }
        }

        private void initiateRampdown(SimpleLocation currentBikeLocation, SimpleLocation trafficLightLocation, float currentSpeed){
            int controlValInt = 100;

            String controlValueString = Integer.toString(controlValInt) + "!";

            //write(controlValueString);
        }

        private void continueRampdown(){
            int controlValInt = 100;

            String controlValueString = Integer.toString(controlValInt) + "!";

            //write(controlValueString);

            // Record the command sent to the arduino
            dbHandler.addCommandSentRow(new CommandSentData(controlValueString));
        }

        private float getDistance(SimpleLocation bikeLoc, SimpleLocation tlLoc){
            final float radiusOfEarthMetres = 6371*10^3;

            double bikeLatitudeRadian = Math.toRadians(bikeLoc.getLatitude());
            double bikeLongitudeRadian = Math.toRadians(bikeLoc.getLongitude());
            double trafficLightLatitudeRadian = Math.toRadians(tlLoc.getLatitude());
            double trafficLightLongitudeRadian = Math.toRadians(tlLoc.getLongitude());

            double latitudeDifference = bikeLatitudeRadian - trafficLightLatitudeRadian;
            double longitudeDifference = bikeLongitudeRadian - trafficLightLongitudeRadian;

            double a = (Math.sin(latitudeDifference/2)*Math.sin(latitudeDifference/2))+ Math.cos(bikeLatitudeRadian)* Math.cos(trafficLightLatitudeRadian)*(Math.sin(longitudeDifference/2)*Math.sin(longitudeDifference/2));
            double c = 2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

            float distance = (float)(radiusOfEarthMetres*c);

            return distance;
        }
   // }

    // LOCATION //
    private void createLocationRequest(){
        mLocationRequest = new LocationRequest();

        mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
    }

    private void createLocationCallback(){
        mLocationCallback = new LocationCallback(){
            @Override
            public void onLocationResult(LocationResult locationResult){
                super.onLocationResult(locationResult);

                currentLocation = locationResult.getLastLocation();

                // Want to write the location to a database table
                dbHandler.addBikeLocation((float)currentLocation.getLatitude(), (float)currentLocation.getLongitude());

                updateLocationUI();
            }
        };
    }

    private void buildLocationSettingsRequest(){
        LocationSettingsRequest.Builder builder = new LocationSettingsRequest.Builder();
        builder.addLocationRequest(mLocationRequest);
        mLocationSettingsRequest = builder.build();
    }
/*
    protected void onActivityResult(int requestCode, int resultCode, Intent data){
        switch (requestCode){
            case REQUEST_CHECK_SETTINGS:
                switch (resultCode) {
                    case Activity.RESULT_OK:
                        Log.i(TAG, "User agreed to make required location settings changes.");
                        // Nothing to do. startLocationUpdates() gets called in onResume() again
                        break;
                    case Activity.RESULT_CANCELED:
                        Log.i(TAG, "User chose not to make required location settings changes.");
                        mRequestingLocationUpdates = false;
                        break;
                }
                break;
        }
    }
*/
    /**
     * Requests location updates from the FusedLocationApi. Note: we don't call this unless location runtime
     * permission has been granted.
     */
    private void startLocationUpdates(){
        mSettingsClient.checkLocationSettings(mLocationSettingsRequest)
                .addOnSuccessListener(this, new OnSuccessListener<LocationSettingsResponse>() {
                    @Override
                    public void onSuccess(LocationSettingsResponse locationSettingsResponse) {
                        Log.i(TAG, "All location settings are satisfied.");

                        //TODO: remove comment
                        // noinspection MissingPermission
                        //FusedLocationClient.requestLocationUpdates(mLocationRequest, mLocationCallback, Looper.myLooper());
                    }
                })
                .addOnFailureListener(this, new OnFailureListener() {
                    @Override
                    public void onFailure(@NonNull Exception e) {
                        int statusCode = ((ApiException) e).getStatusCode();

                        switch(statusCode){
                            case LocationSettingsStatusCodes
                                    .RESOLUTION_REQUIRED:
                                Log.i(TAG, "Location settings are not satisfied. Attempting to upgrade " +
                                        "location settings.");
                                try{
                                    // Show the dialog by calling startResolutionForResult(), and check the
                                    // result in onActivityResult().
                                    ResolvableApiException rae = (ResolvableApiException) e;
                                    rae.startResolutionForResult(TrafficLightNudgingControl.this, REQUEST_CHECK_SETTINGS);
                                }
                                catch(IntentSender.SendIntentException sie){
                                    Log.i(TAG, "PendingIntent unable to execute request.");
                                }
                                break;
                            case LocationSettingsStatusCodes.SETTINGS_CHANGE_UNAVAILABLE:
                                String errorMessage = "Location settings are inadequate, and cannot be " +
                                        "fixed here. Fix in Settings.";
                                Log.e(TAG, errorMessage);
                                Toast.makeText(TrafficLightNudgingControl.this, errorMessage, Toast.LENGTH_LONG).show();
                                mRequestingLocationUpdates = false;
                                break;
                        }
                    }
                });
    }

    public void stopLocationUpdates(){
        if(!mRequestingLocationUpdates){
            Log.d(TAG, "stopLocationUpdates: updates never requested, no-op.");
            return;
        }

        // It is a good practice to remove location requests when the activity is in a paused or
        // stopped state. Doing so helps battery performance and is especially
        // recommended in applications that request frequent location updates.
        mFusedLocationClient.removeLocationUpdates(mLocationCallback)
                .addOnCompleteListener(this, new OnCompleteListener<Void>() {
                    @Override
                    public void onComplete(@NonNull Task<Void> task) {
                        mRequestingLocationUpdates = false;
                    }
                });
    }

    private boolean checkPermissions(){
        int permissionState = ActivityCompat.checkSelfPermission(this,
                Manifest.permission.ACCESS_FINE_LOCATION);
        return permissionState == PackageManager.PERMISSION_GRANTED;
    }

    private void showSnackbar(final int mainTextStringId, final int actionStringId,
                              View.OnClickListener listener) {
        Snackbar.make(
                findViewById(android.R.id.content),
                getString(mainTextStringId),
                Snackbar.LENGTH_INDEFINITE)
                .setAction(getString(actionStringId), listener).show();
    }

    private void requestPermissions(){
        boolean shouldProvideRationale = ActivityCompat.shouldShowRequestPermissionRationale(this,
                Manifest.permission.ACCESS_FINE_LOCATION);

        // Provide an additional rationale to the user. This would happen if the user denied the
        // request previously, but didn't check the "Don't ask again" checkbox.
        if(shouldProvideRationale){
            Log.i(TAG, "Displaying permission rationale to provide additional context.");
            showSnackbar(R.string.permission_rationale,
                    android.R.string.ok, new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            // Request permission
                            ActivityCompat.requestPermissions(TrafficLightNudgingControl.this,
                                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                                    REQUEST_PERMISSIONS_REQUEST_CODE);
                        }
                    });
        } else {
            Log.i(TAG, "Requesting permission");
            // Request permission. It's possible this can be auto answered if device policy
            // sets the permission in a given state or the user denied the permission
            // previously and checked "Never ask again".
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                    REQUEST_PERMISSIONS_REQUEST_CODE);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        Log.i(TAG, "onRequestPermissionResult");
        if (requestCode == REQUEST_PERMISSIONS_REQUEST_CODE) {
            if (grantResults.length <= 0) {
                // If user interaction was interrupted, the permission request is cancelled and you
                // receive empty arrays.
                Log.i(TAG, "User interaction was cancelled.");
            } else if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                if (mRequestingLocationUpdates) {
                    Log.i(TAG, "Permission granted, updates requested, starting location updates");
                    startLocationUpdates();
                }
            } else {
                // Permission denied.

                // Notify the user via a SnackBar that they have rejected a core permission for the
                // app, which makes the Activity useless. In a real app, core permissions would
                // typically be best requested during a welcome-screen flow.

                // Additionally, it is important to remember that a permission might have been
                // rejected without asking the user for permission (device policy or "Never ask
                // again" prompts). Therefore, a user interface affordance is typically implemented
                // when permissions are denied. Otherwise, your app could appear unresponsive to
                // touches or interactions which have required permissions.
                showSnackbar(R.string.permission_denied_explanation,
                        R.string.settings, new View.OnClickListener() {
                            @Override
                            public void onClick(View view) {
                                // Build intent that displays the App settings screen.
                                Intent intent = new Intent();
                                intent.setAction(
                                        Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                                Uri uri = Uri.fromParts("package",
                                        BuildConfig.APPLICATION_ID, null);
                                intent.setData(uri);
                                intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
                                startActivity(intent);
                            }
                        });
            }
        }
    }

    // END LOCATION //

    // BLUETOOTH //

    /**
     * Write to the RNBT-40 bluetooth device on the bike
     * @param message
     * The command to be sent over the Bluetooth channel
     */
    private void write(String message){

        if (message.length() > 0) {
            byte[] send = message.getBytes();
            bluetoothService.write(send);
        }
    }

    // bluetoothServiceConnection is needed to use BluetoothService methods
    private ServiceConnection bluetoothServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            BluetoothService.BluetoothMyLocalBinder binder  = (BluetoothService.BluetoothMyLocalBinder) service;
            bluetoothService = binder.getService();
            isBound = true;
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            isBound = false;
        }
    };

    // DATABASE //

    //databaseServiceConnection is needed to use DatabaseService methods
    private ServiceConnection databaseServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            DatabaseService.DatabaseMyLocalBinder binder  = (DatabaseService.DatabaseMyLocalBinder) service;
            databaseService = binder.getService();
        }

        @Override
        public void onServiceDisconnected(ComponentName name){

        }
    };

    // END DATABASE //

    private void updateLocationUI(){
        if (currentLocation != null) {
            mLatitudeTextView.setText(String.format(Locale.ENGLISH, "%s: %f", mLatitudeLabel,
                    currentLocation.getLatitude()));
            mLongitudeTextView.setText(String.format(Locale.ENGLISH, "%s: %f", mLongitudeLabel,
                    currentLocation.getLongitude()));
        }
        else{
            Log.i(TAG, "Location is null");
        }
    }
}
