package ie.ucd.smartrideRT;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

import com.pathsense.android.sdk.location.PathsenseGeofenceEvent;

public class PathsenseGeofenceDemoGeofenceEventReceiver extends BroadcastReceiver
{
    static final String TAG = PathsenseGeofenceDemoGeofenceEventReceiver.class.getName();

    @Override
    public void onReceive(Context context, Intent intent)
    {
        PathsenseGeofenceEvent geofenceEvent = PathsenseGeofenceEvent.fromIntent(intent);
        if (geofenceEvent != null)
        {
            if (geofenceEvent.isIngress())
            {
                // ingress
                // do something
                Log.i(TAG, "ENTERED");
            }
            else if (geofenceEvent.isEgress())
            {
                // egress
                // do something
                Log.i(TAG, "EXITED");
            }
        }
    }
}
