package ie.ucd.smartrideRT;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import androidx.appcompat.app.AppCompatActivity;
import android.util.Log;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.ToggleButton;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.LimitLine;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.pathsense.android.sdk.location.PathsenseLocationProviderApi;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public class CooperativeCompetitiveControl extends AppCompatActivity {
    public static final int COOP_COMPETE_DATA = 1;
    public static final int PLOTTING_DATA = 2;
    private final String TAG = "CCControl";
    private int lastRequestToMotor = 0;

    // UI
    // Plotting
    LineChart cooperationCharacteristicChart;
    LineChart mValueChart;
    TextView humanOutputPowerTextView;
    TextView motorReferencePowerTextView;
    TextView motorActualPowerTextView;
    boolean plottingEnabled = true;

    ToggleButton toggleCoopPlotEnable;
    ToggleButton toggleMValuePlotEnable;
    boolean plotCoopEnable;
    boolean plotMValEnable;

    // Changing m functionality
    final float INITIAL_M_TEST_VALUE = (float)0.4;
    final float FINAL_M_TEST_VALUE = (float)0.6;
    final float TRANSITION_TIME = 5; // seconds

    Switch changeMSwitch;
    boolean changeMEnabled;
    ToggleButton toggleMStarButton;
    boolean mStarToggleButtonEnabled;
    boolean toggleValueUpper;
    float mStar = INITIAL_M_TEST_VALUE;

    private LineDataSet mTargetDataSet;
    private LineDataSet mActualDataSet;

    // Control algorithm
    private int samplingPeriod = 1100; // milliseconds
    private Timer controlTimer;

    // BluetoothService
    BluetoothService bluetoothService;
    private boolean isBound;

    PathsenseLocationProviderApi api;
    PathsenseGeofenceDemoGeofenceEventReceiver geofenceEventReceiver;

    // Database
    MyDBHandler dbHandler;
    DatabaseService databaseService;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_cooperative_competitive_control);

        // Start service for bluetooth connection to use BluetoothService methods
        Intent i = new Intent(this, BluetoothService.class);
        bindService(i, bluetoothServiceConnection, Context.BIND_AUTO_CREATE);

        dbHandler = new MyDBHandler(this, null, null, 1);

        // Start thread to start syncing data from bike
        Intent j = new Intent(this, DatabaseService.class);
        bindService(j, databaseServiceConnection, Context.BIND_AUTO_CREATE);

        api = PathsenseLocationProviderApi.getInstance(this);
        api.addGeofence("MYGEOFENCE", 53.291377, -6.179869, 100, PathsenseGeofenceDemoGeofenceEventReceiver.class);

        // Start closed loop control with feedback
        controlTimer = new Timer();
        controlTimer.schedule(new CooperativeCompetitiveControlTask(samplingPeriod), 0, samplingPeriod);

        // Initialise the charts in the activity
        cooperationCharacteristicChart = findViewById(R.id.cooperationCharacteristicChart);

        configureButtons();
        configureCharacteristicGraph();
        configureMValueGraph();
    }

    @Override
    protected void onPause() {
        plottingEnabled = false;

        controlTimer.cancel();

        super.onPause();
    }

    @Override
    protected void onResume() {
        plottingEnabled = true;

        controlTimer.schedule(new CooperativeCompetitiveControlTask(samplingPeriod), 0, samplingPeriod);

        super.onResume();
    }

    /**
     * Method to configure the layout of the cooperation characteristic graph
     */
    private void configureCharacteristicGraph() {
        cooperationCharacteristicChart = findViewById(R.id.cooperationCharacteristicChart);
        cooperationCharacteristicChart.setPinchZoom(true);
        cooperationCharacteristicChart.setBackgroundColor(Color.LTGRAY);
        cooperationCharacteristicChart.getAxisRight().setEnabled(false);
        cooperationCharacteristicChart.setDescription(null);

        YAxis yAxisLeft = cooperationCharacteristicChart.getAxisLeft();
        yAxisLeft.setAxisMaximum(400f);
        yAxisLeft.setAxisMinimum(0f);
        yAxisLeft.setLabelCount(10);
        yAxisLeft.setDrawAxisLine(true);

        XAxis xAxis = cooperationCharacteristicChart.getXAxis();
        xAxis.setDrawAxisLine(true);
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setAxisMaximum(400f);
        xAxis.setAxisMinimum(0f);
        xAxis.setLabelCount(8);
        xAxis.setDrawLabels(true);

        Legend mValueLegend = cooperationCharacteristicChart.getLegend();
        mValueLegend.setWordWrapEnabled(true);
        mValueLegend.setPosition(Legend.LegendPosition.ABOVE_CHART_RIGHT);
        mValueLegend.setForm(Legend.LegendForm.SQUARE);
    }

    /**
     * Method to configure the layout of the m-value graphs
     */
    private void configureMValueGraph() {
        humanOutputPowerTextView = findViewById(R.id.humanOutputPowerTextView);
        humanOutputPowerTextView.setText("Human output: none");
        humanOutputPowerTextView.setTextColor(Color.BLACK);

        motorReferencePowerTextView = findViewById(R.id.motorReferenceOutput);
        motorReferencePowerTextView.setText("Motor reference: none");
        motorReferencePowerTextView.setTextColor(Color.BLACK);

        motorActualPowerTextView = findViewById(R.id.motorActualOutput);
        motorActualPowerTextView.setText("Motor actual: none");
        motorActualPowerTextView.setTextColor(Color.BLACK);

        mValueChart = findViewById(R.id.mValueChart);
        mValueChart.setPinchZoom(true);
        mValueChart.getAxisRight().setEnabled(false);
        mValueChart.setBackgroundColor(Color.LTGRAY);
        mValueChart.setDescription(null);

        YAxis yAxisLeft = mValueChart.getAxisLeft();
        yAxisLeft.setAxisMaximum(1f);
        yAxisLeft.setAxisMinimum(0f);
        yAxisLeft.setLabelCount(5);
        yAxisLeft.setDrawAxisLine(true);

        XAxis xAxis = mValueChart.getXAxis();
        xAxis.setDrawAxisLine(true);
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setAxisMaximum(20f);
        xAxis.setAxisMinimum(0f);
        xAxis.setDrawLabels(true);
        xAxis.setLabelCount(5);

        Legend mValueLegend = mValueChart.getLegend();
        mValueLegend.setPosition(Legend.LegendPosition.ABOVE_CHART_RIGHT);
        mValueLegend.setForm(Legend.LegendForm.SQUARE);

        mTargetDataSet = new LineDataSet(null, "mTarget");
        mTargetDataSet.setColor(Color.CYAN);
        mTargetDataSet.setCircleColor(Color.CYAN);
        mTargetDataSet.setCircleRadius(1f);
        mTargetDataSet.setValueTextColor(Color.TRANSPARENT);

        mActualDataSet = new LineDataSet(null, "mActual");
        mActualDataSet.setColor(Color.BLUE);
        mActualDataSet.setCircleColor(Color.BLUE);
        mActualDataSet.setCircleRadius(1f);
        mActualDataSet.setValueTextColor(Color.TRANSPARENT);
    }

    /**
     * Method to wire up buttons
     */
    private void configureButtons() {
        toggleCoopPlotEnable = findViewById(R.id.coopPlotToggle);
        plotCoopEnable = toggleCoopPlotEnable.isChecked();
        toggleCoopPlotEnable.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    plotCoopEnable = true;
                } else {
                    plotCoopEnable = false;
                }
            }
        });

        toggleMValuePlotEnable = findViewById(R.id.mPlotToggle);
        plotMValEnable = toggleMValuePlotEnable.isChecked();
        toggleMValuePlotEnable.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                plotMValEnable = isChecked;
            }
        });

        changeMSwitch = findViewById(R.id.changeMSwitch);
        changeMEnabled = changeMSwitch.isChecked();
        changeMSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                changeMEnabled = isChecked;
                toggleMStarButton.setEnabled(isChecked);
            }
        });

        toggleMStarButton = findViewById(R.id.mStarValueToggle);
        toggleMStarButton.setEnabled(false);
        toggleMStarButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                toggleValueUpper = isChecked;

                if(!isChecked){
                    mStar = INITIAL_M_TEST_VALUE;
                }
            }
        });
    }

    /**
     * Method which takes in data to plot, and appends the new
     * data to the graph, pushing out older values as necessary
     *
     * @param plottingData Class containing the relevant data values that
     *                     are desired to be displayed
     */
    private void updateUIPlots(PlottingData plottingData) {
        if (plotMValEnable) {
            humanOutputPowerTextView.setText("Human output: " + plottingData.getRecentHumanPowerAveraged());
            motorReferencePowerTextView.setText("Motor reference: " + plottingData.getMotorReferencePower());
            motorActualPowerTextView.setText("Motor actual: " + plottingData.getMotorActualPower());

            if (mActualDataSet.getEntryCount() < 20) {
                mActualDataSet.addEntry(new Entry(19, plottingData.getRecentMActual().getMValue()));
                mTargetDataSet.addEntry(new Entry(19, plottingData.getRecentMTarget()));

            } else {
                mActualDataSet.addEntry(new Entry(19, plottingData.getRecentMActual().getMValue()));
                mActualDataSet.removeFirst();

                mTargetDataSet.addEntry(new Entry(19, plottingData.getRecentMTarget()));
                mTargetDataSet.removeFirst();
            }

            // Update the rest
            for (int i = mActualDataSet.getEntryCount() - 1, j = 1; i > 0; i--) {
                mActualDataSet.getEntryForIndex(i - 1).setX(19f - j);
                mTargetDataSet.getEntryForIndex(i - 1).setX(19f - j);
                j++;
            }

            LineData data = new LineData(mActualDataSet, mTargetDataSet);

            mValueChart.setData(data);
            mValueChart.notifyDataSetChanged();
            mValueChart.invalidate();
        }

        // Characteristic plot
        if (plotCoopEnable) {
            List<Entry> characteristicEntries = new ArrayList<>();

            int sizeOfPlottingData = plottingData.getOutputOfCharacteristic().size();

            for (int i = 0; i < sizeOfPlottingData; i++) {
                characteristicEntries.add(new Entry(plottingData.getInputToCharacteristic().get(i), plottingData.getOutputOfCharacteristic().get(i)));
            }

            LineDataSet characteristicDataSet = new LineDataSet(characteristicEntries, "");
            characteristicDataSet.setColor(Color.BLUE);
            characteristicDataSet.setValueTextColor(Color.TRANSPARENT);
            characteristicDataSet.setCircleRadius(1f);
            characteristicDataSet.setCircleColor(Color.BLUE);
            characteristicDataSet.setCubicIntensity(1f);
            characteristicDataSet.setDrawFilled(true);

            LimitLine humanPowerLimitLine = new LimitLine(plottingData.getRecentHumanPowerAveraged(), "");
            humanPowerLimitLine.setLineColor(Color.RED);
            humanPowerLimitLine.setLineWidth(1f);

            cooperationCharacteristicChart.getXAxis().removeAllLimitLines();
            cooperationCharacteristicChart.getXAxis().addLimitLine(humanPowerLimitLine);

            LineData characteristicLineData = new LineData(characteristicDataSet);
            cooperationCharacteristicChart.setData(characteristicLineData);
            cooperationCharacteristicChart.invalidate();
        }
    }

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
                case COOP_COMPETE_DATA:
                    Integer requestToBikeInt = (Integer) msg.obj;
                    String requestToSendToBike = requestToBikeInt + "!";
                    //Log.i(TAG, "Request to send to bike is: " + requestToSendToBike); // Useful for debugging
                    write(requestToSendToBike);
                    break;
                case PLOTTING_DATA:
                    if (plottingEnabled) {
                        PlottingData plottingData = (PlottingData) msg.obj;
                        updateUIPlots(plottingData);
                    }
                    break;
                default:
                    Log.i(TAG, "PPC: default case");
                    break;
            }
            return true;
        }
    }

    /**
     * Used to send message to BluetoothService for it to be sent by Bluetooth
     */
    private void write(String message) {
        if (message.length() > 0) {
            byte[] send = message.getBytes();
            bluetoothService.write(send);
        }
    }

    /**
     * Scheduled task used to control bike
     */
    private class CooperativeCompetitiveControlTask extends TimerTask {
        private float samplingPeriod;
        private float motorEfficiency = 0.8f;
        private float torqueBias = 44.5f;
        private float cranksetEfficiency = 0.9f;
        private float humanPowerFactor = 1.5f;
        private int windowSize = 1;
        private float pollutionLevel = 0.9f;
        private final int numEntriesRequired = 20; // Take the most recent 20 plotting table rows to draw on graph
        private final int maxPlottedWatts = 400;
        private ArrayList<Float> pollutionLevelArray;
        private final float LOW_POLLUTION_LEVEL = (float)0.2;
        private final float MID_POLLUTION_LEVEL = (float)0.4;
        private final float HIGH_POLLUTION_LEVEL = (float)0.6;
        private float mStar = (float)0.5;
        private int timestep;

        public CooperativeCompetitiveControlTask(float samplingPeriod) {
            this.samplingPeriod = samplingPeriod;

            pollutionLevelArray = new ArrayList<>();

            // Add the constant values to the start of the array
            // Code for simple sigmoid from high -> mid -> low pollution level and back
            for(int i = 0;i<40;i++){
                pollutionLevelArray.add(HIGH_POLLUTION_LEVEL);
            }

            float valueToAdd;

            for(int i = 0;i<50;i++){
                valueToAdd = (float)(HIGH_POLLUTION_LEVEL - (HIGH_POLLUTION_LEVEL - MID_POLLUTION_LEVEL)*(1/(1+Math.exp(-(-9+(18*i)/50.0)))));
                pollutionLevelArray.add(valueToAdd);
            }

            for(int i = 0;i<25;i++){
                pollutionLevelArray.add(MID_POLLUTION_LEVEL);
            }

            for(int i = 0;i<50;i++){
                valueToAdd = (float)(MID_POLLUTION_LEVEL - (MID_POLLUTION_LEVEL - LOW_POLLUTION_LEVEL)*(1/(1+Math.exp(-(-9+(18*i)/50.0)))));
                pollutionLevelArray.add(valueToAdd);
            }

            for(int i = 0;i<15;i++){
                pollutionLevelArray.add(LOW_POLLUTION_LEVEL);
            }

            ArrayList<Float> reversedArray = new ArrayList<>(pollutionLevelArray);
            Collections.reverse(reversedArray);

            pollutionLevelArray.addAll(reversedArray);
            
            timestep = 0;
        }

        public void run() {
            // Query the database for the required values
            List<float[]> bikeDataList = dbHandler.getRecentBikeData(motorEfficiency, torqueBias, cranksetEfficiency, humanPowerFactor, windowSize);

            if (bikeDataList != null) {
                float[] motorOutputPowerArray = bikeDataList.get(0);
                float[] humanOutputPowerArray = bikeDataList.get(1);

                float recentMotorOutputPowerAveraged = calculateArrayAverage(motorOutputPowerArray, motorOutputPowerArray.length);
                float recentHumanOutputPowerAveraged = calculateArrayAverage(humanOutputPowerArray, humanOutputPowerArray.length);

                float usingSamplingPeriod = (float) (samplingPeriod / 1000.0);

                if(recentHumanOutputPowerAveraged > 0 && recentMotorOutputPowerAveraged < 10){
                    // Generate dummy motor value to prevent the dead scenario
                    recentMotorOutputPowerAveraged = 10;
                }

                float motorOutputPowerReference;
                float mTarget = 0;

                if(changeMEnabled){
                    // Pre-calculate mStar based on the pollution level, use simple mapping in this case
                    mStar = 1 - pollutionLevelArray.get(timestep);

                    motorOutputPowerReference = computeNextOutputMotorPowerReference(recentHumanOutputPowerAveraged,
                            recentMotorOutputPowerAveraged,
                            pollutionLevelArray.get(timestep),
                            usingSamplingPeriod,
                            mStar);

                    // Threshold is a constant value for the moment
                    if(recentHumanOutputPowerAveraged <= 150){
                        mTarget = mStar;
                    }
                    else{
                        mTarget = recentHumanOutputPowerAveraged /(recentHumanOutputPowerAveraged + motorOutputPowerReference);
                    }

                    if(timestep < pollutionLevelArray.size() - 2){
                        timestep++;
                    }
                }
                else {
                    motorOutputPowerReference = computeNextOutputMotorPowerReference(recentHumanOutputPowerAveraged,
                            recentMotorOutputPowerAveraged,
                            pollutionLevel,
                            usingSamplingPeriod);

                    if (recentHumanOutputPowerAveraged + motorOutputPowerReference != 0) {
                        mTarget = recentHumanOutputPowerAveraged /(recentHumanOutputPowerAveraged + motorOutputPowerReference);
                    }
                }

                // Adjust the output speed of the motor based on the error
                int requestToMotor = computeNextRequestToMotor(recentHumanOutputPowerAveraged,
                        motorOutputPowerReference,
                        recentMotorOutputPowerAveraged);

                float mActualAveraged = 0;

                if (recentHumanOutputPowerAveraged + recentMotorOutputPowerAveraged != 0) {
                    mActualAveraged = recentHumanOutputPowerAveraged / (recentHumanOutputPowerAveraged + recentMotorOutputPowerAveraged);
                }

                float error = 0;

                if(changeMEnabled) {
                    error = pollutionLevelArray.get(timestep);
                }

                // Store data for plotting
                BreathingControlData cooperativeBreathingControlData = new BreathingControlData(Integer.toString(requestToMotor),
                        0,
                        recentHumanOutputPowerAveraged,
                        recentMotorOutputPowerAveraged,
                        mTarget,
                        mActualAveraged,
                        error,
                        samplingPeriod);

                dbHandler.addBreathingControlData(cooperativeBreathingControlData);

                // Plot values - append to current data for each chart
                // We are plotting the last 20 values for each chart
                MValuePlottingData mValueActualData = dbHandler.getLatestMActual();

                // Need to get the characteristic of the feedback for multiple input values
                ArrayList<Float> inputToCharacteristicArray = new ArrayList<>();
                ArrayList<Float> characteristicArray = new ArrayList<>();
                float inputVal = maxPlottedWatts / numEntriesRequired;

                if(changeMEnabled){
                    for (int i = 0; i < numEntriesRequired; i++) {
                        inputToCharacteristicArray.add(inputVal * i);
                        characteristicArray.add((float) Math.sqrt(varyingCompeteCooperateCharacteristic(inputVal * i, pollutionLevelArray.get(timestep), mStar)));
                    }
                }
                else{
                    for (int i = 0; i < numEntriesRequired; i++) {
                        inputToCharacteristicArray.add(inputVal * i);
                        characteristicArray.add((float) Math.sqrt(competeCooperateCharacteristic(inputVal * i, pollutionLevel)));
                    }
                }

                // Object for sending across data to plot
                PlottingData dataToPlot = new PlottingData(mTarget,
                        mValueActualData,
                        inputToCharacteristicArray,
                        characteristicArray,
                        recentHumanOutputPowerAveraged,
                        motorOutputPowerReference,
                        recentMotorOutputPowerAveraged);

                // Update UI graphs
                handler.obtainMessage(PLOTTING_DATA, dataToPlot).sendToTarget();

                // Send request to bike
                handler.obtainMessage(COOP_COMPETE_DATA, requestToMotor).sendToTarget(); // TODO: Must enable this again after target
            }
        }

        /**
         * Method which uses a 4th order Runge-Kutta method to compute the value of the
         * reference output motor power for the next time instance.
         *
         * @param humanOutputPower average of the 5 most recent database entries for the human output power
         * @param motorOutputPower most recent database entry for the motor output power
         * @param pollutionLevel   value between 0 and 1 representing the level of pollution, 0 being low, 1 being high
         * @param dt               timestep used in calculation
         * @return the reference output motor power for the next time instance
         */
        private float computeNextOutputMotorPowerReference(float humanOutputPower, float motorOutputPower, float pollutionLevel, float dt) {
            float f1 = dt * pitchforkBifurcation(humanOutputPower, motorOutputPower, pollutionLevel);
            float f2 = dt * pitchforkBifurcation(humanOutputPower + dt / (float) 2.0, motorOutputPower + f1 / (float) 2.0, pollutionLevel);
            float f3 = dt * pitchforkBifurcation(humanOutputPower + dt / (float) 2.0, motorOutputPower + f2 / (float) 2.0, pollutionLevel);
            float f4 = dt * pitchforkBifurcation(humanOutputPower + dt, motorOutputPower + f3, pollutionLevel);
            float nextOutputReferencePower = motorOutputPower + (1 / (float) 6.0) * (f1 + 2 * f2 + 2 * f3 + f4);

            return nextOutputReferencePower;
        }

        /**
         * Compute reference output motor power for the given moving averaged human input power
         *
         * @param humanOutputPower Average of the 5 most recent database entries for the human output power
         * @param motorOutputPower Average of the 5 most recent database entries for the motor output power?
         * @param pollutionLevel   value between 0 and 1 representing the level of pollution, 0 being low, 1 being high
         * @return the output of the pitchfork bifurcation
         */
        private float pitchforkBifurcation(float humanOutputPower, float motorOutputPower, float pollutionLevel) {
            double retardingFactor = 0.00005; //TODO:
            float outputMotorPowerReference;

            outputMotorPowerReference = (float) retardingFactor * (competeCooperateCharacteristic(humanOutputPower, pollutionLevel) * motorOutputPower - (float) Math.pow((double) motorOutputPower, 3));

            return outputMotorPowerReference;
        }

        private float pitchforkBifurcation(float humanOutputPower, float motorOutputPower, float pollutionLevel, float mStar) {
            //float retardingFactor = (float)0.0001;//0.000085;//0.00005; //TODO:
            float kappa = (float)0.0125;

            if(mStar >= 0.4 && mStar < 1){
                kappa *= 2.5*mStar;
            }

            float retardingFactor = kappa/((2*(1-mStar)/mStar)*humanOutputPower); 

            float outputMotorPowerReference;

            outputMotorPowerReference = retardingFactor * (varyingCompeteCooperateCharacteristic(humanOutputPower, pollutionLevel, mStar) * motorOutputPower - (float) Math.pow((double) motorOutputPower, 3));

            return outputMotorPowerReference;
        }

        /**
         *
         * @param humanOutputPower
         * @param motorOutputPower
         * @param pollutionLevel
         * @param dt
         * @param mStar the target value of m in the cooperation region
         * @return
         */
        private float computeNextOutputMotorPowerReference(float humanOutputPower, float motorOutputPower, float pollutionLevel, float dt, float mStar) {
            float f1 = dt * pitchforkBifurcation(humanOutputPower, motorOutputPower, pollutionLevel, mStar);
            float f2 = dt * pitchforkBifurcation(humanOutputPower + dt / (float) 2.0, motorOutputPower + f1 / (float) 2.0, pollutionLevel, mStar);
            float f3 = dt * pitchforkBifurcation(humanOutputPower + dt / (float) 2.0, motorOutputPower + f2 / (float) 2.0, pollutionLevel, mStar);
            float f4 = dt * pitchforkBifurcation(humanOutputPower + dt, motorOutputPower + f3, pollutionLevel, mStar);
            float nextOutputReferencePower = motorOutputPower + (1 / (float) 6.0) * (f1 + 2 * f2 + 2 * f3 + f4);

            return nextOutputReferencePower;
        }

        /**
         * @param humanOutputPower Average of the 5 most recent database entries for the human output power
         * @param pollutionLevel   value between 0 and 1 representing the level of pollution, 0 being low, 1 being high
         * @return the output of the cooperate-competitive characteristic
         */
        private float competeCooperateCharacteristic(float humanOutputPower, float pollutionLevel) {
            // Need some way to compute a threshold
            // The larger the velocity, the larger the threshold should be
            float cooperationThreshold = 150; // Watts

            double beta = 1.4; // Scaling parameter for the slope of the cooperative characteristic
            float outputOfCharacteristic;

            if (humanOutputPower < cooperationThreshold) {
                outputOfCharacteristic = (float) Math.pow(beta * humanOutputPower * pollutionLevel, 2);
            } else {
                outputOfCharacteristic = (float) Math.pow((beta * pollutionLevel * cooperationThreshold) * Math.exp(-(humanOutputPower - cooperationThreshold) / 20.0), 2);
            }

            return outputOfCharacteristic;
        }

        private float varyingCompeteCooperateCharacteristic(float humanOutputPower, float pollutionLevel, float mStar) {
            // Need some way to compute a threshold
            // The larger the velocity, the larger the threshold should be
            float cooperationThreshold = 150; //300 - 200*pollutionLevel; // Watts
            float outputOfCharacteristic;

            if(mStar < 0.01){
                mStar = (float)0.01; // Prevent division by very small number
            }

            float beta = (1 - mStar)/mStar; // Scaling parameter for the slope of the cooperative characteristic

            if (humanOutputPower < cooperationThreshold) {
                outputOfCharacteristic = (float) Math.pow(beta * humanOutputPower, 2);
            } else {
                outputOfCharacteristic = (float) Math.pow((beta * cooperationThreshold) * Math.exp(-(humanOutputPower - cooperationThreshold) / 20.0), 2);
            }

            return outputOfCharacteristic;
        }

        /**
         * @param motorOutputPowerReference      the value to which we desire the actual motor power to converge to at this timestep
         * @param recentMotorOutputPowerAveraged running average of the most recent values of the actual output motor power
         * @return a value between 90 and 175 representing the next control value that should be sent to the motor over the Bluetooth connection
         */
        private int computeNextRequestToMotor(float recentHumanOutputPower, float motorOutputPowerReference, float recentMotorOutputPowerAveraged) {
            int nextRequestToMotor = 0;

            if (recentHumanOutputPower < 5) {
                nextRequestToMotor = 0;
            } else {
                nextRequestToMotor = lastRequestToMotor + (int) (0.10575 * (motorOutputPowerReference - recentMotorOutputPowerAveraged)); //TODO:

                // Ensure within bounds
                nextRequestToMotor = Math.max(nextRequestToMotor, 90);
                nextRequestToMotor = Math.min(nextRequestToMotor, 175);
            }

            lastRequestToMotor = nextRequestToMotor;

            return nextRequestToMotor;
        }

        public float calculateArrayAverage(float[] floatArray, int numAverages) {
            float sum = 0;

            for (int i = 0; i < numAverages; i++) {
                sum = sum + floatArray[i];
            }

            // Calculate average value
            float average = sum / (float) numAverages;
            
            return average;
        }
    }

    /**
     * bluetoothServiceConnection is needed to use BluetoothService methods
     */
    private ServiceConnection bluetoothServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            BluetoothService.BluetoothMyLocalBinder binder = (BluetoothService.BluetoothMyLocalBinder) service;
            bluetoothService = binder.getService();
            isBound = true;
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            isBound = false;
        }
    };

    /**
     * databaseServiceConnection is needed to use DatabaseService methods
     */
    private ServiceConnection databaseServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            DatabaseService.DatabaseMyLocalBinder binder = (DatabaseService.DatabaseMyLocalBinder) service;
            databaseService = binder.getService();
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
        }
    };
}
