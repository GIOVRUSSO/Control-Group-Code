package ie.ucd.smartrideRT;

public class TrafficLightData {
    private TrafficLightStatus trafficLightStatus;
    private float latitude;
    private float longitude;

    public TrafficLightData(TrafficLightStatus trafficLightStatus, float latitude, float longitude){
        this.trafficLightStatus = trafficLightStatus;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public TrafficLightStatus getTrafficLightStatus() {
        return trafficLightStatus;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getLongitude() {
        return longitude;
    }
}
