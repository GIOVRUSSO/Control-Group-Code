# EmpaLink Sample Project

## Introduction

This project gives you the code you need to connect to an Empatica E4 device and start streaming data, and uploading to the Cloud.

The E4 Link application implemented in the project has very simple functionalities:

- It initializes the EmpaLink library with your API key.
- If the previous step is successful, it starts scanning for Empatica devices, till it finds one that can be used with the API key you inserted in the code.
- When such a device has been found, the app connects to the devices and streams data.
- Store the IBI, EDA, TEMP, BVP, BATTERY data as txt files per miniute
- Upload the latest full data files to the cloud. (dropbox in this project)
- Stop streaming data once the button DISCONNECT has been pressed.

## Setup

- Clone / download this repository.
- Open the sample project in Android Studio.
- Make sure you have a valid API key. You can request one for your Empatica Connect account from our [Developer Area][1].
- Edit `MainActivity.java` and assign your API key to the `EMPATICA_API_KEY` constant .
- Make sure the `.aar` file you'll find inside into the `libs` folder contained in the sample project.
- Build and run the project.
- If a device is in range and its light is blinking green, but the app doesn't connect, please check that the discovered device can be used with your API key.
- If the `allowed` parameter is always false, the device is not linked to your API key. Please check your [Developer Area][1].

Any additional information about the Empatica API for Android, please check the [official documentation][2].

[1]: https://www.empatica.com/connect/developer.php
[2]: http://developer.empatica.com
