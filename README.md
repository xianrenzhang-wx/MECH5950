## Multimodal Wearable Sleep Monitoring System

This repository contains the Arduino firmware for a multimodal wearable system designed for continuous sleep-quality monitoring. The system integrates accelerometer-based respiratory and posture sensing with photoplethysmography (PPG) for cardiopulmonary monitoring, enabling the detection of sleep-related physiological events such as apnea, hypopnea, oxygen desaturation, and posture changes.

## Group Members

He Anwei

Wang Xu

Wang Chaoqi

## Arduino Code Overview

The Arduino code in this repository implements the embedded control and data acquisition logic for the wearable sleep-monitoring device. It is responsible for coordinating multiple sensors, preprocessing raw signals, and transmitting data for further analysis.

Specifically, the firmware performs the following functions:

Sensor interfacing and data acquisition
The code initializes and communicates with a triaxial accelerometer to acquire chest motion and orientation data, as well as a MAX30102 PPG sensor to record red and infrared photoplethysmographic signals for heart rate and SpO₂ estimation.

Multimodal signal synchronization
Accelerometer and PPG data streams are sampled in a synchronized manner to ensure temporal alignment between respiratory mechanics, body posture, and cardiopulmonary signals, which is critical for subsequent multimodal analysis.

Basic signal preprocessing
The firmware applies lightweight preprocessing steps such as baseline correction, simple filtering, and outlier handling to reduce motion artifacts and signal drift before transmission, improving data quality while maintaining low power consumption.

Low-power operation and data transmission
The code is designed for low-power, wearable operation, enabling overnight monitoring. Acquired sensor data are transmitted wirelessly or via serial communication to an external device for storage, visualization, and machine-learning–based analysis.

Overall, the Arduino firmware serves as the hardware–software interface of the wearable system, providing reliable, real-time acquisition of multimodal physiological signals that form the basis for sleep-quality evaluation and sleep-disordered breathing analysis.
