# 🚀 Basler + TensorRT Road Anomaly Detection Pipeline


## 🎬 Demo (Live Output)

### 🎥 GIF Preview
![Demo GIF](gif/output.gif)

---

### 📸 Detection Samples

| Anomaly | Preview |
|--------|--------|
| Laptop | ![](images/anomaly_0039_laptop_frame001027_1773745495.jpg) |
| Person | ![](images/anomaly_0043_person_frame001136_1773745499.jpg) |
| Clock | ![](images/anomaly_0047_clock_frame001199_1773745501.jpg) |
| Person | ![](images/anomaly_0056_person_frame001346_1773745506.jpg) |


## 📌 Overview

This project implements a **real-time road anomaly detection system** using:

* 📷 Basler Camera (pypylon)
* ⚡ TensorRT YOLO (Ultralytics)
* 🛰️ GPS logging (NMEA + GPX)
* 🎯 Object tracking (ByteTrack)
* 🎥 Video recording
* 📸 Detection image saving (with bounding boxes)
* 📄 Structured JSON output

---

## 🎯 Key Features

### ✅ Detection Images with Bounding Boxes

* Saves images **with bounding boxes**
* Each anomaly has a **unique ID**
* High-quality JPEG (95%)
* Full resolution (1920×1200)
* Non-blocking background saving

---

### ✅ JSON Output with Frame Numbers

```json
{
  "Anomalies": [
    {
      "Anomaly number": "1",
      "Timestamp on processed video": "2026-03-17 15:30:22",
      "Anomaly type": "POTHOLE",
      "Frame no.": "157",
      "Latitude": 17.61736077,
      "Longitude": 80.0380441,
      "Distance from start point in meters": "1584.10",
      "Length in meters": "",
      "Width in meters": "",
      "image": "anomaly_images/anomaly_0001_POTHOLE_frame000157.jpg"
    }
  ]
}
```

---

### 📸 Image Naming Convention

```
anomaly_0001_POTHOLE_frame000157_timestamp.jpg
```

| Part        | Meaning      |
| ----------- | ------------ |
| 0001        | Anomaly ID   |
| POTHOLE     | Class        |
| frame000157 | Frame number |
| timestamp   | Capture time |

---

### 🎨 Visual Output

* Colored bounding boxes
* Unique anomaly IDs
* Class labels
* GPS coordinates overlay
* Frame counter

---

## ⚡ Performance Optimizations

* FP16 TensorRT inference
* Background image saving thread
* GPS circular buffer
* Queue-based pipeline
* Real-time FPS control

---

## 📁 Project Structure

```
road_survey_sessions/
└── SESSION_ID/
    ├── survey_video.mp4
    ├── track.gpx
    ├── raw.nmea
    ├── anomalies.json
    └── anomaly_images/
        ├── anomaly_0001.jpg
        ├── anomaly_0002.jpg
```

---

## ▶️ How to Run

```bash
python3 v6.py
```

Press:

```
Ctrl + C
```

to stop recording.

---

## ⚙️ Requirements

* Python 3.8+
* OpenCV (with GStreamer)
* pypylon (Basler SDK)
* ultralytics (YOLO)
* TensorRT engine (.engine file)
* serial (for GPS)

---

## 🔥 Pipeline Flow

```
Basler Camera
     ↓
Frame Capture
     ↓
YOLO TensorRT Inference
     ↓
ByteTrack Tracking
     ↓
Anomaly Detection
     ↓
├── Video Recording
├── JSON Logging
├── GPS Sync
└── Image Saving
```

---

## 📊 Output Summary

* 🎥 Annotated video
* 📄 JSON anomaly file
* 🛰️ GPS track (GPX)
* 📡 Raw NMEA log
* 📸 Detection images

---

## 🚀 Use Cases

* Road condition monitoring
* Smart city infrastructure
* Autonomous vehicle preprocessing
* Survey and inspection systems

---

## 👨‍💻 Author

**** Hanuai

---

## ⭐ Notes

* La
rge files (videos/images) are NOT stored in repo
* Only code + logs are version controlled
* Optimized for NVIDIA Jetson (AGX Orin)

---
