#!/usr/bin/env python3
"""
Integrated Camera + GPS Pipeline for Road Anomaly Detection
Produces JSON with proper frame numbers and saves detection images with bounding boxes
"""

import os
import sys
import time
import cv2
import numpy as np
import json
import math
import threading
import serial
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta, timezone
from collections import OrderedDict
import queue
import signal
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Any
import traceback

# Try importing optional S3 module
try:
    import boto3
    S3_AVAILABLE = True
except ImportError:
    S3_AVAILABLE = False
    print("⚠️ boto3 not installed - S3 upload disabled")

from ultralytics import YOLO
from pypylon import pylon
from trackers import ByteTrackTracker

# ------------------- CONFIGURATION -------------------

# Camera settings
MODEL_PATH = "nano_yolo/yolo26n.engine"
WIDTH = 1920
HEIGHT = 1200
FPS = 30
CONFIDENCE = 0.35
MAX_WIDTH = 1920
BITRATE = 12000000

# Tracker settings
TRACK_MAX_MISSING = 20
TRACK_MIN_IOU = 0.25
TRACK_MAX_CENTER_DISTANCE = 150.0

# GPS settings
GPS_PORT = "/dev/ttyACM0"
GPS_BAUD = 9600

# AWS S3 settings (optional)
S3_BUCKET = "raiotransection.s3.ap-south-1.amazonaws.com"
S3_PREFIX = "output/ndd/pavement/anomalies/frames"
USE_S3 = False

# Session management
BASE_SESSION_DIR = "road_survey_sessions"
IST = timezone(timedelta(hours=5, minutes=30))

# Start point for distance calculation (UPDATE THESE!)
START_LAT = 17.61736077
START_LON = 80.0380441

# Image saving settings
SAVE_DETECTION_IMAGES = True
IMAGE_QUALITY = 95  # JPEG quality (higher = better quality, larger file)
SAVE_FULL_RES = True  # Save at 1920x1080 resolution

# Performance settings
DISPLAY_PREVIEW = True
PREVIEW_SCALE = 0.5
MAX_QUEUE_SIZE = 30

# ------------------- DATA CLASSES -------------------

@dataclass
class GPSPoint:
    """GPS data point"""
    timestamp: float
    latitude: float
    longitude: float
    speed: float = 0.0
    satellites: int = 0
    
@dataclass
class Detection:
    """Single detection in a frame"""
    frame_number: int
    timestamp: float
    track_id: int
    anomaly_number: int
    class_name: str
    confidence: float
    bbox: List[float]  # [x1, y1, x2, y2]
    gps: Optional[GPSPoint] = None
    
@dataclass
class Anomaly:
    """Complete anomaly record for JSON output"""
    anomaly_number: str
    timestamp_on_video: str
    anomaly_type: str
    frame_no: str
    latitude: float
    longitude: float
    distance_from_start: str
    length_in_meters: str
    width_in_meters: str
    image: str
    # Internal fields (not in JSON)
    track_id: Optional[int] = None
    first_frame: Optional[int] = None
    last_frame: Optional[int] = None

# ------------------- TIME UTILS -------------------

def now_ist():
    """Get current time in IST"""
    return datetime.now(IST)

def get_session_timestamp():
    """Create timestamp for session folder"""
    return now_ist().strftime("%Y%m%d_%H%M%S")

def format_timestamp_for_json(seconds=None):
    """Format timestamp for JSON output (YYYY-MM-DD HH:MM:SS)"""
    if seconds:
        dt = datetime.fromtimestamp(seconds, IST)
    else:
        dt = now_ist()
    return dt.strftime("%Y-%m-%d %H:%M:%S")

# ------------------- DISTANCE CALCULATION -------------------

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates"""
    R = 6371000  # Earth's radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi/2) * math.sin(delta_phi/2) + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda/2) * math.sin(delta_lambda/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

# ------------------- GPS LOGGER -------------------

class GPSLogger:
    """GPS logger with circular buffer for efficient access"""
    
    def __init__(self, session_dir, session_ts):
        self.session_dir = session_dir
        self.session_ts = session_ts
        
        # File paths
        self.gpx_path = os.path.join(session_dir, f"track_{session_ts}.gpx")
        self.nmea_path = os.path.join(session_dir, f"raw_{session_ts}.nmea")
        
        # GPS data buffer (1 hour at 1Hz = 3600 points)
        self.buffer_size = 3600
        self.gps_buffer = [None] * self.buffer_size
        self.buffer_index = 0
        self.current_gps = None
        self.gps_count = 0
        
        # Thread control
        self.stop_event = threading.Event()
        self.thread = None
        self.lock = threading.Lock()
        
        # Initialize GPX
        self._init_gpx()
        
    def _init_gpx(self):
        """Initialize GPX XML structure"""
        GPX_NS = "http://www.topografix.com/GPX/1/1"
        ET.register_namespace("", GPX_NS)
        
        self.gpx = ET.Element(f"{{{GPX_NS}}}gpx", version="1.1", creator="Road Survey GPS Logger")
        trk = ET.SubElement(self.gpx, f"{{{GPX_NS}}}trk")
        ET.SubElement(trk, f"{{{GPX_NS}}}name").text = f"Road Survey {self.session_ts}"
        self.seg = ET.SubElement(trk, f"{{{GPX_NS}}}trkseg")
    
    def _parse_nmea(self, line):
        """Fast NMEA parsing"""
        if not line.startswith('$'):
            return None
            
        parts = line.split(',')
        if len(parts) < 6:
            return None
            
        msg_type = parts[0][3:]
        
        # Parse GGA for complete fix
        if msg_type == "GGA" and len(parts) > 9:
            try:
                if parts[2] and parts[4] and parts[6]:
                    # Convert latitude
                    lat_str = parts[2]
                    lat_deg = float(lat_str[:2])
                    lat_min = float(lat_str[2:])
                    latitude = lat_deg + lat_min/60.0
                    if parts[3] == 'S':
                        latitude = -latitude
                    
                    # Convert longitude
                    lon_str = parts[4]
                    lon_deg = float(lon_str[:3])
                    lon_min = float(lon_str[3:])
                    longitude = lon_deg + lon_min/60.0
                    if parts[5] == 'W':
                        longitude = -longitude
                    
                    fix_quality = int(parts[6])
                    satellites = int(parts[7]) if parts[7] else 0
                    
                    if fix_quality > 0:
                        return {
                            'lat': latitude,
                            'lon': longitude,
                            'fix': fix_quality,
                            'sats': satellites
                        }
            except:
                pass
        
        return None
    
    def start(self):
        """Start GPS logging thread"""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print(f"✅ GPS logger started")
        return self
    
    def stop(self):
        """Stop GPS logging thread"""
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        self._save_gpx()
        print(f"✅ GPS logger stopped - {self.gps_count} points logged")
    
    def _run(self):
        """Main GPS logging loop"""
        try:
            ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.5)
        except Exception as e:
            print(f"❌ GPS connection failed: {e}")
            return
        
        nmea_file = open(self.nmea_path, 'w', buffering=8192)
        
        while not self.stop_event.is_set():
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                
                # Write raw NMEA
                current_time = time.time()
                nmea_file.write(f"{current_time} {line}\n")
                
                # Parse for position
                parsed = self._parse_nmea(line)
                if parsed:
                    gps_point = GPSPoint(
                        timestamp=current_time,
                        latitude=parsed['lat'],
                        longitude=parsed['lon'],
                        satellites=parsed['sats']
                    )
                    
                    with self.lock:
                        self.current_gps = gps_point
                        self.gps_buffer[self.buffer_index] = gps_point
                        self.buffer_index = (self.buffer_index + 1) % self.buffer_size
                        self.gps_count += 1
                    
                    # Add to GPX
                    self._add_gpx_point(gps_point)
                    
            except Exception:
                pass
        
        nmea_file.close()
        ser.close()
    
    def _add_gpx_point(self, point):
        """Add point to GPX track"""
        pt = ET.SubElement(
            self.seg,
            "{http://www.topografix.com/GPX/1/1}trkpt",
            lat=f"{point.latitude:.7f}",
            lon=f"{point.longitude:.7f}"
        )
        ET.SubElement(pt, "{http://www.topografix.com/GPX/1/1}time").text = format_timestamp_for_json(point.timestamp)
    
    def _save_gpx(self):
        """Save GPX file"""
        try:
            tree = ET.ElementTree(self.gpx)
            tree.write(self.gpx_path, encoding='utf-8', xml_declaration=True)
        except Exception as e:
            print(f"⚠️ Failed to save GPX: {e}")
    
    def get_gps_for_frame(self, frame_number):
        """Get GPS data synchronized with frame number"""
        if self.current_gps is None or self.gps_count == 0:
            return None
        
        with self.lock:
            # 1 GPS point per second = every 30 frames
            gps_index = min(frame_number // FPS, self.gps_count - 1)
            if gps_index < self.buffer_size:
                point = self.gps_buffer[gps_index % self.buffer_size]
                if point:
                    return point
            
            return self.current_gps

# ------------------- IMAGE SAVER THREAD -------------------

class ImageSaver:
    """Background thread for saving images with bounding boxes"""
    
    def __init__(self, session_dir):
        self.session_dir = session_dir
        self.images_dir = os.path.join(session_dir, "anomaly_images")
        os.makedirs(self.images_dir, exist_ok=True)
        
        self.queue = queue.Queue(maxsize=MAX_QUEUE_SIZE)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._saver_worker, daemon=True)
        self.thread.start()
        
        self.saved_paths = []
        
    def _saver_worker(self):
        """Background thread for saving images"""
        while not self.stop_event.is_set():
            try:
                item = self.queue.get(timeout=0.5)
                if item is None:
                    break
                
                frame, anomaly_num, class_name, frame_number, timestamp = item
                
                # Create filename
                filename = f"anomaly_{anomaly_num:04d}_{class_name}_frame{frame_number:06d}_{int(timestamp)}.jpg"
                filepath = os.path.join(self.images_dir, filename)
                
                # Save image with high quality
                success = cv2.imwrite(filepath, frame, [
                    cv2.IMWRITE_JPEG_QUALITY, IMAGE_QUALITY,
                    cv2.IMWRITE_JPEG_OPTIMIZE, 1
                ])
                
                if success:
                    self.saved_paths.append(filepath)
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"⚠️ Image save error: {e}")
    
    def save_image(self, frame, anomaly_num, class_name, frame_number, timestamp):
        """Queue image for saving (non-blocking)"""
        try:
            self.queue.put_nowait((frame.copy(), anomaly_num, class_name, frame_number, timestamp))
        except queue.Full:
            pass  # Skip if queue is full
    
    def stop(self):
        """Stop the saver thread"""
        self.stop_event.set()
        self.queue.put(None)
        self.thread.join(timeout=2)
        return self.saved_paths

# ------------------- ANOMALY TRACKER -------------------

class AnomalyTracker:
    """Tracks anomalies and assigns unique IDs with detection images"""
    
    def __init__(self, session_dir):
        self.session_dir = session_dir
        self.next_id = 1
        
        # Active tracks: track_id -> Anomaly object
        self.active_tracks = OrderedDict()
        
        # Completed anomalies
        self.completed_anomalies = []
        
        # Image saver
        self.image_saver = ImageSaver(session_dir) if SAVE_DETECTION_IMAGES else None
        
        # Tracking stats
        self.detection_count = 0
        
    def update(self, detections, frame_number, timestamp, gps_data, annotated_frame=None):
        """
        Update tracks with new detections
        Returns: List of (track_id, anomaly_number, is_new) for each detection
        """
        results = []
        current_tracks = set()
        
        for det in detections:
            track_id = det['tracking_id']
            if track_id is None:
                continue
                
            current_tracks.add(track_id)
            class_name = det['object_class_name']
            
            # Check if this is a new track
            if track_id not in self.active_tracks:
                anomaly_number = self.next_id
                self.next_id += 1
                
                # Calculate distance from start
                if gps_data:
                    distance = haversine_distance(
                        START_LAT, START_LON,
                        gps_data.latitude, gps_data.longitude
                    )
                else:
                    distance = 0
                
                # Create anomaly record
                anomaly = Anomaly(
                    anomaly_number=str(anomaly_number),
                    timestamp_on_video=format_timestamp_for_json(timestamp),
                    anomaly_type=class_name.upper(),
                    frame_no=str(frame_number),
                    latitude=gps_data.latitude if gps_data else None,
                    longitude=gps_data.longitude if gps_data else None,
                    distance_from_start=f"{distance:.2f}",
                    length_in_meters="",
                    width_in_meters="",
                    image="",
                    track_id=track_id,
                    first_frame=frame_number,
                    last_frame=frame_number
                )
                
                self.active_tracks[track_id] = anomaly
                
                # Save detection image (with bounding boxes)
                if self.image_saver and annotated_frame is not None:
                    self.image_saver.save_image(
                        annotated_frame, 
                        anomaly_number, 
                        class_name,
                        frame_number,
                        timestamp
                    )
                
                results.append((track_id, anomaly_number, True))
                
            else:
                # Update existing track
                anomaly = self.active_tracks[track_id]
                anomaly.last_frame = frame_number
                results.append((track_id, int(anomaly.anomaly_number), False))
            
            self.detection_count += 1
        
        # Check for lost tracks (not seen in this frame)
        current_time = time.time()
        for track_id in list(self.active_tracks.keys()):
            if track_id not in current_tracks:
                anomaly = self.active_tracks[track_id]
                frames_since_last = frame_number - anomaly.last_frame
                
                # If lost for too long, finalize
                if frames_since_last > TRACK_MAX_MISSING:
                    self._finalize_track(track_id)
        
        return results
    
    def _finalize_track(self, track_id):
        """Move track to completed anomalies"""
        if track_id in self.active_tracks:
            anomaly = self.active_tracks.pop(track_id)
            
            # Create clean anomaly dict for JSON
            anomaly_dict = {
                "Anomaly number": anomaly.anomaly_number,
                "Timestamp on processed video": anomaly.timestamp_on_video,
                "Anomaly type": anomaly.anomaly_type,
                "Frame no.": anomaly.frame_no,
                "Latitude": anomaly.latitude,
                "Longitude": anomaly.longitude,
                "Distance from start point in meters": anomaly.distance_from_start,
                "Length in meters": anomaly.length_in_meters,
                "Width in meters": anomaly.width_in_meters,
                "image": ""  # Will be filled after image saving
            }
            
            self.completed_anomalies.append(anomaly_dict)
    
    def finalize_all(self):
        """Finalize all remaining tracks"""
        for track_id in list(self.active_tracks.keys()):
            self._finalize_track(track_id)
        
        # Stop image saver and get saved paths
        saved_images = []
        if self.image_saver:
            saved_images = self.image_saver.stop()
        
        # Match saved images with anomalies
        for i, anomaly in enumerate(self.completed_anomalies):
            if i < len(saved_images):
                # For local path or S3 URL
                if USE_S3 and S3_AVAILABLE:
                    # Generate S3 URL
                    filename = os.path.basename(saved_images[i])
                    anomaly['image'] = f"https://{S3_BUCKET}/{S3_PREFIX}/{os.path.basename(self.session_dir)}/{filename}"
                else:
                    # Local path reference
                    anomaly['image'] = saved_images[i]
        
        return self.completed_anomalies, saved_images

# ------------------- MAIN PIPELINE -------------------

class RoadSurveyPipeline:
    """Main pipeline class"""
    
    def __init__(self):
        self.session_ts = get_session_timestamp()
        self.session_dir = os.path.join(BASE_SESSION_DIR, self.session_ts)
        os.makedirs(self.session_dir, exist_ok=True)
        
        self.running = True
        self.frame_number = 0
        self.stats = {
            'frames': 0,
            'detections': 0,
            'anomalies': 0,
            'fps_history': []
        }
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        print("\n⏹️ Stopping pipeline...")
        self.running = False
    
    def setup_camera(self):
        """Initialize and configure camera"""
        print("📷 Initializing Basler camera...")
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        
        # Configure camera
        self.camera.Width.SetValue(WIDTH)
        self.camera.Height.SetValue(HEIGHT)
        self.camera.PixelFormat.SetValue("BayerRG8")
        self.camera.AcquisitionFrameRateEnable.SetValue(True)
        self.camera.AcquisitionFrameRate.SetValue(FPS)
        self.camera.ExposureTime.SetValue(20000)
        
        # Image converter
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        
        # Output size for video
        if WIDTH > MAX_WIDTH:
            scale = MAX_WIDTH / WIDTH
            self.out_w = int(WIDTH * scale)
            self.out_h = int(HEIGHT * scale)
        else:
            self.out_w = WIDTH
            self.out_h = HEIGHT
        
        self.scale_x = self.out_w / float(WIDTH)
        self.scale_y = self.out_h / float(HEIGHT)
        
        print(f"✅ Camera initialized: {WIDTH}x{HEIGHT} -> {self.out_w}x{self.out_h}")
    
    def setup_video_writer(self):
        """Setup video writer"""
        video_path = os.path.join(self.session_dir, f"survey_video_{self.session_ts}.mp4")
        
        # Try GStreamer first
        gst_pipeline = (
            f'appsrc is-live=true block=true format=time do-timestamp=true ! '
            f'video/x-raw,format=BGR,width={self.out_w},height={self.out_h},framerate={FPS}/1 ! '
            f'queue max-size-buffers=2 leaky=downstream ! '
            f'videoconvert ! '
            f'video/x-raw,format=I420 ! '
            f'x264enc bitrate={BITRATE//1000} speed-preset=ultrafast tune=zerolatency ! '
            f'mp4mux ! '
            f'filesink location="{video_path}" sync=false'
        )
        
        self.out = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, FPS, 
                                   (self.out_w, self.out_h), True)
        
        if not self.out.isOpened():
            print("⚠️ GStreamer failed, using OpenCV fallback...")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter(video_path, fourcc, FPS, 
                                       (self.out_w, self.out_h), True)
        
        print(f"✅ Video writer initialized")
    
    def draw_detections(self, frame, detections, track_results, scale_x, scale_y):
        """Draw bounding boxes and anomaly numbers on frame"""
        annotated = frame.copy()
        
        for det in detections:
            track_id = det['tracking_id']
            if track_id is None:
                continue
            
            # Find anomaly number for this track
            anomaly_num = None
            for t_id, a_num, _ in track_results:
                if t_id == track_id:
                    anomaly_num = a_num
                    break
            
            if anomaly_num is None:
                continue
            
            # Get bounding box
            x1, y1, x2, y2 = det['bbox']
            
            # Scale to output size
            x1s = int(x1 * scale_x)
            y1s = int(y1 * scale_y)
            x2s = int(x2 * scale_x)
            y2s = int(y2 * scale_y)
            
            # Generate color based on anomaly number
            color = (
                int((37 * anomaly_num) % 255),
                int((67 * anomaly_num) % 180),
                int((97 * anomaly_num) % 255)
            )
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1s, y1s), (x2s, y2s), color, 3)
            
            # Draw label with anomaly number
            label = f"#{anomaly_num} {det['object_class_name']}"
            
            # Get text size
            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
            )
            
            # Draw text background
            cv2.rectangle(
                annotated,
                (x1s, y1s - text_h - 10),
                (x1s + text_w + 10, y1s),
                color,
                -1
            )
            
            # Draw text
            cv2.putText(
                annotated, label,
                (x1s + 5, y1s - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 2
            )
        
        return annotated
    
    def run(self):
        """Main pipeline execution"""
        
        print("=" * 80)
        print("ROAD ANOMALY DETECTION PIPELINE")
        print("=" * 80)
        print(f"📁 Session: {self.session_dir}")
        print(f"📷 Camera: {WIDTH}x{HEIGHT} @ {FPS}FPS")
        print(f"🎯 Confidence: {CONFIDENCE}")
        print(f"🛰️ GPS: {GPS_PORT}")
        print(f"🖼️ Save detection images: {SAVE_DETECTION_IMAGES}")
        print("=" * 80)
        
        # Initialize GPS
        print("\n🛰️ Starting GPS logger...")
        self.gps_logger = GPSLogger(self.session_dir, self.session_ts).start()
        time.sleep(2)
        
        # Load model
        print("\n🚀 Loading TensorRT model...")
        self.model = YOLO(MODEL_PATH)
        print(f"✅ Model loaded on: {self.model.device}")
        
        # Setup camera and video
        self.setup_camera()
        self.setup_video_writer()
        
        # Initialize trackers
        self.tracker = ByteTrackTracker(
            max_missing=TRACK_MAX_MISSING,
            min_iou=TRACK_MIN_IOU,
            max_center_distance=TRACK_MAX_CENTER_DISTANCE,
            fps=FPS,
            use_bytetrack_if_available=False
        )
        print(f"✅ Tracker backend: {self.tracker.backend}")
        
        self.anomaly_tracker = AnomalyTracker(self.session_dir)
        
        # Start grabbing
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        print("\n🎥 Recording started - Press Ctrl+C to stop")
        
        # Performance tracking
        prev_time = time.time()
        fps_history = []
        
        try:
            while self.running and self.camera.IsGrabbing():
                loop_start = time.time()
                
                # Grab frame
                grab = self.camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
                
                if grab.GrabSucceeded():
                    # Convert frame
                    image = self.converter.Convert(grab)
                    frame = image.GetArray()
                    timestamp = time.time()
                    
                    # Get GPS for this frame
                    gps_data = self.gps_logger.get_gps_for_frame(self.frame_number)
                    
                    # Run inference
                    results = self.model.predict(
                        frame,
                        conf=CONFIDENCE,
                        device=0,
                        verbose=False,
                        half=True  # FP16 inference
                    )[0]
                    
                    # Process detections
                    detections = []
                    if results.boxes is not None and len(results.boxes) > 0:
                        boxes = results.boxes.xyxy.cpu().numpy()
                        scores = results.boxes.conf.cpu().numpy()
                        classes = results.boxes.cls.cpu().numpy()
                        
                        # Update tracker
                        track_ids = self.tracker.update(boxes, scores, classes, frame=frame)
                        
                        for i, box in enumerate(boxes):
                            if i < len(track_ids) and track_ids[i] is not None:
                                detection = {
                                    'tracking_id': int(track_ids[i]),
                                    'object_class_name': self.model.names[int(classes[i])],
                                    'bbox': box.tolist(),
                                    'confidence': float(scores[i])
                                }
                                detections.append(detection)
                    else:
                        self.tracker.update(None, None, None, frame=frame)
                    
                    # Create annotated frame with bounding boxes
                    annotated_frame = results.plot()
                    
                    # Get track results from anomaly tracker
                    track_results = self.anomaly_tracker.update(
                        detections,
                        self.frame_number,
                        timestamp,
                        gps_data,
                        annotated_frame if SAVE_DETECTION_IMAGES else None
                    )
                    
                    # Draw enhanced annotations on frame for video
                    display_frame = self.draw_detections(
                        annotated_frame, detections, track_results,
                        self.scale_x, self.scale_y
                    )
                    
                    # Resize for video output
                    if WIDTH > MAX_WIDTH:
                        display_frame = cv2.resize(display_frame, (self.out_w, self.out_h))
                    
                    # Add info text
                    cv2.putText(
                        display_frame,
                        f"Frame: {self.frame_number} | Anomalies: {self.anomaly_tracker.next_id - 1}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2
                    )
                    
                    if gps_data:
                        cv2.putText(
                            display_frame,
                            f"GPS: {gps_data.latitude:.6f}, {gps_data.longitude:.6f}",
                            (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 1
                        )
                    
                    # Write frame
                    self.out.write(display_frame)
                    
                    # Update stats
                    self.stats['frames'] += 1
                    self.stats['detections'] += len(detections)
                    
                    # Calculate FPS
                    current_time = time.time()
                    fps = 1.0 / (current_time - prev_time)
                    prev_time = current_time
                    self.stats['fps_history'].append(fps)
                    
                    # Preview
                    if DISPLAY_PREVIEW:
                        preview = cv2.resize(display_frame, 
                                           (int(self.out_w * PREVIEW_SCALE), 
                                            int(self.out_h * PREVIEW_SCALE)))
                        cv2.putText(preview, f"FPS: {fps:.1f}", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.imshow("Road Survey - Press Ctrl+C to stop", preview)
                        cv2.waitKey(1)
                    
                    self.frame_number += 1
                
                grab.Release()
                
                # Throttle to maintain real-time
                loop_time = time.time() - loop_start
                if loop_time < 1.0/FPS:
                    time.sleep(1.0/FPS - loop_time)
                    
        except Exception as e:
            print(f"\n❌ Error: {e}")
            traceback.print_exc()
            
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown"""
        print("\n\n🛑 Cleaning up...")
        
        # Stop camera
        if hasattr(self, 'camera'):
            self.camera.StopGrabbing()
            self.camera.Close()
        
        # Release video
        if hasattr(self, 'out'):
            self.out.release()
        
        cv2.destroyAllWindows()
        
        # Stop GPS
        if hasattr(self, 'gps_logger'):
            self.gps_logger.stop()
        
        # Get final anomalies
        anomalies, saved_images = self.anomaly_tracker.finalize_all()
        
        # Save JSON
        json_output = {"Anomalies": anomalies}
        json_path = os.path.join(self.session_dir, f"anomalies_{self.session_ts}.json")
        
        with open(json_path, 'w') as f:
            json.dump(json_output, f, indent=2)
        
        # Calculate average FPS
        avg_fps = sum(self.stats['fps_history'][-100:]) / min(100, len(self.stats['fps_history']))
        
        # Print summary
        print("\n" + "=" * 80)
        print("✅ SESSION COMPLETED")
        print("=" * 80)
        print(f"📁 Session folder: {self.session_dir}")
        print(f"📊 Statistics:")
        print(f"   - Frames processed: {self.stats['frames']}")
        print(f"   - Total detections: {self.stats['detections']}")
        print(f"   - Unique anomalies: {len(anomalies)}")
        print(f"   - Average FPS: {avg_fps:.1f}")
        print(f"📹 Video saved")
        print(f"🛰️ GPS track saved")
        print(f"📄 JSON saved: {json_path}")
        
        if saved_images:
            print(f"🖼️ Detection images saved: {len(saved_images)}")
            print(f"   Location: {os.path.join(self.session_dir, 'anomaly_images')}")
        
        if anomalies:
            print("\n📋 First 5 anomalies:")
            for a in anomalies[:5]:
                print(f"   #{a['Anomaly number']}: {a['Anomaly type']} at frame {a['Frame no.']}")

# ------------------- ENTRY POINT -------------------

if __name__ == "__main__":
    pipeline = RoadSurveyPipeline()
    pipeline.run()