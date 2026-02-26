♻️ PlasTech

Smart Plastic Waste Detection and Automated Sorting System

📌 Overview

PlasTech is an intelligent IoT-based plastic waste detection and automated sorting system designed to promote efficient recycling through real-time classification and mechanical actuation.

The system integrates computer vision, embedded systems, and backend data processing to identify plastic materials and trigger automated sorting mechanisms with high reliability.

PlasTech transforms traditional waste segregation into a smart, data-driven process.

🚀 Key Features

🔍 AI-Based Plastic Detection
Uses Edge Impulse computer vision model deployed on ESP32-CAM to classify plastic materials.

⚙️ Automated Sorting Mechanism
Dual-servo control system rotates based on classification results.

💰 Coin Slot Integration
Optional reward mechanism for incentivized recycling.

🌐 Device-to-Cloud Architecture
Sends data to backend server via HTTP/MQTT for logging and monitoring.

📊 Real-Time Monitoring
Backend dashboard for tracking detections and system activity.

🛡️ Fail-safe Logic
Stops scanning when servo is active and resumes automatically.

🏗️ System Architecture

Hardware Layer

ESP32-CAM

Servo Motors (Sorting Mechanism)

Coin Slot Sensor

Power Regulation Module

Software Layer

Edge Impulse ML Model

Embedded C++ Firmware

REST API Backend

Cloud Database (e.g., Firebase / PostgreSQL)

🧠 AI Model

The system uses a lightweight image classification model trained using:

Edge Impulse

Embedded TensorFlow Lite (optimized for microcontrollers)

The model classifies:

Plastic

Non-Plastic

📡 Communication Flow

Camera captures image

AI model performs inference

If plastic detected → servo rotates 90°

Coin slot signal triggers reward mechanism

Data is sent to backend via HTTP POST

Backend stores and visualizes analytics

🎯 Purpose

PlasTech aims to:

Improve recycling efficiency

Reduce manual waste sorting

Promote smart environmental sustainability

Encourage behavioral change through incentive systems

🛠️ Technologies Used

ESP32-CAM

C++ (Embedded)

Edge Impulse

REST API (Node.js / Express)

PostgreSQL / Firebase

MQTT / HTTP Protocol

👨‍💻 Developed By

Jayson Nuñez
Elpie Landoy
Mark John Matining
