
---

# Projektseminar an TU Darmstadt  
## Zusammenfassung:  
Ziel dieser Arbeit ist die Entwicklung eines KI-gesteuerten Fahrzeugs, **JetBot AI**, unter Nutzung des NVIDIA Jetson Nano Moduls. Der Fokus liegt auf der Implementierung zentraler Funktionalitäten wie der autonomen Hinderniserkennung mittels Kameradaten, der Planung von Navigationspfaden sowie der Bewegungssteuerung des Fahrzeugs. Durch die Analyse von Umgebungsdaten wird ein sicherer Navigationspfad generiert, um Hindernisse effektiv zu vermeiden, und der JetBot wird präzise gesteuert, um die erforderlichen Aktionen auszuführen. Die Arbeit integriert fortschrittliche Methoden wie Kartenerstellung, Hinderniserkennung und Umgehungsalgorithmen. Es werden die eingesetzten Verfahren zur Objekterkennung, Hindernisvermeidung und Pfadplanung detailliert erläutert, einschließlich Mapping, Objektverfolgung und -sortierung.

---

# 🚗 JetBot: KI-gesteuerter autonomer Roboter auf dem NVIDIA Jetson Nano

![JetBot Demo](https://github.com/husamhamu/ps_robotik/assets/demo.gif) <!-- Ersetze durch echten GIF- oder Video-Link -->

> **JetBot** ist ein KI-basierter autonomer Roboter auf Basis des NVIDIA Jetson Nano. Er bietet Objekterkennung durch Deep Learning, Hindernis-Lokalisierung mittels AprilTags, Echtzeit-Pfadplanung sowie autonome Navigation durch RRT-Algorithmen und PID-Motorsteuerung.

## 🧠 Zentrale Funktionen

- 🔍 **Objekterkennung**: Integration und Benchmarking von SSD, YOLOv7 und YOLOv3 für Echtzeit-Objekterkennung auf eingebetteter Hardware.  
- 🧭 **Navigation & Hindernisvermeidung**: Lokalisierung von Hindernissen mittels Kamerakalibrierung und AprilTag-Erkennung sowie deren Umgehung durch fortgeschrittene Koordinatentransformationen und Bewegungsplanung.  
- 🗺️ **Pfadplanung**: Implementierung der Algorithmen RRT und RRT* (Rapidly-exploring Random Tree) zur dynamischen Pfadsuche in einer komplexen Umgebung.  
- 🤖 **Robotersteuerung**: Einsatz eines PID-Reglers zur Steuerung des Motorverhaltens basierend auf Kameradaten und Objektposition.  
- 📡 **ROS-Integration**: Vollständig integriert in das Robot Operating System (ROS) zur Verarbeitung und Visualisierung von Echtzeitdaten (rviz, rqt).  
- 🔧 **Hardware-Optimierung**: Ausführung auf ressourcenbeschränktem Jetson Nano mittels Docker und SWAP-Speicheroptimierung für rechenintensive ML-Inferenz.

## 📚 Projektstruktur

```
jetbot/
│
├── deep_learning/              # YOLOv7, SSD, Datenverarbeitung & Training
├── apriltag/                   # AprilTag-Setup, Kalibrierung, ROS-Integration
├── path_planning/             # RRT-Algorithmen, Koordinatentransformation
├── motor_control/             # Differenzialantrieb mit PID-Regelung
├── scripts/                   # ROS-Nodes & Launch-Dateien
└── visualization/             # Karten, rviz-Konfiguration, Koordinatenprotokolle
```

## 📈 Visuelle Demonstration

### Objekterkennung (YOLOv3)  
![YOLO Object Detection]![image](https://github.com/user-attachments/assets/67f3d23c-4762-4822-a489-80e233cc2c36)

### Pfadplanung mit RRT  
![image](https://github.com/user-attachments/assets/cde0e555-9621-4ee4-8148-f3ca9b65fe1f)

### AprilTag-Lokalisierungsablauf  
![image](https://github.com/user-attachments/assets/aff197d6-ce69-448d-9e1d-6737363f098d)

### Voraussetzungen
- Jetson Nano mit Kameramodul  
- Ubuntu 20.04 + ROS Noetic  
- Python 3.8, PyTorch, OpenCV  
- Docker (für effizientes Training)

## 🧪 Aufgaben & Ergebnisse

| Aufgabe | Beschreibung |
|--------|--------------|
| Aufgabe 1 | Objekterkennung und Erstellung einer 2D-Karte mit YOLO und AprilTags |
| Aufgabe 2 | Navigation um rote (im Uhrzeigersinn) und blaue (gegen den Uhrzeigersinn) Würfel, Hindernisvermeidung |
| Aufgabe 3 | Würfel in farblich passende Zielzonen schieben und zur Ausgangsposition zurückkehren |

> 🏁 Erreichte < 8 cm Fehler bei der Objektlokalisierung und ~2 s Latenz bei der Bildinferenz.

## 🧠 Technische Highlights

- 🤖 **YOLOv3/YOLOv7** mit Roboflow-Vorverarbeitung  
- 🧮 **Perspektivtransformation** zur 3D-Lokalisierung aus 2D-Bilddaten  
- 📐 **Kamerakalibrierung** mit ROS-Tools und Schachbrettmuster  
- 🧭 **Proportionalregelung** für sanfte Navigation  
- 🔄 **TF-Tree & Koordinatentransformationen** mit ROS  

## 🛠 Verwendete Technologien

![Python](https://img.shields.io/badge/Python-3.8-blue)  
![ROS](https://img.shields.io/badge/ROS-Noetic-blue)  
![Jetson Nano](https://img.shields.io/badge/Platform-JetsonNano-green)  
![YOLO](https://img.shields.io/badge/DeepLearning-YOLOv7-brightgreen)  
![RRT](https://img.shields.io/badge/Algorithm-RRT%20%2B%20RRT*-orange)  

## 👨‍💻 Mitwirkende

- **Husam Hamu** – M.Sc. Mechatronik, TU Darmstadt  

[📄 Vollständiger Projektbericht (PDF)](./docs/JetBot_Project_Report.pdf) <!-- Pfad ergänzen, falls hochgeladen -->

## 📜 Lizenz

Dieses Projekt steht unter der MIT-Lizenz – siehe die Datei [LICENSE](LICENSE) für Details.

---

Wenn du magst, kann ich dir das auch als formatiertes PDF oder Markdown-Datei exportieren. Sag einfach Bescheid!
