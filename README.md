# **ESP32 Digital VLF Metal Detector (Hybrid App)**

[**🇬🇧 English Version**](https://www.google.com/search?q=%23-esp32-digital-vlf-metal-detector) | [**🇩🇪 Deutsche Version**](https://www.google.com/search?q=%23-esp32-digitaler-vlf-metalldetektor)

# **🇬🇧 ESP32 Digital VLF Metal Detector**

A modern, fully digital VLF (Very Low Frequency) metal detector project powered by the ESP32 microcontroller. This project combines high-performance digital signal processing (DSP) on the hardware side with a responsive, web-based user interface running inside an Android App (via MIT App Inventor).
The circuit diagram and test board for this project can be found under the [Project](https://github.com/folny/ESP32_Metal_Detector/tree/master/Eagle8_file) as a source of inspiration.

## **🌟 Key Features**

### **Firmware (ESP32)**

* **VLF/IB Technology:** Induction Balance method with digital phase and magnitude calculation.  
* **Full Digital Processing:**  
  * Precise signal generation using ESP32 **MCPWM** unit.  
  * **Oversampling** and digital filtering (EMA) of ADC inputs.  
  * Vector math to determine **VDI** (Visual Discrimination Indicator) and signal strength.  
* **Bluetooth Low Energy (BLE):** Efficient communication with the smartphone.  
* **Auto-Calibration:** Systems for Ground Balance and Maximum Signal reference.  
* **Hardware Control:** Support for LED Flashlight (via MOSFET) and battery monitoring.

### **Hybrid App & UI (HTML/JS \+ AI2)**

The app is designed as a hybrid application. The logic and UI are written in standard web technologies (HTML/CSS/JS) but hosted within a native Android container.

* **Real-time Visualization:** Chart.js renders live graphs of VDI and Signal Strength.  
* **Metal Discrimination:** Visual categorization (Iron, Foil, Gold, Silver, etc.) with touch-to-disable segments.  
* **Audio Engine:** Tone generation uses the **Web Audio API** (sound.js) to generate lag-free audio directly in the WebView.  
* **Control Center:** Adjust Sensitivity, Sample Rate, Phase Shift, and toggle Flashlight ('M'/'N' commands).

## **🏗️ System Architecture**

This project uses a unique approach to UI development for microcontrollers:

1. **The Host (MIT App Inventor):** The Android app handles the BLE connection to the ESP32. It acts as a wrapper.  
2. **The View (WebViewer):** The app contains a WebViewer component that loads local HTML files (index.html for UI, ton.html for Audio).  
3. **The Bridge:** Data is passed between the App logic (Blocks) and the JavaScript (logic.js) using the WebViewString property.  
   * *ESP32 \-\> BLE \-\> AI2 \-\> WebViewString \-\> JavaScript* (Update UI)  
   * *JavaScript \-\> WebViewString \-\> AI2 \-\> BLE \-\> ESP32* (Send Commands)

## **🛠️ Hardware Setup**

**Core:** ESP32 Development Board

### **Pinout**

| Function | GPIO | Description |
| :---- | :---- | :---- |
| **TX Coil** | 25 | PWM Transmit Signal |
| **RX X-Channel** | 34 | Analog In (In-Phase / Sensor VP) |
| **RX R-Channel** | 35 | Analog In (Quadrature / Sensor VN) |
| **Vref** | 26 | DAC Output (Reference Voltage) |
| **Phase X Ref** | 32 | Demodulator Reference (X) |
| **Phase R Ref** | 33 | Demodulator Reference (R) |
| **Flashlight** | 4 | LED Control (via MOSFET Gate) |
| **Battery** | 39 | Voltage Divider Input |
| **Status LED** | 2 | Onboard LED |

### **Flashlight Circuit**

To control the high-power LED, use an N-Channel Logic-Level MOSFET (e.g., IRLZ44N):

* **Gate:** GPIO 4 (via 100Ω resistor \+ 10kΩ pull-down).  
* **Drain:** LED Cathode (-).  
* **Source:** GND.

## **💻 Installation & Flashing**

### **1\. ESP32 Firmware**

1. Install [Arduino IDE](https://www.arduino.cc/en/software).  
2. Install the **ESP32 Board Package**.  
3. Required Libraries: EEPROM, BLEDevice.  
4. Open VLF\_Metal\_Detector\_ESP32.ino, select your board, and flash.

### **2\. Android App (MIT App Inventor)**

1. Create a project at [MIT App Inventor](https://appinventor.mit.edu/).  
2. **Import Assets:** Upload the following files to the "Media" section of your project:  
   * index.html (Main UI)  
   * ton.html (Audio Engine container)  
   * logic.js (UI Logic)  
   * sound.js (Audio Logic)  
   * chart.js (Graph Library)  
   * Images (Sondeln.png, VDIImage.png)  
3. **WebView Setup:** Set the HomeUrl of your WebViewer component to file:///android\_asset/index.html.  
4. **BLE Logic:** Implement the BLE block logic to read strings from the ESP32 and pass them to WebViewer.WebViewString.

## **📄 License**

MIT License \- Copyright (c) 2025 Anhaltiner01

# **🇩🇪 ESP32 Digitaler VLF Metalldetektor**

Ein modernes, digitales VLF (Very Low Frequency) Metalldetektor-Projekt auf Basis des ESP32 Mikrocontrollers. Dieses Projekt kombiniert leistungsfähige digitale Signalverarbeitung (DSP) auf dem ESP32 mit einer modernen, webbasierten Benutzeroberfläche, die in einer Android-App (via MIT App Inventor) läuft.
Der Stromlaufplan und das Testboard für dieses Projekt ist unter dem [Projekt](https://github.com/folny/ESP32_Metal_Detector/tree/master/Eagle8_file) als Ideengeber zu finden.

## **🌟 Funktionen**

### **Firmware (ESP32)**

* **VLF/IB Technologie:** Basiert auf dem Induktions-Balance-Verfahren mit digitaler Phasen- und Signalauswertung.  
* **Volldigitale Verarbeitung:**  
  * Hochpräzise Signalerzeugung mittels ESP32 **MCPWM**.  
  * **Oversampling** und digitale Filterung (EMA) der ADC-Werte.  
  * Vektor-Berechnung (Magnitude & Phase) zur Bestimmung von Metallart (VDI) und Tiefe.  
* **Bluetooth Low Energy (BLE):** Energiesparende Kommunikation mit dem Smartphone.  
* **Automatische Kalibrierung:**  
  * **Ground Balance:** Abgleich der Bodenmineralisierung.  
  * **Max Signal:** Kalibrierung auf Referenzobjekte.  
* **Hardware-Steuerung:** Unterstützung für eine Beleuchtungs-LED (MOSFET an GPIO 4\) und Batteriemessung.

### **Hybrid App & UI (HTML/JS \+ AI2)**

Die Benutzeroberfläche ist als Hybrid-App konzipiert. Die Logik läuft in Web-Technologies, eingebettet in eine native Android Hülle.

* **Echtzeit-Visualisierung:** Graphische Darstellung von VDI (Leitwert) und Signalstärke mittels Chart.js.  
* **Metall-Diskriminierung:** Visuelle Unterscheidung von Metallen (Eisen, Folie, Gold, Silber, Kupfer) mit anpassbarem Notch-Filter durch Antippen.  
* **Audio Engine:** Erzeugung von Tönen direkt im Webview via **Web Audio API** (sound.js), basierend auf dem erkannten Metall (keine Latenz durch Bluetooth-Audio-Streaming).  
* **Einstellungen:** Fernsteuerung aller Parameter (Sensitivität, Sample Rate, Phase) und Lichtsteuerung ('M'/'N' Befehle).

## **🏗️ Systemarchitektur & App-Erstellung**

Dieses Projekt nutzt einen hybriden Ansatz für die App-Entwicklung:

1. **Der Host (MIT App Inventor):** Die Android-App verwaltet die BLE-Verbindung zum ESP32. Sie dient als Container.  
2. **Die Ansicht (WebViewer):** Die App enthält eine WebViewer-Komponente, die lokale HTML-Dateien lädt (index.html für die UI, ton.html für Audio).  
3. **Die Brücke:** Daten werden zwischen der App-Logik (Blöcke) und dem JavaScript (logic.js) über die Eigenschaft WebViewString ausgetauscht.  
   * *ESP32 sendet Daten \-\> BLE \-\> AI2 \-\> WebViewString \-\> JavaScript* (Update der Charts/Werte)  
   * *JavaScript Button \-\> WebViewString \-\> AI2 \-\> BLE \-\> ESP32* (Senden von Befehlen)

## **🛠️ Hardware Aufbau**

**Kern:** ESP32 Development Board

### **Pinbelegung**

| Funktion | GPIO | Beschreibung |
| :---- | :---- | :---- |
| **TX Spule** | 25 | Sendesignal (PWM) |
| **RX X-Kanal** | 34 | Analog-Eingang In-Phase (Sensor VP) |
| **RX R-Kanal** | 35 | Analog-Eingang Quadratur (Sensor VN) |
| **Vref** | 26 | Referenzspannung (DAC Ausgang) |
| **Phase X Ref** | 32 | Referenzsignal für Demodulator (X) |
| **Phase R Ref** | 33 | Referenzsignal für Demodulator (R) |
| **Beleuchtung** | 4 | LED-Steuerung (via MOSFET Gate) |
| **Batterie** | 39 | Spannungsteiler zur Messung |
| **Status LED** | 2 | Onboard LED (Status/Heartbeat) |

### **Beleuchtungs-Schaltung**

Für die LED-Beleuchtung wird ein N-Channel Logic-Level MOSFET empfohlen (z.B. IRLZ44N):

* **Gate:** an GPIO 4 (mit 100Ω Widerstand \+ 10kΩ Pull-Down gegen GND).  
* **Drain:** an LED Kathode (-).  
* **Source:** an GND.

## **💻 Installation**

### **1\. ESP32 Firmware**

1. Installiere die [Arduino IDE](https://www.arduino.cc/en/software).  
2. Installiere das **ESP32 Board Package** über den Boardverwalter.  
3. Benötigte Bibliotheken: EEPROM, BLEDevice.  
4. Öffne VLF\_Metal\_Detector\_ESP32.ino, wähle das Board und flashe den Code.

### **2\. Android App (MIT App Inventor)**

1. Erstelle ein Projekt in [MIT App Inventor](https://appinventor.mit.edu/).  
2. **Assets Importieren:** Lade folgende Dateien als **Media Assets** hoch:  
   * index.html (Haupt-UI)  
   * ton.html (Audio-Engine Container)  
   * logic.js (Steuerungslogik)  
   * sound.js (Audio Logik)  
   * chart.js (Diagramm-Bibliothek)  
   * Bilder (Sondeln.png, VDIImage.png)  
3. **WebView Konfiguration:** Setze die HomeUrl des WebViewers auf file:///android\_asset/index.html.  
4. **BLE Logik:** Erstelle die Block-Logik, um Strings vom ESP32 zu empfangen und an WebViewer.WebViewString weiterzuleiten.

## **📱 Bedienung**

1. **Start:** Schalte den ESP32 ein und starte die App.  
2. **Verbindung:** Die App verbindet sich über BLE (ESP32 Metal Detector).  
3. **Kalibrierung (Wichtig\!):**  
   * Tab **Calibration** wählen.  
   * Spule in die Luft halten.  
   * **Start Ground Balance** drücken und Spule "pumpen".  
4. **Suche:**  
   * Tab **Status** wählen.  
   * Diagramm beobachten (Orange \= Material, Blau \= Stärke).  
   * **Licht:** Mit den Befehlen in der App (oder Buttons, falls implementiert) kann das Licht an GPIO 4 geschaltet werden.

## **📄 Lizenz**

MIT Lizenz \- Copyright (c) 2025 Anhaltiner01
