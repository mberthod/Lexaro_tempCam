# Lexaro TempCam — ESP32‑S3 (MLX90640 + Radar + MQTT + WebSocket + Viewer HTTP)

Système de monitoring thermique et radar basé sur ESP32‑S3 (Arduino/PlatformIO), avec:
- MLX90640 (32×24) pour l’imagerie IR
- Radar LD600x (UART 43/44) pour respiration/fréquence cardiaque (trames ASCII‑HEX “AA55…”)
- Capteurs env. I2C: DS3231 (RTC), HDC1080 (T/RH), VEML6030 (lumière)
- Télémetrie WebSocket (JSON) + Viewer HTTP intégré (heatmap en pseudo‑couleurs)
- Publication MQTT (trames JSON allégées)
- Wi‑Fi Manager (portail captif) + CLI USB/WS pour configuration

## Sommaire
- [Matériel requis](#matériel-requis)
- [Câblage (ESP32‑S3)](#câblage-esp32s3)
- [Compilation & flash (PlatformIO)](#compilation--flash-platformio)
- [Fonctionnement](#fonctionnement)
  - [Viewer HTTP (Heatmap)](#viewer-http-heatmap)
  - [WebSocket JSON](#websocket-json)
  - [MQTT (topics & payloads)](#mqtt-topics--payloads)
  - [Commandes WebSocket](#commandes-websocket)
  - [CLI USB (interpréteur)](#cli-usb-interpréteur)
- [Algorithmes et architecture](#algorithmes-et-architecture)
- [Dépannage (FAQ)](#dépannage-faq)
- [Notes MLX90640 (driver officiel)](#notes-mlx90640-driver-officiel)
- [Crédits & licences](#crédits--licences)

---

## Matériel requis
- ESP32‑S3 DevKit (ex. esp32-s3-devkitm-1)
- MLX90640 (32×24), adresse I2C 0x33, pull‑ups 4.7–10 kΩ → 3.3 V (obligatoire)
- Radar LD600x (ex. LD6002H) UART 3.3 V (pas de 5 V)
- DS3231 (RTC), HDC1080 (T/RH), VEML6030 (ALS)
- Micro I2S (WS=GPIO11, SCK/BCK=GPIO12, SD=GPIO13)

## Câblage (ESP32‑S3)
- I2C MLX90640 (bus dédié, rapide):
  - SDA: GPIO9
  - SCL: GPIO10
  - Pull‑ups externes vers 3.3 V: 4.7k–10k (sur SDA et SCL)
  - Alimentation MLX: active‑low (POWER_MLX_PIN LOW = ON)
- I2C Capteurs (bus capteurs):
  - SDA: GPIO48
  - SCL: GPIO47
  - DS3231 @0x68, HDC1080 @0x40, VEML6030 @0x10 (ou @0x48 selon ADD)
- Radar LD600x (UART matériel):
  - RX (ESP reçoit): GPIO43
  - TX (ESP émet): GPIO44
- Micro I2S:
  - WS: GPIO11
  - SCK/BCK: GPIO12
  - SD (in): GPIO13

## Compilation & flash (PlatformIO)
Exemple `platformio.ini` (déjà présent dans le repo):
```ini
[env:esp32-s3-devkitm-1]
platform = espressif32
board = esp32-s3-devkitm-1
framework = arduino
upload_protocol = esptool
upload_speed = 921600
monitor_speed = 115200
build_flags =
  -DARDUINO_USB_MODE=1
  -DARDUINO_USB_CDC_ON_BOOT=1
lib_deps =
  knolleary/PubSubClient
  tzapu/WiFiManager
lib_ldf_mode = deep+
```
Étapes:
1) Brancher via USB natif (CDC) → ESP32‑S3
2) PlatformIO: Build → Upload → Monitor

## Fonctionnement

### Viewer HTTP (Heatmap)
- URL: `http://<IP_ESP>/`
- Rendu 32×24 → 320×240, pseudo‑couleurs avec auto‑échelle (percentiles 5–95%)
- Utilise les trames WebSocket “ir” (voir ci‑dessous)

### WebSocket JSON
- URL WS: `ws://<IP_ESP>/ws`
- Trames typiques (une ligne):
  - IR (heatmap RLE):
    ```json
    {"type":"ir","ts":<us>,"w":32,"h":24,"spot":{"x":12.3,"y":10.2,"t":36.8},"pix":"<RLE count:val;…>"}
    ```
    - RLE “count:val;” (val = T×100). Exemple: `3:3025;5:3010;…`
  - Radar:
    ```json
    {"type":"radar","ts":<us>,"presence":true,"resp_bpm":18.2,"motion":{"rms":0.023}}
    ```
  - Fusion (pistes):
    ```json
    {"type":"fusion","ts":<us>,"tracks":[{"id":1,"x":12.1,"y":10.9,"vx":0.1,"vy":-0.1,"resp_bpm":17.8,"t_body":36.5,"conf":0.82}]}
    ```
  - Événement:
    ```json
    {"type":"event","ts":<us>,"kind":"apnea","id":1,"severity":0.7}
    ```

### MQTT (topics & payloads)
- Base topic configurable (par défaut `lexaro`)
- Topics:
  - `lexaro/ir` → version allégée “ir_sum” (sans heatmap):
    ```json
    {"type":"ir_sum","ts":<us>,"w":32,"h":24,"spot":{"x":12.3,"y":10.2,"t":36.8}}
    ```
  - `lexaro/radar`, `lexaro/fusion`, `lexaro/event` (identiques au WS)
  - `lexaro/test` (commande de test)

### Commandes WebSocket
- Format: texte, monoligne, espaces encodés (`%20`) (ex: `/wifi%20portal`)
- Réponses: logs monoligne encodés (`%20` pour espace)
- Réseau:
  - `/wifi%20set?ssid=MonSSID&pass=MonPass` puis `/wifi%20connect`
  - `/wifi%20portal` (démarre portail captif, arrête WS/HTTP)
  - `/wifi%20portal%20stop` (stop portail, relance WS/HTTP)
  - `/wifi%20portal%20status`
  - `/ws%20restart` | `/ws%20stop` | `/ws%20start`
- MQTT:
  - `/mqtt%20set?host=192.168.0.163&port=1883&topic=lexaro`
  - `/mqtt%20on` → `/mqtt%20connect` → `/mqtt%20status`
  - `/mqtt%20test` (publie `lexaro/test`)
- Réglages:
  - `/set?ir_fps=8&radar_fps=30`
- Aide:
  - `/help`

### CLI USB (interpréteur)
Dans le moniteur série (115200), taper:
- `help` → liste des commandes
- Wi‑Fi: `wifi status`, `wifi connect ssid=… pass=… save=1`, `wifi portal`, `wifi portal stop`
- MQTT: `mqtt status`, `mqtt set host=… port=… user=… pass=… topic=…`, `mqtt on`, `mqtt connect`, `mqtt test`
- Capteurs: `sensors status`, `sensors verbose on|off|status`
- MLX debug: `mlx status`, `mlx verbose on|off`, `mlx probe`, `mlx read reg=0x8000`, `mlx ping`, `mlx speed 100k|400k|1m`, `mlx power`
- Système: `system info`

## Algorithmes et architecture
- Tâches FreeRTOS:
  - IR (8–16 Hz): MLX90640 → calibration → segmentation → centroid/spot → heatmap WS
  - Radar (20–50 Hz): filtrage IIR respiration (bande 0.1–1 Hz) + estimation BPM; parsing trames “AA55…”
  - Fusion (20 Hz): association IR↔radar, suivi 1 piste (EKF simplifié), événements (apnée…)
  - Télémetrie (10 Hz): WS/HTTP (serveur), WS client optionnel, MQTT, commandes
  - UI (1–5 Hz): LED, watchdog, métriques
- Buses:
  - I2C capteurs: SDA=48, SCL=47 (DS3231, HDC1080, VEML6030)
  - I2C MLX: SDA=9, SCL=10 (≥400 kHz; pull‑ups externes requis)
  - I2S: WS=11, SCK=12, SD=13
  - UART Radar: RX=43, TX=44
- Données:
  - Horodatage microsecondes (esp_timer)
  - Buffers circulaires pour synchronisation inter‑tâches

## Dépannage (FAQ)
**WS/HTTP indisponible après portail**
- Envoyer `/wifi%20portal%20stop` puis `/ws%20restart`
- Ouvrir `http://<IP_ESP>/` (pas `https`)

**MQTT “connect refused”**
- Mosquitto 2.x: autoriser `allow_anonymous true` ou créer un `password_file`
- Ouvrir le port 1883 dans le pare‑feu Windows

**MLX90640 “_begin failed”, ping endTx=5**
- Vérifier alimentation (active‑low: LOW = ON), pull‑ups 3.3 V sur SDA/SCL (4.7–10 kΩ), adresse 0x33
- Réduire vitesse: `mlx speed 100k` puis `mlx probe`

**Image IR “fausse”**
- Confirmer que MLX répond (ping=0, `mlx probe` OK), câblage correct, 3.3 V stable
- Vérifier que la page reçoit des trames `{"type":"ir",…}` via WS

## Notes MLX90640 (driver officiel)
- Driver Melexis (API + I2C) intégré localement (voir `src/src/drivers/MLX90640_API.*` et `MLX90640_I2C_Driver.*`)
- Séquence init:
  - `DumpEE` → `ExtractParameters` → `SetChessMode` → `SetRefreshRate(8Hz)`
  - `GetFrameData` → `GetTa` → `CalculateTo` (émissivité 0.95)
- Rappels électriques:
  - I2C 3.3 V (pas 5 V), pull‑ups externes obligatoires
  - Bus dédié (GPIO9/10) conseillé; longueurs de fils ≤20 cm
- Datasheet: MLX90640 (Melexis) [PDF]  
  Voir: MLX90640 datasheet (chap. “Communication protocol”, “Measurement flow”, “Calculate To”)  
  Référence: [MLX90640 Datasheet](https://github.com/melexis/mlx90640-library.git) (voir aussi le lien indiqué en datasheet)

## Crédits & licences
- Firmware Lexaro TempCam: (c) 2025 Lexaro / Hexacore — Tous droits réservés
- MLX90640: (c) Melexis, code API sous licence propre (voir fichiers sources)
- MQTT PubSubClient: (c) Nick O’Leary
- WiFiManager: (c) tzapu et contributeurs

Licence du projet: MIT (à confirmer selon besoins).  
Contributions / issues: ouvrir un ticket sur le repo GitHub.


