import gi
import sys
import time
import serial
import serial.tools.list_ports

# Versión de GStreamer
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import cv2
import numpy as np
from ultralytics import YOLO

# ==========================================
# 1. CONFIGURACIÓN SERIAL (ESP32)
# ==========================================
# TU PUERTO EXACTO (Verificado en tu terminal)
PUERTO_ESP32 = "/dev/cu.usbserial-0001" 
BAUD_RATE = 115200

print(f"=== INICIANDO SISTEMA HÍBRIDO (ETHERNET + USB) ===")
print(f"--> Intentando conectar a ESP32 en: {PUERTO_ESP32}")

esp32 = None

try:
    esp32 = serial.Serial(PUERTO_ESP32, BAUD_RATE, timeout=0.05)
    time.sleep(2) # Esperar a que la ESP32 se reinicie
    print("✅ CONEXIÓN SERIAL EXITOSA: ESP32 Conectada.")
except Exception as e:
    print(f"❌ ERROR CRÍTICO: No se detecta la ESP32 en {PUERTO_ESP32}")
    print(f"   Detalle: {e}")
    print("   (El video funcionará, pero los motores NO se moverán)")

# ==========================================
# 2. LÓGICA DE CONTROL
# ==========================================
def calcular_control(error_x, error_y):
    # Ganancias (Ajusta esto si se mueve muy lento o muy brusco)
    K_slide = 0.6  
    K_yaw   = 0.5  
    K_pitch = 0.5  

    slide = int(K_slide * error_x)
    yaw   = int(K_yaw   * error_x)
    pitch = int(-K_pitch * error_y)

    # Limitar velocidad maxima a 255
    slide = max(min(slide, 255), -255)
    yaw   = max(min(yaw, 255), -255)
    pitch = max(min(pitch, 255), -255)

    return slide, yaw, pitch

def enviar_comandos_serial(slide, yaw, pitch):
    if esp32 is None: return

    # Mensaje: "100,-50,20\n"
    mensaje = f"{slide},{yaw},{pitch}\n"
    
    try:
        # 1. ENVIAR
        esp32.write(mensaje.encode('utf-8'))
        
        # 2. ESCUCHAR CONFIRMACIÓN (Feedback)
        # Esto lee lo que la ESP32 nos grita de vuelta
        if esp32.in_waiting > 0:
            respuesta = esp32.readline().decode('utf-8', errors='ignore').strip()
            if respuesta:
                print(f"    [ESP32 DICE]: {respuesta}") # <--- AQUÍ VERÁS SI RESPONDE
            
    except Exception as e:
        print(f"Error enviando datos: {e}")

# ==========================================
# 3. GSTREAMER PIPELINE
# ==========================================
Gst.init(None)

PIPELINE_STR = """
udpsrc port=5000 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96" !
rtph264depay ! avdec_h264 !
videoconvert ! video/x-raw, format=BGR !
appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true
"""

print("Cargando YOLO...")
model = YOLO("yolov8n.pt") 
print("Modelo cargado.")

try:
    pipeline = Gst.parse_launch(PIPELINE_STR)
    appsink = pipeline.get_by_name("sink")
    pipeline.set_state(Gst.State.PLAYING)
    print("--> Esperando flujo de video UDP en puerto 5000...")
except Exception as e:
    print(f"[ERROR GSTREAMER] {e}")
    sys.exit(1)

# Función captura
def grab_frame():
    try:
        sample = appsink.emit("pull-sample")
        if sample is None: return None
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        h = caps.get_structure(0).get_value("height")
        w = caps.get_structure(0).get_value("width")
        ok, map_info = buffer.map(Gst.MapFlags.READ)
        if not ok: return None
        frame = np.ndarray((h, w, 3), dtype=np.uint8, buffer=map_info.data)
        buffer.unmap(map_info)
        return frame
    except:
        return None

# ==========================================
# 4. BUCLE PRINCIPAL
# ==========================================
print("\n=== SISTEMA CORRIENDO ===")

try:
    while True:
        frame = grab_frame()
        
        if frame is None:
            time.sleep(0.01)
            continue

        results = model(frame, verbose=False)

        person_boxes = []
        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0: 
                    person_boxes.append(box)

        annotated = frame.copy()
        h, w, _ = frame.shape
        cx_img = w // 2
        cy_img = h // 2

        # Centro Imagen
        cv2.rectangle(annotated, (cx_img - 20, cy_img - 20), (cx_img + 20, cy_img + 20), (255, 255, 255), 2)

        if len(person_boxes) > 0:
            # Tomar la persona con más confianza
            best = max(person_boxes, key=lambda b: float(b.conf[0]))
            x1, y1, x2, y2 = best.xyxy[0].cpu().numpy().astype(int)
            cx_obj = (x1 + x2) // 2
            cy_obj = (y1 + y2) // 2

            # Dibujar tracking
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.line(annotated, (cx_img, cy_img), (cx_obj, cy_obj), (0, 255, 255), 1)

            error_x = cx_obj - cx_img
            error_y = cy_obj - cy_img

            # CALCULAR
            s_cmd, y_cmd, p_cmd = calcular_control(error_x, error_y)
            
            # IMPRIMIR DEBUG (Para que veas que está pasando)
            print(f"--> ENVIANDO: S={s_cmd} Y={y_cmd} P={p_cmd}")
            
            # ENVIAR AL ESP32
            enviar_comandos_serial(s_cmd, y_cmd, p_cmd)

            cv2.putText(annotated, f"CMD: S{s_cmd} Y{y_cmd} P{p_cmd}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Si no hay nadie, enviar ceros para frenar
            # print("--> NADIE DETECTADO (Frenando)")
            enviar_comandos_serial(0, 0, 0)
        
        cv2.imshow("Mac YOLO -> ESP32 Serial", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Deteniendo...")
finally:
    pipeline.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()
    if esp32: esp32.close()
