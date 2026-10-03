"""Controlador Supervisor de la simulación 3D (etapa SIN PID/PI).

La velocidad de la banda sale de la dinámica real del motor definida en
control/transfer_function.py (planta en lazo abierto, comando escalón
constante), simulada paso a paso. Con esa velocidad se anima todo el sistema
descrito en el README:

    1. Motor + encoder: rodillos y disco del encoder giran con la banda.
    2. Sensor de presencia: su LED se enciende cuando hay un producto en el
       pórtico de inspección.
    3. Cámara: destella al capturar y "clasifica" el producto en A/B/C.
    4. Desviador: la paleta gira hacia la salida que le corresponde.
    5. Compartimentos A/B/C: A recto al final de la banda, B a +y, C a -y.
    6. Sensor de verificación: al caer el producto en un compartimento
       compara el compartimento real contra la categoría esperada y enciende
       la lámpara verde (correcto) o roja (error).

La clasificación por visión real (vision/classifier.py) aún está pendiente:
aquí la categoría se toma del color de la caja (rojo=A, verde=B, azul=C).
Para que se vea también la señal roja, cada FAULT_EVERY_N-ésimo producto
sale por un compartimento equivocado (falla del desviador); poner 0 lo
desactiva. Los productos son nodos Solid sin Physics: se animan directo.
"""

import math
import os
import sys

import numpy as np
from scipy import signal

from controller import Supervisor

# Permite importar el paquete control/ del repo (dos niveles por encima de
# simulation/) aunque el controlador corra con su propio cwd en Webots.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from control.transfer_function import build_transfer_function  # noqa: E402

TIME_STEP = 16  # ms, igual al basicTimeStep del mundo
DT = TIME_STEP / 1000.0  # s

# Escala física: una unidad de la salida de T(s) equivale a esta velocidad de
# banda (m/s). La escala es FIJA y está calibrada con la ganancia de referencia
# (Km=18 -> ganancia DC 540 -> 0.25 m/s en régimen). Así, si se cambia Km en
# control/transfer_function.py, la velocidad final de la banda cambia de verdad
# (lazo abierto: nadie la corrige).
REFERENCE_DC_GAIN = 540.0
BELT_SPEED_PER_UNIT = 0.25 / REFERENCE_DC_GAIN
ROLLER_RADIUS = 0.15  # m, para girar rodillos y encoder según la velocidad

BELT_START_X = -1.15
BELT_END_X = 1.15
PRODUCT_Z = 0.37
PRODUCT_DEFS = ["PRODUCT_1", "PRODUCT_2", "PRODUCT_3", "PRODUCT_4", "PRODUCT_5"]
PRODUCT_CATEGORY = {  # color de la caja -> categoría (stand-in de la IA)
    "PRODUCT_1": "A", "PRODUCT_2": "B", "PRODUCT_3": "C",
    "PRODUCT_4": "A", "PRODUCT_5": "B",
}

INSPECTION_X = -0.3  # pórtico: sensor de presencia + cámara
INSPECTION_HALF = 0.06
DIVERTER_X = 0.5
BLADE_ANGLE = {"A": 0.0, "B": 0.7, "C": -0.7}  # rad (B hacia +y, C hacia -y)
BLADE_RATE = 2.0  # rad/s
BIN_POS = {"A": (1.6, 0.0), "B": (0.8, 0.55), "C": (0.8, -0.55)}
BIN_FLOOR_Z = 0.1
SLIDE_SPEED = 0.35  # m/s lateral sobre la paleta
FALL_SPEED = 1.2  # m/s
BELT_HALF_WIDTH = 0.25
RESULT_HOLD_S = 2.0  # cuánto dura la señal verde/roja
RESPAWN_DELAY_S = 2.5  # tiempo en el compartimento antes de reaparecer

FAULT_EVERY_N = 4  
STATUS_PERIOD_S = 1.0

LAMP_ON = {"G": [0.0, 1.0, 0.0], "R": [1.0, 0.0, 0.0]}
OFF = [0.0, 0.0, 0.0]
NEXT_CATEGORY = {"A": "B", "B": "C", "C": "A"}


def make_discrete_motor(dt):
    """Discretiza T(s) del motor (control/transfer_function.py) para simularla
    paso a paso, sincronizada con el timestep de Webots."""
    sys_tf = build_transfer_function()
    A, B, C, D = signal.tf2ss(sys_tf.num, sys_tf.den)
    Ad, Bd, Cd, Dd, _ = signal.cont2discrete((A, B, C, D), dt, method="zoh")
    return Ad, Bd, Cd, Dd


class Product:
    def __init__(self, def_name, node):
        self.name = def_name
        self.node = node
        self.field = node.getField("translation")
        self.expected = PRODUCT_CATEGORY[def_name]
        self.reset()

    def reset(self):
        x, y, _ = self.field.getSFVec3f()
        self.x, self.y, self.z = x, y, PRODUCT_Z
        self.state = "belt"  # belt -> diverting -> in_bin
        self.classified = None  # categoría que decide la cámara
        self.actual = None  # compartimento real
        self.bin_timer = 0.0

    def respawn(self):
        self.field.setSFVec3f([BELT_START_X, self.y, PRODUCT_Z])
        self.x, self.z = BELT_START_X, PRODUCT_Z
        self.state = "belt"
        self.classified = None
        self.actual = None
        self.bin_timer = 0.0
        self.route = None  # salida a la que lo manda el desviador

    def push(self):
        self.field.setSFVec3f([self.x, self.y, self.z])


def set_emissive(node, color):
    node.getField("emissiveColor").setSFColor(color)


def main():
    sup = Supervisor()

    def get(name):
        node = sup.getFromDef(name)
        if node is None:
            print(f"[aviso] no se encontró DEF {name} en el mundo")
        return node

    products = [Product(n, get(n)) for n in PRODUCT_DEFS if get(n) is not None]
    blade = get("BLADE")
    encoder = get("ENCODER")
    roller_l, roller_r = get("ROLLER_L"), get("ROLLER_R")
    sensor_led = get("SENSOR_LED_MAT")
    camera_led = get("CAMERA_LED_MAT")
    lamps = {
        (k, c): get(f"LAMP_{k}_{c}_MAT") for k in "ABC" for c in "GR"
    }
    lamp_timer = {k: 0.0 for k in "ABC"}

    Ad, Bd, Cd, Dd = make_discrete_motor(DT)
    x_state = np.zeros((Ad.shape[0], 1))  # estado interno del motor
    u = np.array([[1.0]])  # comando escalón: motor "encendido"

    roller_angle = 0.0
    blade_angle = 0.0
    camera_flash = 0.0
    classified_count = 0
    elapsed = 0.0
    next_status = 0.0

    while sup.step(TIME_STEP) != -1:
        # --- Motor (lazo abierto): un paso de T(s) en espacio de estados ---
        x_state = Ad @ x_state + Bd @ u
        y_out = float((Cd @ x_state + Dd @ u)[0, 0])
        belt_speed = y_out * BELT_SPEED_PER_UNIT  # m/s reales
        dx = belt_speed * DT

        # --- Rodillos y encoder giran con la banda ---
        roller_angle += belt_speed / ROLLER_RADIUS * DT
        rot = [0.0, 1.0, 0.0, roller_angle % (2 * math.pi)]
        for node in (roller_l, roller_r, encoder):
            if node is not None:
                node.getField("rotation").setSFRotation(rot)

        # --- Sensor de presencia + cámara ---
        present = False
        for p in products:
            if p.state == "belt" and abs(p.x - INSPECTION_X) <= INSPECTION_HALF:
                present = True
                if p.classified is None:
                    # La cámara captura y la "IA" decide la categoría.
                    classified_count += 1
                    p.classified = p.expected
                    camera_flash = 0.4
                    p.route = p.classified
                    if FAULT_EVERY_N and classified_count % FAULT_EVERY_N == 0:
                        p.route = NEXT_CATEGORY[p.route]  # falla del desviador
        if sensor_led is not None:
            set_emissive(sensor_led, [1.0, 0.0, 0.0] if present else OFF)
        camera_flash = max(0.0, camera_flash - DT)
        if camera_led is not None:
            set_emissive(camera_led, [1.0, 1.0, 1.0] if camera_flash > 0 else OFF)

        # --- Desviador: apunta a la salida del producto más adelantado ---
        target = 0.0
        holding = any(p.state == "diverting" and abs(p.y) < BELT_HALF_WIDTH + 0.05
                      for p in products)
        if holding:
            target = blade_angle  # no mover la paleta mientras se despeja
        else:
            ahead = [p for p in products
                     if p.state == "belt" and p.classified
                     and 0.1 < p.x < DIVERTER_X]
            if ahead:
                front = max(ahead, key=lambda p: p.x)
                target = BLADE_ANGLE[front.route]
        step = BLADE_RATE * DT
        blade_angle += max(-step, min(step, target - blade_angle))
        if blade is not None:
            blade.getField("rotation").setSFRotation([0.0, 0.0, 1.0, blade_angle])

        # --- Movimiento de productos ---
        for p in products:
            if p.state == "belt":
                p.x += dx
                if p.x >= DIVERTER_X and p.classified:
                    # La paleta ya apunta a su ruta: B/C se desvían, A sigue recto.
                    if p.route != "A":
                        p.actual = p.route
                        p.state = "diverting"
                if p.state == "belt" and p.x >= BELT_END_X:
                    p.actual = "A"
                    p.state = "diverting"
            elif p.state == "diverting":
                bx, by = BIN_POS[p.actual]
                step_l = SLIDE_SPEED * DT
                if p.actual == "A":
                    p.x += min(step_l, max(0.0, bx - p.x))
                    on_belt = p.x < BELT_END_X + 0.05
                    p.y += max(-step_l, min(step_l, by - p.y))
                else:
                    p.x += max(-step_l, min(0.8 * dx, bx - p.x))
                    p.y += max(-step_l, min(step_l, by - p.y))
                    on_belt = abs(p.y) < BELT_HALF_WIDTH
                if not on_belt:
                    p.z = max(BIN_FLOOR_Z, p.z - FALL_SPEED * DT)
                if (not on_belt and p.z <= BIN_FLOOR_Z
                        and abs(p.y - by) < 0.03 and abs(p.x - bx) < 0.2):
                    p.state = "in_bin"
                    p.bin_timer = 0.0
                    # Sensor de verificación: compara real vs esperado.
                    ok = p.actual == p.expected
                    for c in "GR":
                        set_emissive(lamps[(p.actual, c)],
                                     LAMP_ON[c] if (c == "G") == ok else OFF)
                    lamp_timer[p.actual] = RESULT_HOLD_S
                    print(f"[verificación] {p.name}: esperado={p.expected} "
                          f"real={p.actual} -> {'OK (verde)' if ok else 'ERROR (rojo)'}")
            elif p.state == "in_bin":
                p.bin_timer += DT
                if p.bin_timer >= RESPAWN_DELAY_S:
                    p.y = 0.05 if p.name in ("PRODUCT_2", "PRODUCT_4") else -0.05
                    p.respawn()
            p.push()

        # --- Apagar lámparas tras el tiempo de señal ---
        for k in "ABC":
            if lamp_timer[k] > 0:
                lamp_timer[k] -= DT
                if lamp_timer[k] <= 0:
                    for c in "GR":
                        set_emissive(lamps[(k, c)], OFF)

        elapsed += DT
        if elapsed >= next_status:
            print(f"[motor] t={elapsed:5.1f}s  y={y_out:.3f}  "
                  f"velocidad_banda={belt_speed:.4f} m/s")
            next_status += STATUS_PERIOD_S


if __name__ == "__main__":
    main()
