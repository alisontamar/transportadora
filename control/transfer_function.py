"""Función de transferencia de lazo cerrado para el control de velocidad
de la banda transportadora (motor + encoder, controlador PI).

    T(s) = Km*(Kp*s + Ki) / (tau_m*s^2 + (1 + Km*Kp*Ke)*s + Km*Ki*Ke)

Donde:
    Km    : ganancia del motor
    Ke    : ganancia del encoder (realimentación)
    tau_m : constante de tiempo del motor
    Kp,Ki : ganancias proporcional e integral del controlador PI

Este script arma la función de transferencia con scipy.signal, calcula
sus polos y ceros, y grafica la respuesta al escalón.

Nota: se usa scipy.signal (y no la librería "python-control") a propósito,
para evitar que "import control" dentro de este paquete, también llamado
control/, termine resolviendo al propio paquete local en vez de a la
librería de terceros.
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# --- Parámetros del sistema (valores iniciales de referencia) ---
Km = 2.5      # ganancia del motor
Ke = 1.0      # ganancia del encoder
tau_m = 0.5   # constante de tiempo del motor [s]
Kp = 1.2      # ganancia proporcional del PI
Ki = 3.0      # ganancia integral del PI


def build_transfer_function(Km=Km, Ke=Ke, tau_m=tau_m, Kp=Kp, Ki=Ki):
    """Construye T(s) = Km*(Kp*s + Ki) / (tau_m*s^2 + (1 + Km*Kp*Ke)*s + Km*Ki*Ke)."""
    num = [Km * Kp, Km * Ki]
    den = [tau_m, 1 + Km * Kp * Ke, Km * Ki * Ke]
    return signal.TransferFunction(num, den)


def analyze_poles_zeros(sys):
    """Devuelve (polos, ceros) del sistema y los imprime en consola."""
    poles = np.roots(sys.den)
    zeros = np.roots(sys.num)
    print("Polos:", poles)
    print("Ceros:", zeros)
    return poles, zeros


def plot_pole_zero_map(sys):
    """Grafica el mapa de polos y ceros en el plano s."""
    poles, zeros = analyze_poles_zeros(sys)

    plt.figure()
    plt.scatter(poles.real, poles.imag, marker="x", s=100, color="red", label="Polos")
    if len(zeros):
        plt.scatter(
            zeros.real, zeros.imag,
            marker="o", s=100, facecolors="none", edgecolors="blue", label="Ceros",
        )
    plt.axhline(0, color="black", linewidth=0.5)
    plt.axvline(0, color="black", linewidth=0.5)
    plt.title("Mapa de polos y ceros - T(s)")
    plt.xlabel("Re")
    plt.ylabel("Im")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_step_response(sys, t_final=5.0):
    """Grafica la respuesta al escalón unitario del sistema."""
    t = np.linspace(0, t_final, 1000)
    t_out, y_out = signal.step(sys, T=t)

    plt.figure()
    plt.plot(t_out, y_out)
    plt.title("Respuesta al escalón - Control de velocidad de la banda")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Velocidad normalizada")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    sys = build_transfer_function()
    print("Función de transferencia T(s):")
    print(sys)

    analyze_poles_zeros(sys)
    plot_pole_zero_map(sys)
    plot_step_response(sys)


if __name__ == "__main__":
    main()
