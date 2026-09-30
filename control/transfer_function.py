"""Función de transferencia en lazo abierto para la banda transportadora
(motor + encoder, SIN controlador PID/PI).

    T(s) = Km / (tau_m*s + 1)

Donde:
    Km    : ganancia del motor
    tau_m : constante de tiempo del motor

en esta etapa no se incluye el controlador
PID/PI: un controlador cierra el lazo y corrige el error de velocidad,
por lo que "esconde" la dinámica real del motor y hace que el sistema
responda de forma casi perfecta. Sin controlador, la banda responde con
su dinámica natural (primer orden): alcanza la velocidad de forma más
lenta y con el error de estado estacionario propio de un sistema en lazo
abierto, sin corrección.

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
Km = 18    
tau_m = 0.1


def build_transfer_function(Km=Km, tau_m=tau_m):
    num = [(Km)*3]
    den = [tau_m, 0.1]
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
    plt.title("Mapa de polos y ceros - T(s) sin controlador")
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
    plt.title("Respuesta al escalón - Banda sin controlador (lazo abierto)")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Velocidad normalizada")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    sys = build_transfer_function()
    print("Función de transferencia T(s) (sin controlador PID/PI):")
    print(sys)

    analyze_poles_zeros(sys)
    plot_pole_zero_map(sys)
    plot_step_response(sys)


if __name__ == "__main__":
    main()
