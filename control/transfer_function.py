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

import matplotlib.pyplot as plt  # para dibujar las gráficas
import numpy as np                # cálculo numérico (raíces, vectores)
from scipy import signal          # funciones de transferencia y respuesta al escalón

# --- Parámetros del sistema (valores iniciales de referencia) ---
Km = 18      # [NOTA-1] ganancia del motor
tau_m = 0.1  # [NOTA-2] constante de tiempo del motor

# =====================================================================
# NOTAS PARA LA EXPOSICIÓN (qué tocar si el ingeniero pide un cambio)
# ---------------------------------------------------------------------
# T(s) = 3*Km / (tau_m*s + 0.1)   [ver build_transfer_function()]
#   -> polo      = -0.1 / tau_m            (con los valores actuales: -1)
#   -> constante de tiempo = 10 * tau_m    (actual: 1 s)
#   -> ganancia estática   = 30 * Km       (actual: 540)
#
# [NOTA-1] CAMBIAR LA VELOCIDAD FINAL  ->  variable `Km`
#     Km = 9   -> la banda termina a la mitad de velocidad (~0.125 m/s)
#     Km = 36  -> al doble (~0.5 m/s)
#     No se corrige solo: es lazo abierto (por eso luego se añade PID).
#     En Webots funciona porque la escala es fija
#     (REFERENCE_DC_GAIN = 540 en loop_products.py, calibrada con Km=18).
#
# [NOTA-2] HACERLO MÁS RÁPIDO / MÁS LENTO AL ARRANCAR  ->  variable `tau_m`
#     tau_m = 0.05 -> arranca el doble de rápido (tau = 0.5 s)
#     tau_m = 0.3  -> más lento y "pesado" (tau = 3 s)
#     Cambia SOLO el tiempo de respuesta; la velocidad final no cambia.
#
# [NOTA-3] QUITAR LA VELOCIDAD / APAGAR EL MOTOR
#     - Km = 0            -> num = [0]: la banda nunca se mueve.
#     - En Webots: loop_products.py, línea `u = np.array([[1.0]])`
#       (la entrada). u = 0 -> motor apagado. Si se pasa a 0 estando en
#       marcha, la banda frena suavemente (decae con la constante de tiempo).
#
# [NOTA-4] PASAR A SEGUNDO ORDEN  ->  cambiar `den` en build_transfer_function()
#     Dos polos reales (más lento al arrancar, sin oscilar):
#         den = np.polymul([tau_m, 0.1], [0.05, 1])
#     Oscilatorio (sobrepaso; wn=5, zeta=0.4, misma ganancia estática 540):
#         num = [540*25],  den = [1, 2*0.4*5, 25]
#     El controlador de Webots se adapta solo al orden.
#
# [NOTA-5] AÑADIR OTRA DINÁMICA / UN CERO
#     Un cero: num = [a, b]  (p. ej. num = [Km*3, 1]). Verlo con el mapa
#     de polos y ceros (plot_pole_zero_map).
#
# [NOTA-6] ¿DÓNDE SE VE EL ENCODER?
#     Es solo el disco que gira en Webots (ENCODER en loop_products.py).
#     Sin PID no se usa para nada: no afecta la dinámica. Con PID sería
#     la realimentación de velocidad.
#
# [NOTA-7] ESCALA m/s EN WEBOTS  ->  loop_products.py: BELT_SPEED_PER_UNIT
#     (0.25 = velocidad final en m/s con Km=18).
#
# Después de cualquier cambio: correr este archivo (python -m
# control.transfer_function) para ver la gráfica y, en Webots,
# File -> Reload World.
# =====================================================================


def build_transfer_function(Km=Km, tau_m=tau_m):  # arma T(s); usa Km y tau_m por defecto
    # [NOTA-4] Aquí se cambia el orden / forma: num (numerador) y den (denominador)
    num = [(Km)*3]        # numerador: 3*Km (ganancia del motor, sin s)
    den = [tau_m, 0.1]    # denominador: tau_m*s + 0.1 (polinomio de primer orden)
    return signal.TransferFunction(num, den)  # objeto T(s) de scipy


def analyze_poles_zeros(sys):  # calcula y muestra polos y ceros
    """Devuelve (polos, ceros) del sistema y los imprime en consola."""
    poles = np.roots(sys.den)   # polos = raíces del denominador
    zeros = np.roots(sys.num)   # ceros = raíces del numerador (aquí no hay)
    print("Polos:", poles)      # muestra los polos en consola
    print("Ceros:", zeros)      # muestra los ceros en consola
    return poles, zeros         # los devuelve para graficarlos


def plot_pole_zero_map(sys):  # dibuja el mapa de polos y ceros
    """Grafica el mapa de polos y ceros en el plano s."""
    poles, zeros = analyze_poles_zeros(sys)  # obtiene polos y ceros

    plt.figure()  # nueva ventana de gráfica
    # polos como "x" rojas
    plt.scatter(poles.real, poles.imag, marker="x", s=100, color="red", label="Polos")
    if len(zeros):  # solo dibuja ceros si existen
        plt.scatter(
            zeros.real, zeros.imag,
            marker="o", s=100, facecolors="none", edgecolors="blue", label="Ceros",  # ceros como "o" azules
        )
    plt.axhline(0, color="black", linewidth=0.5)  # eje horizontal (Re)
    plt.axvline(0, color="black", linewidth=0.5)  # eje vertical (Im)
    plt.title("Mapa de polos y ceros - T(s) sin controlador")  # título
    plt.xlabel("Re")      # etiqueta eje X: parte real
    plt.ylabel("Im")      # etiqueta eje Y: parte imaginaria
    plt.legend()          # leyenda (polos/ceros)
    plt.grid(True)        # cuadrícula
    plt.tight_layout()    # ajusta márgenes
    plt.show()            # muestra la gráfica


def plot_step_response(sys, t_final=5.0):  # dibuja la respuesta al escalón; t_final = segundos a simular
    """Grafica la respuesta al escalón unitario del sistema."""
    t = np.linspace(0, t_final, 1000)          # 1000 instantes entre 0 y t_final
    t_out, y_out = signal.step(sys, T=t)       # respuesta al escalón: y_out = velocidad

    plt.figure()  # nueva ventana de gráfica
    plt.plot(t_out, y_out)  # velocidad vs tiempo
    plt.title("Respuesta al escalón - Banda sin controlador (lazo abierto)")  # título
    plt.xlabel("Tiempo [s]")              # etiqueta eje X
    plt.ylabel("Velocidad normalizada")   # etiqueta eje Y
    plt.grid(True)        # cuadrícula
    plt.tight_layout()    # ajusta márgenes
    plt.show()            # muestra la gráfica


def main():  # programa principal: corre todo el análisis
    sys = build_transfer_function()  # arma T(s) con Km y tau_m
    print("Función de transferencia T(s) (sin controlador PID/PI):")  # encabezado
    print(sys)  # imprime T(s) (numerador y denominador)

    analyze_poles_zeros(sys)    # imprime polos y ceros
    plot_pole_zero_map(sys)     # gráfica de polos y ceros
    plot_step_response(sys)     # gráfica de la respuesta al escalón


if __name__ == "__main__":  # solo corre main() si se ejecuta este archivo directamente
    main()
