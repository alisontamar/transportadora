# Guía de exposición – Banda transportadora clasificadora (sin PID)

## 1. Idea en 30 segundos
Una banda lleva productos (cajas de color). Un sensor detecta que llegó uno, una
cámara lo clasifica en **A, B o C**, un desviador lo manda a su compartimento y
un sensor final verifica si cayó en el correcto (luz **verde** = bien, **roja** = error).
La banda la mueve un **motor con encoder**, modelado con una función de
transferencia. **En esta etapa no hay PID**: el motor trabaja en lazo abierto.

## 2. Componentes (lo que se ve en Webots)
| Componente | Qué hace | Cómo se ve |
|---|---|---|
| Motor + encoder | Mueve la banda; el encoder mide giro/velocidad | Cilindro azul + disco negro con marcas amarillas |
| Banda y rodillos | Transportan los productos | Caja gris con rodillos en los extremos |
| Sensor de presencia | Detecta que hay un producto en la zona de inspección | Emisor/receptor en el pórtico, LED rojo |
| Cámara | Captura y clasifica (A/B/C) | Cuelga del pórtico, LED blanco destella |
| Desviador | Gira para mandar el producto a su salida | Paleta amarilla |
| Compartimentos A/B/C | Destinos finales | Cajones rojo (A), verde (B), azul (C) |
| Sensor de verificación | Compara compartimento real vs esperado | Sensor sobre cada cajón + lámpara verde/roja |

Recorrido: **presencia → cámara → desviador → compartimento → verificación**.
A sigue recto al final de la banda, B se desvía a un lado y C al otro.

## 3. El modelo matemático (control/transfer_function.py)
```
T(s) = 54 / (0.1 s + 0.1) = 540 / (s + 1)
```
- Primer orden, **un polo en s = -1**, sin ceros.
- **Constante de tiempo τ = 1 s**: alcanza 63 % de la velocidad final en 1 s y
  ~98 % en 4 s (por eso la banda arranca lenta en la animación).
- Ganancia estática 540; se escala para que la velocidad final sea **0.25 m/s**.
- **Lazo abierto**: se le da una orden fija (escalón = motor encendido) y la
  velocidad queda donde la planta lo determine. Si algo cambia (carga, voltaje),
  **nadie lo corrige**: ese es el error de estado estacionario que el PID
  arreglaría en la siguiente etapa. Por eso no se añade aún: así se ve la
  dinámica "cruda" del motor.

Cómo se conecta con la animación: el controlador de Webots discretiza `T(s)`,
la simula paso a paso cada 16 ms, y la velocidad obtenida mueve los productos,
los rodillos y el encoder.

## 4. Dónde cambiar cosas (por si el ingeniero pregunta)
| Quiero cambiar… | Archivo | Qué tocar |
|---|---|---|
| Ganancia o rapidez del motor | `control/transfer_function.py` | `Km` (ganancia), `tau_m` (constante de tiempo) |
| Orden / forma de la función | `control/transfer_function.py` | `build_transfer_function()` (`num`, `den`) |
| Encender/apagar el motor | `simulation/controllers/loop_products/loop_products.py` | `u = np.array([[1.0]])` (la entrada) |
| Velocidad final de la banda | mismo controlador | `0.25` dentro de `BELT_SPEED_PER_UNIT` |
| Fallas del desviador (luz roja) | mismo controlador | `FAULT_EVERY_N` (0 = sin fallas) |
| Qué categoría es cada caja | mismo controlador | diccionario `PRODUCT_CATEGORY` |
| Posición de desviador, cajones | mismo controlador / `.wbt` | `DIVERTER_X`, `BIN_POS` / nodos en `transportadora.wbt` |

Después de editar, en Webots: **File → Reload World**.

## 5. Preguntas típicas y respuestas

**¿Y si fuera de segundo orden?**
Se añade otra dinámica (p. ej. la eléctrica del motor además de la mecánica).
Forma general: `T(s) = K·ωn² / (s² + 2ζωn·s + ωn²)`.
- ζ ≥ 1: sin oscilación, solo más lento al arrancar (dos polos reales).
- ζ < 1: **sobrepaso y oscilación** de velocidad antes de estabilizarse; se vería
  la banda acelerando de más y corrigiéndose sola.
En el código solo se cambia el denominador, por ejemplo
`den = np.polymul([tau_m, 0.1], [0.05, 1])` (dos polos reales) o
`num=[K*25]`, `den=[1, 2*0.4*5, 25]` (oscilatorio). El controlador de Webots
funciona igual porque se adapta al orden del sistema.

**¿Qué pasa si le quito la velocidad (entrada u = 0 o Km = 0)?**
El motor está apagado: la banda no se mueve, los productos no avanzan, nunca
llegan a la cámara ni al desviador y no se enciende ninguna lámpara. Si la entrada
se pone en 0 *después* de estar andando, la velocidad **decae exponencialmente**
con τ = 1 s (la banda frena suavemente, no de golpe).

**¿Y si reduzco Km a la mitad?**
La velocidad final se reduce a la mitad y **no se compensa**: es lo que hace el
lazo abierto. Con PID se corregiría.

**¿Y si le quito el encoder?**
En esta etapa casi no cambia nada: sin PID nadie usa su medición (solo la mide
y se ve girar). En la etapa con PID sí sería indispensable, porque es el
sensor que cierra el lazo de velocidad.

**¿Y si le quito la cámara?**
Nadie decide la categoría, el desviador no actúa y todo va recto al
compartimento A; los productos B y C dan **luz roja** en la verificación.

**¿Y si le quito el desviador?**
Igual: todo termina en A y los B y C se marcan como error.

**¿Y si le quito el sensor de verificación?**
El sistema clasifica igual, pero ya no hay comprobación ni señal verde/roja:
se pierde la retroalimentación a nivel de proceso (no sabemos si acertó).

**¿Por qué sin PID?**
Para observar primero la respuesta natural del motor (lenta, con error de
estado estacionario) y después mostrar cuánto mejora al añadir el controlador.

**¿La IA es real?**
Aún no: la categoría se toma del color de la caja. Se reemplazará con
`vision/classifier.py` (OpenCV + modelo de clasificación).

## 6. Cierre sugerido
"Modelamos el motor como un sistema de primer orden en lazo abierto, lo
integramos a una simulación 3D con sensor, cámara, desviador y verificación, y
vimos que sin controlador la velocidad depende por completo de la planta. El
siguiente paso es cerrar el lazo con un PID usando el encoder."
