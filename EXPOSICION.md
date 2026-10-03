# Guía de exposición – Banda transportadora clasificadora (sin PID)

## 1. Idea en 30 segundos
Una banda lleva productos (cajas de color). Un sensor detecta que llegó uno, una
cámara lo clasifica en **A, B o C**, un desviador lo manda a su compartimento y
un sensor final verifica si cayó en el correcto (luz **verde** = bien, **roja** = error).
La banda la mueve un **motor con encoder**, modelado con una función de
transferencia. **En esta etapa no hay PID**: el motor trabaja en lazo abierto.

## 2. Componentes (lo que se ve en Webots)
| Componente            | Qué hace                                       | Cómo se ve                                          |
|-----------------------|------------------------------------------------|-----------------------------------------------------|
| Motor + encoder       | Mueve la banda; el encoder mide giro/velocidad | Cilindro azul + disco negro con marcas amarillas    |
| Banda y rodillos      | Transportan los productos                      | Caja gris con rodillos en los extremos              |
| Sensor de presencia   | Detecta que un produt en la zona de inspección | Emisor/receptor en el pórtico, LED rojo             |
| Cámara                | Captura y clasifica (A/B/C)                    | Cuelga del pórtico, LED blanco destella             |
| Desviador             | Gira para mandar el producto a su salida       | Paleta amarilla                                     | 
| Compartimentos A/B/C  | Destinos finales                               | Cajones rojo (A), verde (B), azul (C)               |
| Sensor de verificación| Compara compartimento real vs esperado         | Sensor sobre cada cajón + lámpara verde/roja        |

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
| Orden / forma de la función | `control/transfer_function.py` | líneas 88-89, `num` y `den` (ver sección 5.1) |
| Encender/apagar el motor | `simulation/controllers/loop_products/loop_products.py` | `u = np.array([[1.0]])` (la entrada) |
| Velocidad final de la banda | mismo controlador | `0.25` dentro de `BELT_SPEED_PER_UNIT` |
| Fallas del desviador (luz roja) | mismo controlador | `FAULT_EVERY_N` (0 = sin fallas) |
| Qué categoría es cada caja | mismo controlador | diccionario `PRODUCT_CATEGORY` |
| Posición de desviador, cajones | mismo controlador / `.wbt` | `DIVERTER_X`, `BIN_POS` / nodos en `transportadora.wbt` |

Después de editar, en Webots: **File → Reload World**.

## 5. Preguntas típicas y respuestas

**¿Y si fuera de segundo o tercer orden?**
Se añade otra dinámica (p. ej. la eléctrica del motor además de la mecánica).
- Polos reales: sin oscilación, solo arranca más lento.
- Polos complejos (ζ < 1): **sobrepaso y oscilación** de velocidad antes de estabilizarse.
Solo se cambian `num` y `den` (una o dos líneas): ver el paso a paso en la sección 5.1.

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

## 5.1 Cómo cambiar a 2.º o 3.er orden (paso a paso, con líneas)

Solo hay que tocar **`control/transfer_function.py`, líneas 88-89** (dentro de
`build_transfer_function`). No hay que tocar nada más: el controlador de Webots
(`loop_products.py`, líneas 88-89: `tf2ss` y `cont2discrete`) toma `num` y `den`
y se adapta solo al orden. *(Los números de línea son los actuales; si agregas
o quitas líneas, búscalas por el texto.)*

Hoy (1.er orden):
```python
88    num = [(Km)*3]
89    den = [tau_m, 0.1]          # 0.1*s + 0.1   -> polo en -1
```

**Opción A – 2.º orden, dos polos reales (sin oscilar).** Reemplaza la línea 89:
```python
89    den = np.polymul([tau_m, 0.1], [0.05, 1])   # (0.1s+0.1)(0.05s+1)
```
Polos en -1 y -20; el segundo polo (0.05 s) es la dinámica extra. Mantiene la
ganancia estática en 540, así que la velocidad final sigue en 0.25 m/s; solo
cambia la forma del arranque (más suave al inicio).

**Opción B – 2.º orden oscilatorio (ωn = 5, ζ = 0.4).** Reemplaza las líneas 88 y 89:
```python
88    num = [540*25]                # K*wn^2, con K=540
89    den = [1, 2*0.4*5, 25]        # s^2 + 2*zeta*wn*s + wn^2
```
Polos complejos en -2 ± 4.58j; sobrepaso de ~25 % (la velocidad llega a ~677 antes
de asentarse en 540). En la animación se ve la banda pasándose y corrigiéndose.
Ojo: en esta opción `Km` y `tau_m` ya no se usan; los parámetros son `K`, `wn` y `zeta`.

**Opción C – 3.er orden.** Reemplaza la línea 89:
```python
89    den = np.polymul(np.polymul([tau_m, 0.1], [0.05, 1]), [0.02, 1])
```
Polos en -1, -20 y -50. Es el mismo motor con dos dinámicas extra (p. ej. eléctrica
y del sensor); arranque aún más suave, velocidad final igual.

**Comprobarlo:** ejecutar `python -m control.transfer_function` (mapa de polos y
escalón) y en Webots **File → Reload World**.

**Dos detalles para explicar si preguntan**
- La escala de m/s (`loop_products.py`, línea 50: `REFERENCE_DC_GAIN = 540.0`) está
  calibrada con esa ganancia estática. Si cambias `num`/`den` y la ganancia deja de ser
  540, la velocidad final cambia proporcionalmente (lazo abierto, nadie la corrige).
- Orden = grado del denominador = número de polos = número de estados del motor.

## 6. Cierre sugerido
"Modelamos el motor como un sistema de primer orden en lazo abierto, lo
integramos a una simulación 3D con sensor, cámara, desviador y verificación, y
vimos que sin controlador la velocidad depende por completo de la planta. El
siguiente paso es cerrar el lazo con un PID usando el encoder."

Investigar que significa en el sistema un polo y un cero
como cambiaria el sistema con el 3er grado
tomar en cuenta distintos tamaños de cajas volumen y peso
cuales son las salidas y entradas
clasificacion, la variable retroalimentada