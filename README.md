# Sistema de Clasificación de Productos con Visión Artificial y Control de Lazo Cerrado

Proyecto universitario de simulación que integra control automático y visión
artificial para clasificar productos sobre una banda transportadora.

## Descripción del sistema

1. Una banda transportadora mueve productos hacia una zona de inspección.
2. Un **sensor de presencia** y una **cámara** detectan el producto.
3. Un **modelo de IA** clasifica el producto en categoría **A**, **B** o **C**
   a partir de la imagen capturada.
4. Un **motor + encoder** mueve la banda. *(Nota: en esta etapa del proyecto,
   por indicación del docente, no se incluye todavía un controlador PID/PI;
   se analiza el sistema en lazo abierto, con la dinámica natural del motor,
   antes de agregar el controlador que "perfeccionaría" la respuesta.)*
5. Un **desviador** (servomotor) envía el producto al compartimento
   correspondiente según la categoría detectada.
6. Un **sensor de verificación** confirma en qué compartimento terminó el
   producto realmente, compara ese resultado contra la categoría esperada, y
   enciende una señal **verde** (correcto) o **roja** (error). Esta
   comparación cierra el lazo de retroalimentación del sistema.

## Objetivo

Modelar y simular, en un entorno controlado, el comportamiento dinámico de la
banda (lazo de control de velocidad) junto con la lógica de clasificación por
visión artificial, para validar el diseño antes de una eventual
implementación física o en un entorno 3D (Webots).

## Tecnología

- **Python** como lenguaje principal.
- **python-control** (o `scipy.signal`) para el modelado y análisis del lazo
  de control de velocidad (función de transferencia, polos/ceros, respuesta
  al escalón).
- **OpenCV** para el procesamiento de imagen y la futura integración del
  modelo de clasificación.
- **Webots** (integración futura) para la simulación 3D de la banda
  transportadora y sus sensores/actuadores.

## Estructura de carpetas

```
transportadora/
├── control/              # Lazo de control de velocidad de la banda
│   ├── transfer_function.py   # T(s), polos/ceros, respuesta al escalón
│   └── __init__.py
├── vision/               # Procesamiento de imagen y clasificación IA
│   ├── classifier.py          # Clasificación en categoría A/B/C (pendiente)
│   └── __init__.py
├── simulation/            # Integración con Webots (simulación 3D)
│   ├── webots_interface.py    # Puente con el mundo de Webots (pendiente)
│   ├── worlds/
│   │   └── transportadora.wbt # Mundo 3D: banda, motor/encoder, sensor, cámara, desviador, compartimentos A/B/C, señal verde/roja
│   ├── controllers/
│   │   └── loop_products/     # Anima el sistema completo (sin PID/PI) con la T(s) del motor
│   └── __init__.py
├── tests/                 # Pruebas unitarias
│   └── __init__.py
├── main.py                 # Punto de entrada del sistema completo
├── requirements.txt
├── .gitignore
└── README.md
```

### `control/`

Contiene el modelado matemático de la velocidad de la banda: la función de
transferencia del motor (lazo abierto, **sin controlador PID/PI** en esta
etapa), el cálculo de polos y ceros, y el análisis de la respuesta al
escalón. Es el primer módulo funcional del proyecto.

Se deja el controlador fuera a propósito: con PID/PI el sistema corrige el
error de velocidad y responde de forma casi perfecta, lo que oculta la
dinámica real del motor. Sin controlador se ve el comportamiento "crudo"
(más lento, con error de estado estacionario), que sirve como punto de
partida antes de cerrar el lazo.

## Etapa actual: sistema sin controlador PID/PI

En esta etapa del proyecto **no se implementa el
controlador PID/PI**. El objetivo es ver primero cómo se comporta la banda
"cruda" (solo motor + encoder), antes de agregar un controlador que corrija
ese comportamiento y lo vuelva casi perfecto.

### Qué había antes (con PI)

El lazo cerrado incluía un controlador PI (proporcional-integral) que
comparaba la velocidad medida por el encoder contra la velocidad deseada, y
ajustaba la señal al motor para corregir el error:

```
T(s) = Km*(Kp*s + Ki) / (tau_m*s² + (1 + Km*Kp*Ke)*s + Km*Ki*Ke)
```

- `Kp`, `Ki`: ganancias del controlador PI.
- `Ke`: ganancia del encoder (realimentación).
- Sistema de **2do orden** (2 polos): respuesta rápida, sin error de
  velocidad en estado estacionario, porque el controlador corrige
  continuamente el error.

### Qué hay ahora (sin PI)

Se quitó el controlador y el lazo de realimentación que lo acompaña. Lo que
queda es la planta sola: el motor respondiendo directamente a la entrada,
sin nadie corrigiendo su error.

```
T(s) = Km / (tau_m*s + 1)
```

- `Km`: ganancia del motor.
- `tau_m`: constante de tiempo del motor.
- Sistema de **1er orden** (1 solo polo): respuesta más lenta y, si `Km` no
  es exactamente 1, con error de estado estacionario (la banda se acerca a
  la velocidad pedida, pero no la alcanza de forma exacta).

### Cómo se hizo

En `control/transfer_function.py`:

1. Se eliminaron los parámetros del controlador (`Kp`, `Ki`) y el de
   realimentación del encoder (`Ke`) de `build_transfer_function`.
2. Se reemplazó la fórmula de `T(s)` de 2do orden (lazo cerrado con PI) por
   la fórmula de 1er orden `Km / (tau_m*s + 1)` (planta en lazo abierto).
3. El resto del script (`analyze_poles_zeros`, `plot_pole_zero_map`,
   `plot_step_response`) no cambió: siguen funcionando igual porque reciben
   el sistema (`sys`) ya armado y no dependen de si tiene o no controlador.

Para comprobarlo, corriendo `python control/transfer_function.py` ahora se
obtiene 1 solo polo (antes eran 2) y la respuesta al escalón tarda más en
estabilizarse.

Cuando se agregue el controlador PID/PI en una etapa posterior, este mismo
análisis (polos, ceros, respuesta al escalón) va a servir para comparar
"antes vs. después" y mostrar qué tanto mejora el control automático.

### `vision/`

Contendrá el procesamiento de imagen (OpenCV) y el modelo de clasificación
que determina la categoría (A, B o C) de cada producto detectado por la
cámara. Aún no implementado.

### `simulation/`

Contendrá la interfaz de integración con **Webots** para la simulación 3D de
la banda transportadora, sus sensores (presencia, verificación) y
actuadores (motor, servomotor del desviador). `webots_interface.py` (el
puente con `control/` y `vision/`) todavía está pendiente.

Como primer vistazo del entorno 3D ya existe `worlds/transportadora.wbt`:
solo la banda (`ConveyorBelt`) con unas cajas de colores circulando encima
(un controlador `Supervisor` en `controllers/loop_products/` las reubica al
inicio cuando llegan al final, para simular el ciclo). No incluye todavía
sensores, desviador, ni la lógica de clasificación/control del resto del
proyecto — es solo la maqueta visual de la que partirá la integración.

Para abrirlo:

1. Instalar [Webots](https://cyberbotics.com/) (probado con R2025a).
2. Abrir `simulation/worlds/transportadora.wbt` desde Webots.
3. Iniciar la simulación (▶) para ver los productos circulando sobre la banda.

### `tests/`

Pruebas unitarias para los distintos módulos del proyecto.

## Estado actual

- [x] Estructura base del repositorio.
- [x] Módulo de control (`control/transfer_function.py`) funcional: define
      T(s) del motor **sin controlador PID/PI** (lazo abierto), calcula
      polos/ceros y grafica la respuesta al escalón.
- [ ] Controlador PID/PI (se agregará en una etapa posterior).
- [ ] Módulo de visión artificial (clasificación A/B/C).
- [x] Vistazo 3D en Webots: banda transportadora con productos circulando
      (`simulation/worlds/transportadora.wbt`), sin sensores/desviador aún.
- [ ] Integración completa con Webots (`webots_interface.py`, sensores,
      desviador, señal verde/roja).
- [ ] Lazo de verificación y señal verde/roja.

## Uso

Instalar dependencias:

```bash
pip install -r requirements.txt
```

Ejecutar el análisis del lazo de control de velocidad:

```bash
python control/transfer_function.py
``` 