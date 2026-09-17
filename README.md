# Sistema de Clasificación de Productos con Visión Artificial y Control de Lazo Cerrado

Proyecto universitario de simulación que integra control automático y visión
artificial para clasificar productos sobre una banda transportadora.

## Descripción del sistema

1. Una banda transportadora mueve productos hacia una zona de inspección.
2. Un **sensor de presencia** y una **cámara** detectan el producto.
3. Un **modelo de IA** clasifica el producto en categoría **A**, **B** o **C**
   a partir de la imagen capturada.
4. Un **controlador PI** regula la velocidad de la banda (motor + encoder),
   modelado como un sistema de segundo orden.
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
│   │   └── transportadora.wbt # Vistazo 3D: banda + productos circulando
│   ├── controllers/
│   │   └── loop_products/     # Hace circular los productos sobre la banda
│   └── __init__.py
├── tests/                 # Pruebas unitarias
│   └── __init__.py
├── main.py                 # Punto de entrada del sistema completo
├── requirements.txt
├── .gitignore
└── README.md
```

### `control/`

Contiene el modelado matemático del lazo de velocidad: la función de
transferencia del sistema motor + encoder + controlador PI, el cálculo de
polos y ceros, y el análisis de la respuesta al escalón. Es el primer módulo
funcional del proyecto.

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
      T(s), calcula polos/ceros y grafica la respuesta al escalón.
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
