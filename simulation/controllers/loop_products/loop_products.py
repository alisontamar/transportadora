"""Controlador Supervisor para el vistazo inicial en Webots.

Hace que los productos (cajas) reaparezcan al inicio de la banda cuando
llegan al extremo opuesto, para dar el efecto visual de productos
circulando en un ciclo continuo. Es solo un recurso escénico para esta
primera vista 3D: no hay clasificación ni control real todavía (eso vive en
vision/classifier.py y control/transfer_function.py).
"""

from controller import Supervisor

TIME_STEP = 16
BELT_START_X = -1.15
BELT_END_X = 1.15
PRODUCT_HEIGHT_Z = 0.37
PRODUCT_DEFS = ["PRODUCT_1", "PRODUCT_2", "PRODUCT_3", "PRODUCT_4", "PRODUCT_5"]

supervisor = Supervisor()

products = [
    node for node in (supervisor.getFromDef(name) for name in PRODUCT_DEFS)
    if node is not None
]

while supervisor.step(TIME_STEP) != -1:
    for node in products:
        translation_field = node.getField("translation")
        x, y, _ = translation_field.getSFVec3f()
        if x >= BELT_END_X:
            translation_field.setSFVec3f([BELT_START_X, y, PRODUCT_HEIGHT_Z])
            node.resetPhysics()
