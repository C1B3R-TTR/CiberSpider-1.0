"""
Araña procedural que persigue tu cursor por TODAS las pantallas.

Instalación:   pip install PySide6
Ejecutar:      python spider.py
Salir:         Ctrl+C en la terminal, o clic derecho en el icono de la bandeja > Salir
"""
import sys
import math
import cmath
import time
import signal

from PySide6.QtCore import Qt, QTimer, QPointF, QRectF
from PySide6.QtGui import QCursor, QPainter, QColor, QPen, QBrush, QIcon, QPixmap
from PySide6.QtWidgets import QApplication, QWidget, QSystemTrayIcon, QMenu

# ----------------------------- AJUSTES -----------------------------
S = 0.35            # escala de la araña (1.0 = tamaño de tu dibujo original)
MAX_SPEED = 900.0   # velocidad máxima del cuerpo (px/s)
STOP_DIST = 90.0    # distancia al cursor a la que se detiene
STEP_TIME = 0.13    # duración de cada paso (s)
STEP_FRAC = 0.30    # cuánto se estira una pata (fracción de su alcance) antes de dar un paso
JOINT_R = 6         # radio de los puntos rojos
LINE_W = 2          # grosor de las patas
FPS = 60
# --------------------------------------------------------------------

# Cuerpo de tu dibujo: rectángulo 60x220 centrado en (616, 480); cabeza (círculo azul) abajo.
CENTER = complex(616, 480)

# Cada pata: [punto en el cuerpo, articulación 1, 2, 3, punta] en coordenadas de tu imagen.
LEGS_IMG = [
    # izquierda (arriba -> abajo)
    [(586, 395), (536, 339), (476, 219), (376, 119), (156, 79)],
    [(586, 425), (536, 399), (416, 339), (294, 219), (116, 259)],
    [(586, 513), (536, 461), (436, 421), (294, 441), (176, 569)],
    [(586, 535), (516, 529), (476, 639), (376, 559), (216, 721)],
    # derecha (arriba -> abajo)
    [(646, 395), (656, 319), (716, 239), (896, 179), (1076, 259)],
    [(646, 425), (696, 381), (856, 359), (1016, 401), (1156, 521)],
    [(646, 511), (696, 461), (836, 461), (1016, 539), (1116, 659)],
    [(646, 535), (716, 529), (786, 639), (996, 701), (1096, 799)],
]
# Grupos de la marcha: las patas de un grupo se mueven juntas, el otro grupo se queda plantado
GROUPS = [0, 1, 0, 1, 1, 0, 1, 0]


def loc(p):
    """Punto de la imagen -> vector local (complejo) respecto al centro del cuerpo."""
    return complex(p[0], p[1]) - CENTER


def fabrik(joints, lengths, target, root, iters=8):
    """Cinemática inversa FABRIK. Modifica `joints` in situ."""
    n = len(joints)
    if abs(target - root) >= sum(lengths):
        d = target - root
        d = d / abs(d) if abs(d) > 1e-9 else 1
        joints[0] = root
        for i in range(1, n):
            joints[i] = joints[i - 1] + d * lengths[i - 1]
        return
    for _ in range(iters):
        joints[-1] = target
        for i in range(n - 2, -1, -1):
            d = joints[i] - joints[i + 1]
            d = d / abs(d) if abs(d) > 1e-9 else 1
            joints[i] = joints[i + 1] + d * lengths[i]
        joints[0] = root
        for i in range(1, n):
            d = joints[i] - joints[i - 1]
            d = d / abs(d) if abs(d) > 1e-9 else 1
            joints[i] = joints[i - 1] + d * lengths[i - 1]


class Leg:
    def __init__(self, pts, group, body_pos, rot):
        local = [loc(p) for p in pts]
        self.attach = local[0]
        tip = local[-1]
        self.rest = self.attach + (tip - self.attach) * 0.8     # posición de reposo del pie
        self.lengths = [abs(local[i + 1] - local[i]) * S for i in range(len(local) - 1)]
        self.reach = sum(self.lengths)
        self.group = group
        self.joints = [body_pos + rot * (q * S) for q in local]  # pose inicial = tu dibujo
        self.foot = self.joints[-1]
        self.stepping = False
        self.t = 0.0
        self.start = self.end = self.foot


class Spider(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(
            Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.Tool
            | Qt.WindowTransparentForInput | Qt.NoDropShadowWindowHint
        )
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WA_ShowWithoutActivating)

        # Cubrir el escritorio virtual completo (todas las pantallas)
        rect = None
        for sc in QApplication.screens():
            rect = sc.geometry() if rect is None else rect.united(sc.geometry())
        self.setGeometry(rect)
        self.origin = complex(rect.x(), rect.y())

        c = QCursor.pos()
        self.pos_ = complex(c.x(), c.y()) - self.origin
        self.vel = 0j
        self.angle = 0.0
        rot = cmath.exp(1j * self.angle)
        self.legs = [Leg(p, g, self.pos_, rot) for p, g in zip(LEGS_IMG, GROUPS)]

        self.last = time.perf_counter()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(1000 / FPS))

    # ------------------------- lógica -------------------------
    def tick(self):
        now = time.perf_counter()
        dt = min(now - self.last, 0.05)
        self.last = now

        # --- cuerpo: perseguir el cursor ---
        c = QCursor.pos()
        cursor = complex(c.x(), c.y()) - self.origin
        delta = cursor - self.pos_
        dist = abs(delta)
        if dist > STOP_DIST:
            speed = min(MAX_SPEED, 80 + (dist - STOP_DIST) * 3.0)
            target_vel = delta / dist * speed
        else:
            target_vel = 0j
        self.vel += (target_vel - self.vel) * min(1.0, dt * 8)
        self.pos_ += self.vel * dt

        # --- orientación: la cabeza (círculo azul) mira hacia donde camina ---
        if abs(self.vel) > 25:
            target_ang = cmath.phase(self.vel) - math.pi / 2
            diff = (target_ang - self.angle + math.pi) % (2 * math.pi) - math.pi
            self.angle += diff * min(1.0, dt * 6)
        rot = cmath.exp(1j * self.angle)

        # --- patas ---
        for leg in self.legs:
            anchor = self.pos_ + rot * (leg.attach * S)
            rest = self.pos_ + rot * (leg.rest * S)
            thr = leg.reach * STEP_FRAC

            if leg.stepping:
                leg.t += dt / STEP_TIME
                k = min(leg.t, 1.0)
                e = k * k * (3 - 2 * k)
                leg.foot = leg.start + (leg.end - leg.start) * e
                if leg.t >= 1.0:
                    leg.stepping = False
            else:
                d = abs(leg.foot - rest)
                overstretched = abs(leg.foot - anchor) > leg.reach * 0.97
                other_busy = any(o.stepping and o.group != leg.group for o in self.legs)
                if d > thr and (not other_busy or d > thr * 1.8 or overstretched):
                    lead = self.vel * 0.12
                    if abs(lead) > thr:
                        lead *= thr / abs(lead)
                    leg.start = leg.foot
                    leg.end = rest + lead
                    leg.t = 0.0
                    leg.stepping = True

            fabrik(leg.joints, leg.lengths, leg.foot, anchor)

        self.update()

    # ------------------------- dibujo -------------------------
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        white = QColor(235, 235, 235)

        # patas
        p.setPen(QPen(white, LINE_W, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for leg in self.legs:
            for a, b in zip(leg.joints, leg.joints[1:]):
                p.drawLine(QPointF(a.real, a.imag), QPointF(b.real, b.imag))

        # articulaciones
        p.setPen(QPen(white, 1.5))
        p.setBrush(QBrush(QColor(255, 20, 20)))
        for leg in self.legs:
            for j in leg.joints[1:]:
                p.drawEllipse(QPointF(j.real, j.imag), JOINT_R, JOINT_R)

        # cuerpo
        p.save()
        p.translate(self.pos_.real, self.pos_.imag)
        p.rotate(math.degrees(self.angle))
        p.setPen(QPen(white, 2))
        p.setBrush(QBrush(QColor(18, 18, 18)))
        p.drawRect(QRectF(-30 * S, -110 * S, 60 * S, 220 * S))
        p.setPen(QPen(white, 1.5))
        p.setBrush(QBrush(QColor(110, 176, 255)))
        p.drawEllipse(QPointF(0, 79 * S), 21 * S, 21 * S)
        p.restore()


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)   # Ctrl+C cierra la app
    app = QApplication(sys.argv)

    spider = Spider()
    spider.show()

    # icono de bandeja para poder cerrarla (la ventana ignora el mouse)
    pm = QPixmap(32, 32)
    pm.fill(Qt.transparent)
    pp = QPainter(pm)
    pp.setRenderHint(QPainter.Antialiasing)
    pp.setBrush(QColor(255, 20, 20))
    pp.setPen(Qt.white)
    pp.drawEllipse(4, 4, 24, 24)
    pp.end()
    tray = QSystemTrayIcon(QIcon(pm))
    menu = QMenu()
    menu.addAction("Salir", app.quit)
    tray.setContextMenu(menu)
    tray.setToolTip("Araña")
    tray.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
