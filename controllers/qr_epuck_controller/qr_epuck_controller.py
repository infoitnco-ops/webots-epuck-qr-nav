"""!
@file qr_epuck_controller.py
@brief E-puck robotvezérlő Webots környezethez QR-kód felismeréssel.

A vezérlő feladata:
- a kamera képének olvasása,
- QR-kód detektálása OpenCV segítségével,
- a kiolvasott parancs normalizálása,
- egyszerű navigáció végrehajtása a QR tartalma alapján,
- alap akadályelkerülés a közelségérzékelők felhasználásával.

Támogatott QR-parancsok:
- bal
- jobb
- allj / állj / stop
- elore / előre / forward / continue
"""

from controller import Robot
import math
import os

try:
    import cv2
    import numpy as np
except Exception as exc:
    raise RuntimeError(
        "OpenCV vagy NumPy nem érhető el a Webots Python környezetben. "
        "Telepítsd a cv2 és numpy csomagokat ahhoz a Pythonhoz, amit a Webots használ."
    ) from exc


# ============================================================================
# Konstansok
# ============================================================================

## Szimulációs lépésköz [ms].
TIME_STEP = 64

## Maximális keréksebesség [rad/s].
MAX_SPEED = 6.28

## Keresési állapotban használt forgási sebesség.
SEARCH_SPEED = 1.6

## Előremeneti sebesség.
FORWARD_SPEED = 3.0

## Fordulási sebesség.
TURN_SPEED = 2.2

## Akadályérzékelési küszöb a proximity szenzorokra.
AVOID_THRESHOLD = 80.0

## Végrehajtási idő lépésszámban mérve.
EXECUTION_STEPS = 18  # ~1.1 s 64 ms timestep mellett


# ============================================================================
# Robot és eszközök inicializálása
# ============================================================================

## Webots robot objektum.
robot = Robot()

## Bal kerék motor.
left_motor = robot.getDevice('left wheel motor')

## Jobb kerék motor.
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

## Kamera objektum.
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# Nem használjuk a Webots saját Recognition funkcióját,
# mert itt az OpenCV-s QR-felismerés dolgozik.

## Proximity szenzorok nevei.
ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']

## Proximity szenzor objektumok listája.
ps = []
for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ps.append(sensor)

## OpenCV QR-kód detektor.
qr = cv2.QRCodeDetector()


# ============================================================================
# Állapotváltozók
# ============================================================================

## Aktuális vezérlési állapot.
state = 'SEARCH'

## Utoljára végrehajtott parancs.
last_command = None

## QR-újraolvasási várakozási idő lépésszámban.
cooldown_steps = 0

## A végrehajtási állapotból hátralévő lépések száma.
execute_steps_left = 0

## Az éppen végrehajtott parancs.
current_command = None


# ============================================================================
# Segédfüggvények
# ============================================================================

def clamp_speed(value):
    """!
    @brief Sebesség korlátozása a megengedett tartományra.

    @param value A kívánt keréksebesség.
    @return A levágott, érvényes tartományba eső sebesség.
    """
    return max(-MAX_SPEED, min(MAX_SPEED, value))


def set_speed(left, right):
    """!
    @brief A bal és jobb motor sebességének beállítása.

    A sebességeket a függvény automatikusan levágja a megengedett
    [-MAX_SPEED, MAX_SPEED] tartományra.

    @param left A bal kerék kívánt sebessége.
    @param right A jobb kerék kívánt sebessége.
    """
    left_motor.setVelocity(clamp_speed(left))
    right_motor.setVelocity(clamp_speed(right))


def read_frame():
    """!
    @brief A kamera aktuális képének beolvasása.

    A Webots kamera 4 csatornás képet ad vissza (tipikusan BGRA/RGBA jelleggel).
    A QR-felismerés robusztussága érdekében két különböző színcsatorna-értelmezést
    is előállítunk, majd később mindkettőt kipróbáljuk.

    @return
    - (bgr, rgb) tuple, ha sikerült képet olvasni,
    - None, ha nincs elérhető képkocka.
    """
    width = camera.getWidth()
    height = camera.getHeight()
    raw = camera.getImage()

    if raw is None:
        return None

    frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))

    # A Webots kamera 4 csatornás képet ad; az alpha csatorna eldobható.
    bgr = frame[:, :, :3]
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

    return bgr, rgb


def detect_qr():
    """!
    @brief QR-kód keresése a kamera aktuális képében.

    A függvény több képreprezentációt is kipróbál, mert a csatornasorrend
    platform- és verziófüggő lehet. Az első sikeres dekódolás eredményét adja vissza.

    @return
    Egy 3 elemű tuple:
    - text: a dekódolt nyers szöveg, vagy üres string,
    - points: a QR-kód sarokpontjai, vagy None,
    - image: az a kép, amelyen a feldolgozás történt.
    """
    frames = read_frame()
    if frames is None:
        return "", None, None

    candidates = []

    for image in frames:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.shape[2] == 3 else image
        text, points, _ = qr.detectAndDecode(gray)
        candidates.append((text, points, image))

    for text, points, image in candidates:
        if text:
            return text.strip(), points, image

    return "", None, frames[0]


def normalize_command(text):
    """!
    @brief A beolvasott QR-szöveg normalizálása belső parancsformára.

    Többféle nyelvi és ékezetes/ékezet nélküli alakot ugyanarra a belső
    parancsra képezünk le.

    @param text A QR-kódból dekódolt szöveg.
    @return
    - 'bal'
    - 'jobb'
    - 'allj'
    - 'elore'
    - vagy None, ha a szöveg nem ismert parancs.
    """
    t = text.strip().lower()
    mapping = {
        'bal': 'bal',
        'left': 'bal',
        'jobb': 'jobb',
        'right': 'jobb',
        'stop': 'allj',
        'állj': 'allj',
        'allj': 'allj',
        'elore': 'elore',
        'előre': 'elore',
        'forward': 'elore',
        'continue': 'elore',
        'folytatás': 'elore',
        'folytatas': 'elore',
    }
    return mapping.get(t)


def qr_center_x(points):
    """!
    @brief A QR-kód vízszintes középpontjának meghatározása.

    @param points A QR-kód sarokpontjai.
    @return A QR-kód középpontjának x koordinátája pixelben, vagy None.
    """
    if points is None:
        return None

    pts = np.array(points).reshape(-1, 2)
    return float(np.mean(pts[:, 0]))


def obstacle_state():
    """!
    @brief Az akadályhelyzet kiértékelése a proximity szenzorok alapján.

    A szenzorokat egyszerű logikai csoportokba rendezzük:
    - bal oldali akadály
    - jobb oldali akadály
    - frontális akadály

    @return Egy 4 elemű tuple:
    - left_obstacle: bal oldali akadály van-e,
    - right_obstacle: jobb oldali akadály van-e,
    - front_obstacle: elöl akadály van-e,
    - values: a nyers szenzorértékek listája.
    """
    values = [s.getValue() for s in ps]

    right_obstacle = (
        values[0] > AVOID_THRESHOLD or
        values[1] > AVOID_THRESHOLD or
        values[2] > AVOID_THRESHOLD
    )

    left_obstacle = (
        values[5] > AVOID_THRESHOLD or
        values[6] > AVOID_THRESHOLD or
        values[7] > AVOID_THRESHOLD
    )

    front_obstacle = (
        values[7] > AVOID_THRESHOLD or
        values[0] > AVOID_THRESHOLD or
        values[1] > AVOID_THRESHOLD or
        values[6] > AVOID_THRESHOLD
    )

    return left_obstacle, right_obstacle, front_obstacle, values


def start_execute(command):
    """!
    @brief Parancsvégrehajtási állapot indítása.

    A függvény beállítja az állapotgépet EXECUTE módba, eltárolja az aktuális
    parancsot, és aktivál egy rövid cooldown időt, hogy ugyanaz a QR-kód
    ne aktiválódjon újra azonnal.

    @param command A végrehajtandó belső parancs ('bal', 'jobb', 'allj', 'elore').
    """
    global state, current_command, execute_steps_left, cooldown_steps, last_command

    state = 'EXECUTE'
    current_command = command
    execute_steps_left = EXECUTION_STEPS
    cooldown_steps = 25
    last_command = command

    print(f"QR parancs: {command}")


# ============================================================================
# Fő vezérlési ciklus
# ============================================================================

while robot.step(TIME_STEP) != -1:
    left_obs, right_obs, front_obs, values = obstacle_state()

    if front_obs:
        # Egyszerű elsőbbségi akadályelkerülés.
        if left_obs and not right_obs:
            set_speed(TURN_SPEED, -TURN_SPEED)
        else:
            set_speed(-TURN_SPEED, TURN_SPEED)
        continue

    if cooldown_steps > 0:
        cooldown_steps -= 1

    if state == 'EXECUTE':
        if current_command == 'bal':
            set_speed(-TURN_SPEED, TURN_SPEED)
        elif current_command == 'jobb':
            set_speed(TURN_SPEED, -TURN_SPEED)
        elif current_command == 'allj':
            set_speed(0.0, 0.0)
        elif current_command == 'elore':
            set_speed(FORWARD_SPEED, FORWARD_SPEED)
        else:
            set_speed(0.0, 0.0)

        execute_steps_left -= 1
        if execute_steps_left <= 0:
            state = 'SEARCH'
            current_command = None

        continue

    raw_text, points, image = detect_qr()
    cmd = normalize_command(raw_text) if raw_text else None

    if cmd:
        if cooldown_steps == 0:
            center_x = qr_center_x(points)
            width = camera.getWidth()
            error = 0.0 if center_x is None else (center_x - width / 2.0)

            if abs(error) > width * 0.10:
                # Középre forgatás, mielőtt végrehajtjuk a parancsot.
                gain = 0.015
                turn = max(-2.0, min(2.0, gain * error))
                set_speed(turn, -turn)
            else:
                start_execute(cmd)
        else:
            # Cooldown alatt nem hajtunk végre új parancsot.
            set_speed(0.0, 0.0)
    else:
        # SEARCH állapot: lassú körbeforgás QR kereséshez.
        set_speed(SEARCH_SPEED, -SEARCH_SPEED)