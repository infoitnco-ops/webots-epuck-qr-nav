# Webots E-puck QR világ

## Tartalom
- `worlds/qr_epuck.wbt` – minimális világ
- `controllers/qr_epuck_controller/qr_epuck_controller.py` – Python controller
- `textures/*.png` – kész QR-kód képek

## Mit csinál?
- Az E-puck lassan körbeforogva QR-kódot keres.
- Ha talál egyet, középre fordul.
- A tartalom alapján reagál:
  - `bal`
  - `jobb`
  - `allj`
  - `elore`
- Egyszerű elsőbbségi akadályelkerülés is benne van.

## Használat
1. Másold be ezt a mappát egy Webots projektként.
2. Nyisd meg a `worlds/qr_epuck.wbt` fájlt.
3. Ellenőrizd, hogy a Webots Python környezetében elérhető a `numpy` és `opencv-python`.
4. Indítsd a szimulációt.

## Megjegyzés
Ha a Webots nálad lefagy, próbáld először:
- kikapcsolni a fölösleges robotablakokat,
- frissíteni a videódrivert,
- újabb Webots verziót használni,
- vagy a kamera felbontását lejjebb venni 160x120-ra a világfájlban.
