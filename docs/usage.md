# spot — ghid SM5 standalone

Proiectul păstrează runner-ul și solverul din pachetul SM5 verificat 1.1.0-crawl. Versiunea curentă este 1.1.1. Nu include o integrare ROS executabilă.

## 1. Prima pornire, fără motoare

Clonează repository-ul sau descarcă arhiva sa de pe GitHub. Intră în folderul care conține `runner_gait_ikfin.py`. Nu copia un mediu virtual între Windows și Raspberry Pi.

Pe Windows, urmează [README](../README.md). Pe Linux, pentru simulare:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python runner_gait_ikfin.py --check
python runner_gait_ikfin.py --sim
```

Pentru hardware pe Pi, folosește mediul separat de la secțiunea 5. Modul implicit și importul modulelor nu deschid I²C. `--sim` afișează o animație cinematică Matplotlib; nu simulează frecarea, contactele sau căderea. În SSH fără afișaj poți folosi `--dry-run` sau `--render`.

## 2. Testează direcțiile fără hardware

```bash
python runner_gait_ikfin.py --dry-run --vx 0.003 --duration 12
python runner_gait_ikfin.py --sim --vx -0.003 --duration 12
python runner_gait_ikfin.py --sim --vy 0.003 --duration 12
python runner_gait_ikfin.py --sim --wz 0.015 --duration 12
python runner_gait_ikfin.py --sim --vx 0.002 --vy 0.001 --wz 0.01
python runner_gait_ikfin.py --dry-run --vx 0 --duration 3
```

| Parametru | Semn pozitiv | Unitate |
| --- | --- | --- |
| `--vx` | înainte | metri/secundă |
| `--vy` | stânga | metri/secundă |
| `--wz` | rotație antiorară, privit de sus | radiani/secundă |

Vitezele sunt comenzi pentru model, nu viteze măsurate ale robotului. Comenzile
mari sunt reduse proporțional astfel încât lungimea pasului să rămână în limita
configurată; raportul dintre translație și rotație se păstrează. Nu există turbo.

`--duration` este durata comenzii de mers. După aceasta, piciorul ridicat aterizează,
picioarele sunt readuse pe rând la poziția nominală și corpul revine la centru.
Această oprire controlată poate dura încă un ciclu sau mai mult. **Space nu este
oprire de urgență. X/Ctrl+C solicită dezactivarea imediată a ieșirii.**

Gaitul livrat este **crawl: cel mult un picior ridicat**, în ordinea FL, RR, FR, RL.
Versiunea veche numea „TRIPOD” mișcarea în perechi diagonale, care era un trot.
Acest pachet nu pretinde că include un trot/gallop validat; stabilizează întâi crawl.

## 3. Tastatură

```bash
python runner_gait_ikfin.py --dry-run --keyboard --duration 120
```

| Tastă | Acțiune |
| --- | --- |
| W / S sau săgeți sus / jos | înainte / înapoi |
| A / D sau săgeți stânga / dreapta | lateral stânga / dreapta |
| Q / E | rotație stânga / dreapta |
| Space | oprire normală, cu aterizare și revenire |
| X sau Ctrl+C | oprire de urgență; se încearcă tăierea alimentării |
| Esc | oprire normală, apoi închidere |
| 2 | `CROUCH_READY`, numai după oprirea mersului |
| 3 | `STAND`; mersul este permis numai din această poziție |
| 1 / 4 | `LIE` / `UP_READY`: respinse dacă depășesc limitele |

Ține apăsată tasta direcției. Un terminal nu raportează sigur eliberarea tastelor;
comenzile expiră după 0,65 s fără repetări. Dacă autorepetarea sistemului tău este
mai lentă, mersul poate începe și se poate opri. Pierderea tastaturii cere oprire
normală, nu garantează tăierea instantanee a curentului. Testele grafice `--sim`
redau o secvență calculată și nu acceptă `--keyboard`; comenzile din tabel sunt
pentru modul cu terminal.

## 4. Ce s-a păstrat din fișierele tale

Maparea de referință păstrată din runner-ul standalone verificat:

| Picior | Umăr `sh` | Coapsă `up` | Gambă `lo` |
| --- | --- | --- | --- |
| FL — față stânga | 15 | 14 | 13 |
| FR — față dreapta | 3 | 2 | 1 |
| RL — spate stânga | 11 | 10 | 9 |
| RR — spate dreapta | 7 | 6 | 5 |

`config/poses.json` este copia nemodificată a fișierului tău. `STAND` reproduce
exact cele 12 valori originale. `cal.json` nu este adăugat ca al doilea offset:
pozițiile tale conțin deja unghiuri absolute și nu există suficiente date pentru
a justifica aplicarea încă unei calibrări peste ele.

Geometria inițială provine din `ik_fin.py`: corp 252,05 × 105,577 mm și segmente
45 / 111,5 / 155 mm. **Acestea sunt valori din cod, nu măsurători confirmate.**
Sensurile servourilor sunt păstrate din aceeași versiune; nu sunt confirmate fizic.

Limitele de 15°–165° din `config/robot.json` sunt o restricție software provizorie,
nu limita mecanică reală a fiecărei articulații. Pozițiile vechi `LIE` și `UP_READY`
cer 0° sau 180° pentru unele motoare și sunt refuzate cu această configurație.
Nu am înlocuit valorile lor cu poziții inventate și nu le execut automat la pornire
sau în timpul unei erori. `CROUCH_READY` trece verificarea numerică a limitelor,
dar și ea trebuie verificată mecanic cu robotul sprijinit.

Impulsurile inițiale 750–2250 µs corespund valorilor implicite folosite de vechiul
driver ServoKit, nu celor 500–2500 µs din CSV. Valoarea „180°” este domeniul de
comandă al bibliotecii și nu dovedește o rotație mecanică de 180°.
[Documentația Adafruit](https://docs.circuitpython.org/projects/motor/en/latest/api.html)
explică necesitatea verificării domeniului real și riscul atingerii opritoarelor.

## 5. Înainte de orice test cu motoare

1. Sprijină robotul astfel încât să nu cadă și picioarele să nu atingă masa.
   Păstrează o întrerupere fizică a alimentării la îndemână, independentă de Python.
2. Verifică fiecare canal și sensul fiecărui motor, cu deplasări mici față de o
   poziție cunoscută. Nu face o baleiere automată între 0° și 180°.
3. Măsoară segmentele și poziția nominală a labelor; actualizează geometria.
4. Stabilește separat `min_deg`, `max_deg`, `min_pulse_us`, `max_pulse_us` și
   `max_rate_deg_s` pentru fiecare servo. Sensul `direction` inversează numai
   variația față de `STAND`; `offset_deg` se aplică o singură dată.
5. Verifică separat releul ESP32: bus 1, adresă 0x08, registru 0x01,
   ON `[0x02]`, OFF `[0x01]`. Aceste mesaje provin din codul tău; efectul electric
   trebuie verificat. Un releu blocat sau o comandă I²C eșuată poate lăsa motoarele
   alimentate, chiar dacă programul încearcă oprirea.
6. Numai după verificările reale, marchează în `config/robot.json`
   `geometry.verified_on_robot` și `hardware.calibration_verified_on_robot` cu
   `true`. Aceste marcaje consemnează verificarea ta; nu o efectuează automat.

Până atunci, `--hardware` se oprește **înainte de importarea driverelor și înainte
de deschiderea I²C**. Nu ai nevoie să activezi aceste marcaje pentru simulator.

Instalează dependențele pentru hardware numai pe Raspberry Pi. Un mediu `.venv`
creat implicit nu vede `python3-lgpio` instalat prin apt. Pe Pi 5, Blinka are
nevoie de acest modul la importarea ServoKit, înainte de accesul la PCA9685.

Din folderul proiectului, folosește Python-ul distribuției și un mediu separat.
Oprește-te dacă oricare verificare eșuează; nu continua la comanda cu motoare.
Aceste comenzi nu schimbă fișierele de calibrare și nu pornesc runner-ul hardware:

```bash
sudo apt install python3-venv python3-lgpio
/usr/bin/python3 -c "import sys, lgpio; print(sys.executable, lgpio.__file__)"
/usr/bin/python3 -m venv --system-site-packages .venv-rpi-system
.venv-rpi-system/bin/python -m pip install -r requirements-hardware.txt
.venv-rpi-system/bin/python -c "import lgpio; from adafruit_servokit import ServoKit; from smbus2 import SMBus; print('IMPORTURI OK')"
source .venv-rpi-system/bin/activate
python runner_gait_ikfin.py --check
```

Folosește un nume nou dacă `.venv-rpi-system` aparține deja altei instalări.
La pornirile următoare activează același mediu. Nu copia `.venv` de pe Windows.
`IMPORTURI OK` confirmă numai încărcarea bibliotecilor; nu validează I²C,
releul, servourile sau calibrarea. `--check` verifică configurația offline.

Nu instala forțat `adafruit-lgpio==0.2.2.0` pe Python 3.12: acea versiune cere
Python >=3.13. Dacă Python-ul de sistem nu poate importa `lgpio`, repară mai întâi
instalarea pachetului de sistem; nu adăuga manual căi către extensii din alt Python.

Surse: [Python 3.12 venv](https://docs.python.org/3.12/library/venv.html),
[instalare Adafruit](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi),
[metadatele adafruit-lgpio 0.2.2.0](https://pypi.org/project/adafruit-lgpio/0.2.2.0/).

Primul test, după verificările de mai sus, este menținerea poziției cunoscute:

```bash
python runner_gait_ikfin.py --hardware --initial-pose STAND --vx 0 --duration 3
```

**Aliniază și sprijină robotul în poziția inițială declarată.** MG996R nu furnizează
acestui program poziția actuală. Prima comandă alimentată poate produce o mișcare
bruscă dacă poziția reală este diferită; limitarea vitezei nu poate ghici punctul
de pornire. Programul pregătește toate cele 12 comenzi cu releul oprit și abia
apoi pornește alimentarea.

După verificarea menținerii poziției, poți testa mișcarea picioarelor în aer:

```bash
python runner_gait_ikfin.py --hardware --initial-pose STAND --vx 0.002 --duration 12
```

Testul cu tastatura verifică întâi offline combinațiile de comenzi și poate dura
mai mult la pornire; progresul este afișat înainte să fie pornite motoarele:

```bash
python runner_gait_ikfin.py --hardware --initial-pose STAND --keyboard --duration 120
```

La ieșire, alimentarea este oprită. **Robotul poate cădea când pierde cuplul;
păstrează-l sprijinit pe durata acestor teste.** O eroare, X sau Ctrl+C nu execută
automat o secvență `STAND → LIE`.

Comenzile I²C nu sunt o tranzacție atomică între toate servourile. Dacă o scriere
eșuează la mijlocul cadrului, unele canale pot fi deja actualizate; programul
încearcă imediat OFF și dezactivarea PWM. Întârzierile buclei peste 0,20 s sunt
respinse la următoarea iterație. Acesta nu este un watchdog hardware: un proces
blocat în kernel, `kill -9`, alimentarea pierdută a calculatorului sau un bus
blocat nu pot fi rezolvate garantat de acest program. Pentru utilizare fără
supraveghere trebuie un watchdog independent în ESP32/releu și o oprire fizică.

## 6. Afișare live pe laptop

`--sim` repetă o secvență calculată offline. Pentru comenzi live, rulează în
primul terminal:

```bash
python runner_gait_ikfin.py --dry-run --keyboard --telemetry --duration 120
```

În al doilea terminal, cu același mediu Python:

```bash
python viewer_live.py
```

Pe Windows poți folosi direct `.\.venv\Scripts\python.exe` în loc de `python`.
Tastele se apasă în primul terminal, iar al doilea afișează ilustrația. Vizualizatorul
nu are comenzi pentru motoare; închiderea sa nu oprește programul de control.

Pentru Raspberry Pi, serverul ascultă numai la `127.0.0.1:8765`. Vizualizatorul
rulează pe laptop și se conectează printr-un tunel SSH, descris în `live-view.md`.
Nu expune acest serviciu pe internet. Protocolul are numai două citiri JSON,
`/metadata` și `/state`; nu primește comenzi. Nu necesită pachete Python suplimentare.

Ilustrația reconstruiește ultima comandă acceptată de ieșire, inclusiv limitarea
vitezei. MG996R nu furnizează feedback de poziție aici: fereastra nu confirmă că
robotul real s-a mișcat, că stă în echilibru sau că releul a oprit fizic curentul.
Datele vechi, conexiunea întreruptă și modurile `DRY RUN` / `HARDWARE` sunt marcate.
La schimbarea sesiunii runnerului trebuie redeschis vizualizatorul, pentru a
reîncărca exact configurația acelei sesiuni.

Deschiderea serverului are loc înainte de activarea ieșirii. Închiderea ieșirii
precede oprirea serverului. Datele sunt publicate la aproximativ 10 Hz, iar
graficul este actualizat aproximativ de 5 ori pe secundă, în funcție de laptop.
Aceste frecvențe nu sunt garanții de timp real pe Raspberry Pi. Citirea HTTP și
desenarea nu execută operații I²C. `SIGTERM` și, pe Unix, `SIGHUP` cer cleanup;
nici aceasta nu înlocuiește întrerupătorul fizic sau un watchdog independent.

## 7. Verificări și exporturi

```bash
python -m unittest discover -s tests -v
python validate.py
python runner_gait_ikfin.py --dry-run --csv trajectory.csv --render preview.png
```

`validate.py` rulează suita de teste, apoi combinațiile translație/rotație, și
scrie `validation_results.json`. CSV-ul este o înregistrare a simulării offline,
nu telemetrie a pozițiilor reale. `legacy-ros.md` descrie separat erorile din sursele ROS vechi; `provenance.md` explică baza acestei versiuni.

Fișierele importante sunt `runner_gait_ikfin.py` (pornire), `gait_core.py`
(crawl și legătura IK), `ik_fin.py` (IK/FK), `controller.py` (stări și poziții),
`robot_io.py` (ieșire fără/cu hardware), `robot_config.py` (validare) și
`config/robot.json` (configurația ta). `telemetry.py` transmite numai datele de
afișare, iar `viewer_live.py` deschide ilustrația live pe laptop.

Nu sunt incluse integrarea ROS 2/RViz, controllerul de joc, IMU, SLAM sau mersul
dinamic. Nu am unit automat pachetele vechi cu mapări și geometrii contradictorii.
Acest pachet rezolvă varianta standalone de crawl găsită în fișierele cele mai
recente; testarea fizică a mersului rămâne obligatorie.
