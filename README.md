# spot

Control SM5 standalone pentru mers crawl, cinematică inversă/directă, control din tastatură și afișare live. Versiune **1.2.0-rc2**, pornită din `main` 4f21de8 și comparată cu instalarea 1.1.1 de pe robot.

**Ridicarea autonomă și mersul pe sol nu sunt validate.** Operatorul a raportat suprasarcină la ridicare, cu labele din spate prea în față. [Instrumentul de repoziționare](docs/rear-setback.md) pregătește probe separate, cu întreaga greutate preluată extern. Nu schimbă automat traseul demonstrației sau calibrarea; variantele de mers cu ridicare mai mare sunt încă offline.

Pregătirea demonstrației, probele graduale, validarea poziției joase și revenirea sunt documentate în [ghidul RoboFest](docs/robofest.md). [Interfața opțională de senzori](docs/sensors.md) este dezactivată implicit; nu există feedback măsurat sau stabilizare activă. Rapoartele istorice din `docs/validation.json` descriu versiunea anterioară; pentru această versiune rulează `validate.py`.

Pornirea implicită este offline. Numai `--hardware` poate activa ieșirea reală, după verificarea configurației. Acest proiect nu pretinde o integrare ROS 2 completă sau un robot validat fizic.

## Pornire pe Windows

Din folderul proiectului:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe runner_gait_ikfin.py --check
.\.venv\Scripts\python.exe runner_gait_ikfin.py --sim
```

Pentru test rapid fără fereastră:

```powershell
.\.venv\Scripts\python.exe runner_gait_ikfin.py --dry-run --vx 0.003 --duration 3
```

## Raspberry Pi și lgpio

Pe Pi 5 cu Python 3.12, pachetul `python3-lgpio` instalat în sistem trebuie să fie vizibil interpretorului hardware. Folosește un venv creat cu `/usr/bin/python3 -m venv --system-site-packages`; verifică întâi importul lgpio în Python-ul sistemului. Pașii compleți, fără pornirea motoarelor, sunt în [ghidul de utilizare](docs/usage.md#5-înainte-de-orice-test-cu-motoare).

`config/robot.json` conține geometrie și limite provizorii, cu verificările hardware **false**. Nu înlocui configurația calibrată de pe robot. Poți păstra o copie locală ignorată de Git, `config/robot.local.json`, și să o selectezi cu `--config config/robot.local.json`.

## Tastatură și imagine live

```text
python runner_gait_ikfin.py --dry-run --keyboard --telemetry --duration 120
python viewer_live.py
```

Rulează comenzile în două terminale, cu același mediu Python. Tastele se apasă în terminalul runner-ului: W/S înainte/înapoi, A/D lateral, Q/E rotație, Space oprire normală, X/Ctrl+C oprire de urgență. [Laptop + tunel SSH](docs/live-view.md).

Imaginea afișează comenzi reconstruite cinematic, nu feedback măsurat de la servouri.

## Structură

| Componentă | Rol |
|---|---|
| `runner_gait_ikfin.py` | CLI, preflight și bucla de control |
| `controller.py`, `gait_core.py` | Stări, crawl, transferul corpului și limite de viteză |
| `ik_fin.py` | IK/FK |
| `robot_config.py`, `config/` | Configurație, poziții și validare |
| `robot_io.py` | Ieșire offline sau ServoKit/PCA9685 + releu |
| `keyboard_control.py` | Taste și expirarea comenzilor |
| `telemetry.py`, `viewer_live.py`, `simulation.py` | Afișare, simulare și export offline |
| `tests/`, `validate.py` | Teste și 27 de combinații de mers |
| `docs/`, `legacy/` | Ghiduri, proveniență și diagnosticul surselor ROS vechi |

## Teste

Pe Windows, setează UTF-8 și pentru subprocese:

```powershell
$env:PYTHONUTF8 = "1"
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe validate.py --output reports/validation.json
```

Pe Linux:

```bash
python -m unittest discover -s tests -v
python validate.py --output reports/validation.json
```

Rezultatul local verificat este în [docs/validation.json](docs/validation.json). Hardware-ul nu a fost exercitat. Testul SIGHUP este disponibil numai pe Unix.

## Ce a fost corectat

- Erorile de import ServoKit/Blinka sunt raportate controlat, cu cauza păstrată și instrucțiuni pentru lgpio.
- Telemetria refuză un port ocupat și pe Windows.
- Ghidul de instalare separă mediul hardware Raspberry Pi de mediul laptopului.
- Sunt păstrate geometria, calibrarea de referință, formulele gait/IK și interfața CLI existente.

Sursele ROS vechi au defecte independente: [diagnostic ROS](docs/legacy-ros.md). Ele nu sunt lansate de acest proiect. Patch-ul de validare pentru driverul vechi `spot_os` este disponibil separat în `legacy/`.

Înainte de mers real rămân de verificat pe robot contactul electric, accesul I²C, releul, canalele, limitele, STAND și echilibrul sub sarcină. [Proveniență și limite](docs/provenance.md).
