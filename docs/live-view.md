# Afișare live pe laptop

Vizualizatorul afișează comenzile reconstruite prin FK; nu măsoară pozițiile reale. Folosește aceeași versiune a proiectului pe laptop și pe Pi.

## Laptop Windows, fără robot

După instalarea dependențelor din README, în primul PowerShell, din folderul `spot`:

```powershell
.\.venv\Scripts\python.exe runner_gait_ikfin.py --dry-run --keyboard --telemetry --duration 120
```

În alt PowerShell, din același folder:

```powershell
.\.venv\Scripts\python.exe viewer_live.py
```

Tastele se apasă în primul terminal, nu în fereastra grafică. W/S: înainte/înapoi; A/D: lateral; Q/E: rotație; Space: oprire normală; Esc: oprire normală și ieșire; X/Ctrl+C: oprire de urgență. Închiderea vizualizatorului nu oprește runner-ul.

## Raspberry Pi și tunel SSH

Pe Pi, instalează proiectul și mediul hardware conform [ghidului](usage.md#5-înainte-de-orice-test-cu-motoare). Oprește runner-ul local de pe laptop dacă folosește portul 8765.

În PowerShell pe laptop, înlocuiește `IP_ROBOT` cu adresa reală a Raspberry Pi. Contul `spot` este cel din sesiunea de diagnostic; adaptează-l dacă instalarea ta folosește alt cont.

```powershell
ssh -t -o ExitOnForwardFailure=yes -L 127.0.0.1:8765:127.0.0.1:8765 spot@IP_ROBOT
```

În terminalul SSH, intră în folderul real al proiectului de pe Pi. Dacă l-ai clonat în `~/spot`:

```bash
cd ~/spot
source .venv-rpi-system/bin/activate
python runner_gait_ikfin.py --dry-run --keyboard --telemetry --duration 120
```

Mai întâi testează fără motoare. Așteaptă mesajele `Read-only telemetry` și `DRY RUN`. Pe laptop, într-un alt PowerShell:

```powershell
.\.venv\Scripts\python.exe viewer_live.py
```

Tastele se apasă acum în terminalul SSH. Serverul ascultă exclusiv pe loopback. Nu este necesară expunerea portului în router.

## Dacă imaginea nu se conectează

- Păstrează tunelul și runner-ul pornite; închide instanțele locale vechi care ocupă portul.
- Verifică de pe laptop `curl.exe --max-time 5 http://127.0.0.1:8765/metadata`.
- Dacă runner-ul se oprește la importul lgpio, urmează pașii de instalare din ghid; după oprirea runner-ului, telemetria lui nu mai este disponibilă.
- Redeschide vizualizatorul după repornirea runner-ului, pentru a reîncărca configurația sesiunii.

Modurile `DRY RUN` și `HARDWARE`, datele învechite și deconectările sunt marcate. Trecerea la `--hardware` se face numai după verificarea fizică a calibrării și a releului, cu robotul sprijinit. Nu modifica marcajele de verificare doar pentru a trece de un mesaj de blocare.
