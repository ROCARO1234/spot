# Pregătirea demonstrației RoboFest

Controllerul standalone este baza acestei versiuni. Configurația calibrată se
citește din `~/Desktop/sm5_gait_fixed/config`; instalarea nu o înlocuiește și nu
modifică limitele, canalele, direcțiile sau impulsurile servourilor. LIE și
UP_READY rămân respinse dacă depășesc limitele. CROUCH_READY are nevoie de
validare fizică; trecerea verificărilor offline nu dovedește această validare.

## Instalare și diagnostic fără motoare

Folosește același interpretor Python în care importurile hardware funcționează.
Dacă mediul nu este în directorul `.venv` al instalării originale, indică explicit
calea verificată; nu instala dependențe într-un alt mediu doar pentru acest script.

```sh
export ROBOT_PYTHON="$HOME/Desktop/Spot_SM5_RoboFest_v1/spot/.venv/bin/python"
export SPOT_CONFIG_DIR="$HOME/Desktop/sm5_gait_fixed/config"
bash scripts/install_release.sh /cale/spot-robofest.tar.gz robofest-candidate
cd "$HOME/Desktop/Spot_SM5_RoboFest_releases/robofest-candidate"
bash scripts/start_demo.sh
"$ROBOT_PYTHON" -m unittest discover -s tests -v
bash scripts/start_demo.sh --dry-run --vx 0.003 --duration 12 --dynamicity 0.5
```

Argumentele implicite ale lansatorului sunt `--check`, fără acces la motoare.
Arhiva trebuie să conțină `SHA256SUMS`, verificat pentru toate fișierele; scriptul
respinge arhive cu legături, traversare de directoare sau o versiune deja existentă.
Se creează un director nou în `~/Desktop/Spot_SM5_RoboFest_releases`; nu se
activează servicii, pornire automată sau servouri. Un checksum asigură integritatea
pachetului transferat, nu identitatea unui autor; folosește pachetul primit și
valoarea checksum livrată împreună cu acesta.

## Probe scurte, robot susținut

Oprește orice alt controller. Susține corpul astfel încât picioarele să se poată
mișca liber, verifică zona și păstrează întrerupătorul fizic la îndemână. Prima
poziție alimentată trebuie să corespundă poziției fizice aliniate de operator;
fără feedback montat, software-ul nu poate cunoaște poziția inițială reală.

Întâi verifică fiecare probă offline, fără `--hardware`. Apoi, numai după anunț
și confirmarea explicită a operatorului, execută o singură fracție a tranziției.
Fiecare probă pornește din STAND, coboară parțial și revine în STAND. Realiniază
robotul susținut înaintea următoarei alimentări, deoarece releul se oprește la final.

```sh
"$ROBOT_PYTHON" probe_pose.py --config "$SPOT_CONFIG_DIR/robot.json" \
  --poses "$SPOT_CONFIG_DIR/poses.json" --fraction 0.1
# DOAR după confirmarea operatorului că robotul este susținut și aliniat STAND:
"$ROBOT_PYTHON" probe_pose.py --config "$SPOT_CONFIG_DIR/robot.json" \
  --poses "$SPOT_CONFIG_DIR/poses.json" --fraction 0.1 --hardware --confirm-supported
```

Continuă separat cu fracțiile `0.25`, `0.5`, `0.75`, `1`, repetând întâi verificarea
offline și confirmarea operatorului pentru fiecare probă. Oprește imediat la
lovituri mecanice, blocaj, zgomot anormal, încălzire sau pierdere de sprijin. Nu
trece automat la fracția următoare. Verifică separat poziția joasă cu greutatea
robotului susținută gradual și ridicarea în STAND, înainte de orice mers pe sol.
Faptul că mișcarea funcționează cu picioarele libere nu dovedește capacitatea de
ridicare sub sarcină sau echilibrul pe sol.

Numai după confirmarea fizică a poziției joase și a tranziției complete în ambele
sensuri, înregistrează rezultatul real. Nu folosi acest pas pentru a ocoli proba:

```sh
mkdir -p "$HOME/Desktop/Spot_SM5_RoboFest_validation"
"$ROBOT_PYTHON" pose_validation.py record \
  --config "$SPOT_CONFIG_DIR/robot.json" --poses "$SPOT_CONFIG_DIR/poses.json" \
  --pose CROUCH_READY --operator "Numele operatorului" \
  --evidence "Descrierea reală a probei complete, în ambele sensuri, și a sprijinului verificat" \
  --confirmed-supported-roundtrip \
  --output "$HOME/Desktop/Spot_SM5_RoboFest_validation/crouch.json"
```

Înregistrarea este o atestare a operatorului pentru configurația verificată,
nu o măsurare automată. După schimbări relevante de calibrare sau geometrie,
validează din nou; nu edita manual amprentele pentru a reutiliza aprobarea.

## Demonstrație după validare

Aliniază și susține robotul în poziția joasă validată CROUCH_READY, cu zona
liberă. Pornește o probă scurtă susținută înainte de mersul pe sol. Lansatorul
hardware cere textul explicit `ROBOT SUPPORTED`; această confirmare nu
înlocuiește fișierul de validare a poziției joase.

```sh
mkdir -p logs
bash scripts/start_demo.sh --hardware --demo --keyboard \
  --initial-pose CROUCH_READY --low-pose CROUCH_READY \
  --low-validation "$HOME/Desktop/Spot_SM5_RoboFest_validation/crouch.json" \
  --dynamicity 0.5 --duration 20 --log "logs/demo-$(date -u +%Y%m%dT%H%M%SZ).jsonl"
```

Secvența normală este: poziție joasă validată → ridicare STAND → teleoperare →
frânare și așezarea picioarelor → coborâre → releu OFF. `Esc` cere oprirea normală
și încheierea secvenței; expirarea duratei declanșează aceeași secvență. `Space`
cere oprirea normală a mersului, fără a încheia singură sesiunea. `W/S` înainte/
înapoi, `A/D` lateral, `Q/E` rotire. Eliberarea comenzilor și lipsa repetării
tastelor declanșează oprirea prin timeout.

`X`, `Ctrl+C`, semnalele SIGTERM/SIGHUP recepționate sau o eroare critică cer oprirea de urgență:
alimentarea se oprește fără coborâre programată. Robotul poate pierde susținerea
când dispare cuplul; folosește sprijinul și întrerupătorul fizic. Închiderea unei
ferestre de afișare nu este comandă de oprire.

O conexiune de rețea întreruptă nu poate fi detectată instantaneu: lipsa tastelor
oprește mersul prin timeout; alimentarea este întreruptă când sesiunea primește
SIGHUP, la expirarea duratei sau la o eroare detectată. Pentru oprire imediată
la orice blocaj de sistem/rețea folosește întrerupătorul fizic. Nu există watchdog
hardware independent instalat. Durata include ridicarea inițială; coborârea și
oprirea normală au timp suplimentar.

Dynamicitatea se ajustează numai în intervalul permis `0.3..1`; începe cu `0.5`.
Nu crește vitezele sau limitele servourilor pentru aspectul demonstrației.
Transferul de greutate și traiectoriile sunt modele cinematice; echilibrul,
contactele, sarcina și alunecarea nu sunt măsurate. Nu există promisiunea
performanțelor Boston Dynamics Spot.

## Revenire la instalarea anterioară

Încheie și deconectează alimentarea servo înainte de schimbarea controllerului.
Instalarea originală și backup-ul datat se păstrează. Scriptul următor alege
controllerul original și face doar verificare offline:

```sh
bash scripts/rollback.sh
# Probă STAND de 5 secunde, numai cu acordul operatorului și robot susținut:
bash scripts/rollback.sh --hardware --initial-pose STAND --duration 5
```

Revenirea nu șterge fișiere și nu pornește automat alimentarea. Nu transmite
opțiunile noi `--demo` sau `--low-validation` controllerului original. Păstrează
logul sesiunii, raportul testelor, amprentele versiunii și backup-ul calibrărilor.

Rămân de verificat fizic: toate fracțiile coborârii, poziția joasă sub sarcină,
ridicarea completă, oprirea normală și de urgență, apoi pași foarte scurți pe o
suprafață plană. IMU și feedback-ul celor 12 servouri sunt opționale și încă
indisponibile; nu există stabilizare activă pe senzori nemontați.
