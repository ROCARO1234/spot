# Identificarea feedbackului servo — diagnostic separat

**Dependență confirmată de utilizator:** semnalul de poziție necesită alimentarea
servourilor. Citirea fără activarea lor servește doar diagnosticului ADC/topologiei
și bazei de zgomot; nu furnizează poziții utilizabile sau asocieri de articulații.
Pentru o bază de feedback alimentat este necesară ulterior o etapă separată,
după verificarea fizică a tensiunilor și confirmarea operatorului. Alimentarea
singură poate pune în mișcare servourile dacă PWM-ul existent este activ; nu
se presupune că releu ON înseamnă automat „fără mișcare”.

Pachetul furnizat de utilizator, `Spot_AutoMap_Feedback_v1.zip` și scriptul său
`auto_map_feedback.py`, **nu sunt încă disponibile**. Fișierele de aici sunt
suport de diagnostic/analiză pregătit separat; nu reprezintă o inspecție sau
o copie a acelui script. Nicio funcție de mișcare nu este implementată aici.

## Configurație explicită

Bus 1, selector mux `0x71`, protocol control-byte cu exact un bit activ; ADC
`0x48` pe fiecare canal. Mux 0–2: ADS1015; mux 3: ADS1115, asociat de utilizator
piciorului RR. ADC-urile nu au identificator unic de model: adresa I²C nu poate
distinge ADS1015 de ADS1115. Tipurile provin din configurația hardware declarată.
`0x70` nu este folosit. Canalele 0–2 și intrările analogice rămân neasociate.

Driverul citește toate AIN0–AIN3 ale fiecărui ADC. Selectează un singur canal,
verifică masca prin readback și dezactivează toate canalele la schimbare și
închidere. Folosește single-shot, așteaptă terminarea conversiei precedente,
verifică registrul de configurare și aplică timeout. Nu face scanare de adrese.
Nu importă ServoKit, PCA9685 sau ieșirea de motoare și nu adresează `0x08`, `0x40`
sau `0x70`; numai `0x71` și `0x48` primesc comenzi în cititor.

ADS1015: 12 biți semnați, aliniați la stânga în registrul de 16 biți; se extinde
semnul, apoi se deplasează 4 biți. ADS1115: 16 biți semnați. La scala ±6,144 V,
cuantizarea este 3 mV, respectiv 0,1875 mV. Pentru 128 SPS, codul DR este 000
la ADS1015 și 100 la ADS1115. Octeții registrelor sunt MSB-first.

Surse primare: [ADS1015](https://www.ti.com/lit/ds/symlink/ads1015.pdf),
[ADS1115](https://www.ti.com/lit/ds/symlink/ads1115.pdf), protocol selector
[TCA9548A compatibil](https://www.ti.com/lit/ds/symlink/tca9548a.pdf).

## Tensiuni neconfirmate — mișcarea rămâne blocată

Limita operațională a pinilor analogici este GND..VDD. Scala PGA nu permite
depășirea VDD; limitele absolute de avarie nu sunt un domeniu normal de lucru.
Citirea tensiunii unui pin ADC nu măsoară independent alimentarea ADC-ului și nu
demonstrează că valorile vor rămâne admise când servoul este alimentat/mișcat.

Sunt necesare măsurători fizice la fiecare placă: VDD față de masa comună,
minimul/maximul feedbackului la intrările ADC, circuitul de adaptare sau
conexiunea directă și nivelurile pull-up/adaptarea I²C. Nu completa configurația
cu presupuneri. Câmpurile măsurate sunt dicționare cu cheile `"0"`–`"3"`, pentru
canalele mux; `measurement_evidence` descrie măsurătorile reale. Configurația
livrată are valori null și verificările electrice false.

## Repetarea citirii fără mișcare

Folosește mediul existent al proiectului. Înainte de citire, oprește procesele
identificate de control/scanare și verifică `fuser -v /dev/i2c-1`. Nu porni alt
scanner în paralel. Lacătul aplicației este doar pentru procese cooperante și
nu exclude un program străin care îl ignoră.

```sh
py="$HOME/Desktop/Spot_SM5_RoboFest_v1/spot/.venv/bin/python"
cd "$HOME/Desktop/Spot_Feedback_Diagnostics_20261101_r3"
"$py" feedback_readout.py                 # doar verificare offline, fără I²C
"$py" feedback_readout.py --read --i2c-idle-confirmed --samples 50 \
  --output-dir "$HOME/feedback_results/baseline-$(date -u +%Y%m%dT%H%M%SZ)"
```

Directorul trebuie să fie nou. `baseline.json` păstrează conversiile brute,
tipul ADC configurat, volții, momentele de achiziție și media, deviația standard,
MAD, minimul/maximul și zgomotul vârf-la-vârf pe toate cele 16 intrări. Flagurile
de saturație sunt raportate; valori ușor negative nu sunt tăiate la zero. Erorile
de deselectare/închidere sunt păstrate. Conexiunile se închid **înainte** de
salvarea rezultatelor. Codul de ieșire 0 confirmă achiziția, nu sănătatea electrică
sau identificarea unei articulații. Semnalele SIGTERM/SIGHUP cer curățare.

`mapping-proposal.json` conține toate cele 12 articulații, canalele PCA declarate
și intrări neidentificate. Pentru RR, mux 3 este doar constrângerea furnizată de
utilizator, nu o asociere identificată prin mișcare. Datele obținute cu alimentarea
servourilor oprită nu demonstrează că feedbackul funcționează când este alimentat.

`config/robot.json` și `config/poses.json` din pachet sunt fixture-uri/configurații
de referință pentru teste, cu verificările hardware neconfirmate. Nu înlocuiesc
fișierele calibrate din `sm5_gait_fixed/config`; comanda de preflight de mai jos
citește explicit configurațiile calibrate externe.

### Diagnostic de registre brute

Dacă datele contrazic formatul ADS1015 declarat, modul strict oprește achiziția.
Nu modifică automat tipul ADC. Pentru investigație, `--raw-only` citește explicit
registrele și zgomotul în unități signed16, fără a inventa coduri ADC normalizate,
tensiuni sau rezoluția reală. Modelul rămâne configurat, dar neverificat de date.

```sh
"$py" feedback_readout.py --read --raw-only --i2c-idle-confirmed --samples 20 \
  --output-dir "$HOME/feedback_results/raw-$(date -u +%Y%m%dT%H%M%SZ)"
```

Acest mod nu este o conversie automată ADS1015→ADS1115 și nu poate fi utilizat
pentru autorizarea mișcării. Se păstrează separat încercarea strictă nereușită.

## Preflight offline pentru identificare ulterioară

```sh
"$py" feedback_preflight.py \
  --robot-config "$HOME/Desktop/sm5_gait_fixed/config/robot.json" \
  --poses "$HOME/Desktop/sm5_gait_fixed/config/poses.json" \
  --amplitude-deg 1 --rate-deg-s 1 \
  --output "$HOME/feedback_results/plan-$(date -u +%Y%m%dT%H%M%SZ).json"
```

Nu mișcă nimic; salvează un plan neexecutabil și returnează 2 dacă există
blocaje. Verifică STAND și toate cele 12×repetări×ținte înainte de o eventuală
alimentare viitoare. Amplitudinea este limitată la ±3°, cu revenire la referință;
nu crește automat. La pornirea unui viitor script de mișcare, toate servourile
pot primi STAND: operatorul trebuie să confirme robot complet susținut, labe
libere, aproape de STAND și întrerupător accesibil. Nu este autorizare de mers,
ridicare autonomă sau stabilizare. Scriptul original trebuie întâi obținut și
verificat; preflight-ul de aici nu garantează comportamentul unui alt script.

## Analiza propunerii

`feedback_mapping.propose_mapping` analizează numai ferestre măsurate, repetate,
la offseturi zero/plus/minus simetrice, pentru un singur servo. Compară toate
cele 16 intrări în volți, cu prag de răspuns și zgomot bazat și pe cuantizarea
fiecărui ADC. Respinge ambiguitatea, crosstalk, repetări inconsistente, drift,
asocieri duplicate și repartizări incompatibile pe picioare. Nu folosește a doua
opțiune pentru a ascunde o coliziune. Trei articulații ale unui picior trebuie
să aibă același mux, distinct de celelalte picioare, iar RR trebuie să fie mux 3.

Încrederea este ordinală, nu probabilitate. Panta V/grad comandat este doar un
indicator de asociere; nu dovedește unghiul fizic și nu este calibrare absolută.
Rezultatul rămâne propunere pentru verificare fizică. Nu scrie în `robot.json`
sau `poses.json` și nu activează control pe feedback. Testele sintetice sunt
etichetate ca atare și nu sunt raportate drept măsurători ale robotului.
