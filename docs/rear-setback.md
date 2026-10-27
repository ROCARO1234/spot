# Diagnostic după prima probă pe sol

**Versiunea nu este validată pentru ridicare autonomă sau mers pe sol.**
Operatorul a clarificat că dificultatea apare la ridicare: labele din spate sunt
prea în față, robotul nu își preia singur greutatea. Estimarea operatorului pentru
repoziționare este 20–30 mm înapoi. Alimentarea servo a fost confirmată oprită.

În model, CROUCH_READY pune RL cu 23,5 mm și RR cu 39,1 mm mai în față decât
STAND. Acestea sunt coordonate calculate din unghiurile calibrate, nu măsurători.
Interpolarea inițială directă în unghiuri nu controlează traseul tălpii sau sarcina.
Noul instrument construiește o prepoziționare separată: ridică un singur picior
din spate, îl duce înapoi, îl așază, apoi repetă pentru celălalt picior înaintea
extensiei. Profilul de întoarcere este inversul traseului și se termină în
CROUCH_READY. Limitele și unghiurile calibrate nu sunt schimbate.

Modul hardware implicit este pentru **probe cu întreaga greutate preluată extern**,
nu pentru mers pe sol. Proba supravegheată de transfer al greutății are un mod
explicit separat, descris mai jos. Poziția/înclinarea corpului și contactele reale nu sunt măsurate;
nu se calculează cuplul necesar. Profilul nu este introdus în `--demo`, iar o
probă susținută reușită nu validează ridicarea sub greutate.

## Comparare offline

```sh
py="$HOME/Desktop/Spot_SM5_RoboFest_v1/spot/.venv/bin/python"
cfg="$HOME/Desktop/sm5_gait_fixed/config"
"$py" compare_motion_candidates.py --config "$cfg/robot.json" \
  --poses "$cfg/poses.json" --output /cale/noua/comparison.json
"$py" rear_setback.py --config "$cfg/robot.json" --poses "$cfg/poses.json" \
  --rear-setback-mm 20 --stage RL --fraction 0.25
```

Compararea verifică repoziționări de 20/25/30 mm și mers cu ridicare 12/18 mm,
la aceeași viteză și cu același plafon de pas de 30 mm. Ridicarea de 18 mm este
doar o variantă offline, nu o valoare validată pe robot și nu schimbă profilul
de producție. Proba anterioară de 5 secunde comanda doar primul picior înainte
de oprire; restul mișcărilor erau reașezarea picioarelor. Nu a verificat un ciclu
complet de mers repetat. Viteza sau plafonul de pas nu se măresc pentru a compensa
o problemă de sprijin la ridicare.

## Prima probă nouă, numai după confirmare fizică separată

Începe cu un singur picior și o fracție mică. Robotul este aliniat CROUCH_READY,
cu corpul pe suport și întreaga greutate preluată de suport; labele sunt libere.
La 20 mm și fracție 0,25, deplasarea modelată este 5 mm înapoi și ridicarea 3 mm.
La final se revine în CROUCH_READY și se cere releu OFF. Nu executa automat
fracțiile sau piciorul următor. Operatorul confirmă direcția fizică, lipsa
interferențelor și oprirea alimentării după fiecare probă.

```sh
# Numai după anunț și confirmarea explicită pentru această probă:
"$py" rear_setback.py --config "$cfg/robot.json" --poses "$cfg/poses.json" \
  --rear-setback-mm 20 --stage RL --fraction 0.25 --hardware --confirm-supported \
  --low-validation "$HOME/Desktop/Spot_SM5_RoboFest_validation/crouch-supported-20261027.json" \
  --log /cale/noua/rear-RL-quarter.jsonl
```

`--stage RR` verifică separat piciorul drept. `--stage rise` include ambele
repoziționări, extensia și întoarcerea, exclusiv cu sprijin complet și confirmare
separată după verificarea picioarelor. `Ctrl+C`, SIGTERM/SIGHUP sau o eroare
închid ieșirea fără întoarcere programată. Întrerupătorul fizic rămâne necesar.

Toată traiectoria este verificată înainte de alimentare. Timpii pot fi măriți
pentru limitele de viteză ale servourilor; unghiurile nu sunt tăiate la limită.
Logurile disting timpul planificat de timpul real și nu pretind feedback.

## Observarea direcției

La primele probe RL de 5 mm și 10 mm, operatorul a văzut mișcare, dar nu a putut
identifica sensul. Direcția nu este încă validată. Nu mări cursa doar pentru
acest motiv. Cu alimentarea oprită, pune un reper vizual fix lângă laba liberă.
Opțiunea `--hold-s 3` păstrează poziția de capăt timp de 3 s înainte de revenire,
fără mărirea cursei, ridicării sau vitezei. O probă RL de 10 mm, cu ridicare
modelată de 6 mm și această pauză durează aproximativ 8 s. Necesită o confirmare
nouă, sprijin complet și log separat; nu este o probă pe sol.

## Probă explicită de transfer al greutății

`--loaded-rise --confirm-catch-ready` permite numai o probă supravegheată, fără
mers, după validarea fizică a întregului traseu susținut. Cere separat
`--supported-route-validation`, legat de calibrarea și de toate cadrele exacte
ale traseului. Nu acceptă un traseu parțial sau altă cursă/pauză. Înregistrarea
nu poate fi creată în aceeași invocare cu `--hardware` și nu validează sarcina.

Atestarea probei susținute se poate înregistra offline prin aceiași parametri
`--rear-setback-mm 20 --stage rise --fraction 1 --hold-s 3`, plus
`--record-supported-route /cale/noua/route.json --confirm-supported-route
--operator NUME --evidence DESCRIERE_REALĂ`. Se folosește numai după observația
explicită a operatorului. Nu se generează automat din codul de ieșire zero.

Pentru proba de sarcină, labele ating un plan aderent. Operatorul preia toată
greutatea în timpul repoziționării și reduce sprijinul treptat numai la extensie,
putând prelua imediat corpul. Înaintea întoarcerii/repoziționării inverse preia din
nou greutatea. Nu se încearcă mers sau stabilizare. Se păstrează un log nou și
se cere o confirmare nouă de pregătire înainte de alimentare. Eșecul probei
înseamnă că ridicarea rămâne nevalidată, indiferent de rezultatul software.
