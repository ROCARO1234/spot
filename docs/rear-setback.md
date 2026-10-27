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

Instrumentul este pentru **probe cu întreaga greutate preluată extern**, nu pentru
mers pe sol. Poziția/înclinarea corpului și contactele reale nu sunt măsurate;
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
