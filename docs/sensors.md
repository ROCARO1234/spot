# Interfața opțională pentru senzori

Senzorii planificați nu sunt încă montați. `SensorHub()` este dezactivat implicit,
nu importă biblioteci hardware, nu scanează I²C și întoarce date indisponibile.
Nicio stabilizare sau corecție de servou nu este activată de această interfață.
Tracking-ul controllerului rămâne diferența calculată între țintă și comanda
limitată în viteză; nu reprezintă feedback măsurat.

`config/sensors.example.json` este o configurație separată, fără modificări ale
calibrărilor robotului. Poate fi încărcată prin `load_sensor_settings(path)` și
transmisă la `SensorHub(settings)`. Un backend hardware nu este disponibil încă;
activarea configurației fără backend întoarce explicit „indisponibil”.

După alegerea componentelor, un driver poate implementa `SensorBackend.read()`.
Integrarea se face explicit cu `backend: "injected"`, `enabled: true` și un obiect
backend transmis constructorului. Acesta trebuie să returneze `None` dacă nu are
date sau un dicționar cu exact următoarele câmpuri:

```text
schema_version: 1
source: identificatorul sursei reale și al calibrării
timestamp_monotonic_s: momentul achiziției în time.monotonic() al procesului
imu: null sau {accel_mps2: [x, y, z], gyro_rps: [x, y, z]}
joint_angles_deg: null sau toate cele 12 unghiuri FL/FR/RL/RR × sh/up/lo
```

Convenția corpului: X înainte, Y stânga, Z sus. Driverul trebuie să aplice rotația
de montare stabilită prin măsurători și calibrarea înainte de a furniza datele.
Accelerația include gravitația; nu se deduce automat o orientare sau o înălțime.
Unghiurile feedback trebuie convertite în aceeași convenție ca unghiurile servo
comandate, prin calibrare individuală măsurată. Verificarea 0–180° din interfață
este doar validare de date și nu schimbă limitele fizice din `robot.json`.

Cadrele mai vechi decât `max_age_s`, cele din viitor, cele cu timpul regresiv,
NaN/Inf, câmpuri lipsă, vectori incompleți sau mai puțin de 12 unghiuri sunt
respinse. Nu se înlocuiesc cu zerouri, comenzi, ultimul cadru sau valori simulate.
`SensorReading` indică disponibilitatea, motivul, sursa și vechimea. IMU și
feedback-ul pot lipsi separat; consumatorul trebuie să verifice fiecare câmp.
O eroare de citire întoarce indisponibil; un viitor mecanism de stabilizare va
necesita propria politică verificată de oprire, fără a folosi date vechi.

Înainte de un driver real sunt necesare: modelul exact și varianta GY-91,
orientarea montajului, modelele ADC și multiplexorului, adresele/canalele I²C,
tensiunile maxime și circuitul de adaptare, masă comună, calibrarea celor 12
semnale și frecvența/întârzierea achiziției. Nu conecta ieșiri analogice de servou
direct la Raspberry Pi. Alegerea și verificarea electrică rămân o etapă separată.

Teste offline, din directorul `spot`:

```sh
python3 -m unittest discover -s tests -p test_sensors.py -v
```
