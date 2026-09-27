# Patch pentru vechiul spot_os

`spot_os_servo_validation.patch` se aplică numai în repository-ul `ROCARO1234/spot_os`, pe baza commitului `faf3441bdca117ac019504ec15f9c45c7df272ca`. Nu este necesar pentru runner-ul standalone din spot.

Din rădăcina checkout-ului spot_os, cu patch-ul copiat acolo:

```text
git apply --check spot_os_servo_validation.patch
git apply spot_os_servo_validation.patch
python -m unittest discover -s soft-hard_converter/src/sth_conv_py/test -p test_servo_frames.py -v
```

Cele 4 teste nu importă hardware real sau ROS. Verifică callback-ul real cu ieșiri simulate. Mesajele invalide sunt respinse înaintea oricărei scrieri. Limitele 0–180 reprezintă contractul driverului vechi, nu limite mecanice validate.

Sursele vechi sunt distribuite de spot_os cu licența Apache-2.0; textul aferent este păstrat în `LICENSE-spot_os`.
