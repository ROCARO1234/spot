# Proveniență

Această versiune curată pornește din pachetul utilizatorului `sm5_gait_fixed`, VERSION `1.1.0-crawl`, identificat local la auditul din 27 septembrie 2026. Este un program standalone, distinct de repository-ul ROS `ROCARO1234/spot_os`.

Documentele istorice menționează o versiune 1.2. Nu am găsit sursele acelei versiuni în copia verificată; această distribuție nu se prezintă drept un upgrade al versiunii 1.2.

[Manifestul arhivei originale](provenance-source.json) păstrează SHA-urile surselor din `desktop_robot.zip`. Corecțiile actuale nu schimbă `config/robot.json`, `config/poses.json`, `gait_core.py`, `ik_fin.py` sau `controller.py`. Limitele și geometria sunt valori din cod, nu măsurători fizice validate aici.

Versiunea 1.1.1 adaugă tratarea erorilor Blinka, corectarea rezervării portului Windows, teste de regresie și documentație de instalare. Documentația este grupată în `docs/`; nu sunt incluse medii virtuale, cache-uri, build-uri colcon sau configurații locale de acces.

Validarea este cinematică și software. Nu validează frecarea, contactele, centrul de greutate real, alimentarea, servourile ori oprirea fizică.
