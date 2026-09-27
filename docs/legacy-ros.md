# Probleme în sursele ROS vechi

Audit: 27 septembrie 2026. Pachetele de mai jos nu reprezintă programul standalone din acest repository. Nu sunt incluse drept cod hardware executabil.

## spot_os main

Commit verificat: `faf3441bdca117ac019504ec15f9c45c7df272ca`.

- `ros2_ws_spot_rcode/src/leg_service/leg_service/Ik_for_leg.py:18,46`: `IK(10,100,50,111,155,0,0,0,0,0)` produce `ValueError: math domain error`. Calculul A_X amestecă lungimea 3D cu laturile unei proiecții. Este necesară corectarea geometriei și verificarea IK/FK; un clamp al acos ar ascunde problema.
- `soft-hard_converter/src/sth_conv_py/sth_conv_py/servo_driver.py:23–70`: mesajul fără S16 scrie primele 15 canale, apoi produce KeyError. Patch-ul din `legacy/` validează toate valorile înainte de prima scriere. Nu repară o eroare fizică I²C în timpul unui cadru și nu confirmă calibrarea.
- Driverul vechi deschide ServoKit la import. Nu trebuie importat pentru teste fără înlocuirea accesului hardware.

## Ramura codex/add-ros2-iron-code-for-leg-position-control

Commit verificat: `ad77b575e6e5e2b16368db01c96e765d9f11db19`.

- `spot_gait/gait_controller.py:56–70`: folosește abs(vx)+abs(vy); înainte, înapoi și lateral produc aceleași ținte. Yaw singur nu avansează faza.
- `spot_gait/leg_kinematics.py`: generator sinusoidal declarat placeholder, fără IK implementat.
- `gait_controller.py:39` și `ps4_control.py:17`: indexări fără verificarea numărului de axe joystick.
- `terminal_control.py:21`: o intrare numerică invalidă produce ValueError netratat.

## Pachetul local sm3_gait_controller_pipeline_full

- `setup.py:22–24`: lipsesc 3 executabile cerute de `full_pipeline_launch.py`.
- `gait_node.py:13–30`: selectează o denumire și scrie în log, fără publicarea țintelor picioarelor.
- `inverse_kinematics_node.py:27–30`: placeholder care returnează zerouri.
- `foot_trajectory_viz_node.py:16`: folosește base_link, dar lansarea nu oferă robot_description, robot_state_publisher sau JointState.
- `keyboard_controller.py:12–15`: salvează terminalul după trecerea în raw; la X nu publică Twist zero.

Nu activa pur și simplu executabilele lipsă: s-ar conecta un IK neimplementat la un driver real.

Niciuna dintre cele 5 ramuri spot_os inventariate nu conținea runner_gait_ikfin.py sau URDF. ROS 2, colcon și Raspberry Pi nu au fost disponibile pentru teste de integrare în auditul Windows; reproducerile au folosit funcțiile Python reale și componente externe simulate.
