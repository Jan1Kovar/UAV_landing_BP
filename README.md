# UAV_landing_BP
Veškerý kód použitý k autonomnímu přistání drona na dronportu v Gazebo simulaci.

rozdělení jednotlivých misí*-------------------------------------------




Qgroundcontrol před spuštěním simulace
-------
přesun do složky se souborem

./QGroundControl.AppImage

*-------------------------------------------Aruco test
-------

Načtení simulace nový terminál
-------
cd gazebo_simulation/run

./Aruco_test.sh

Spuštění Gazebo clienta pro vizualizaci nový terminál
-------
cd gazebo_simulation/run

./run_gzclient.bash

Zahájení mise nový terminál
-------
cd gazebo_simulation/scripts/aruco_landing 

python3 Aruco_test.py


Lepší proces měření, který nebyl využit při měření v BP. Vznikl z kombinované metody.
python3 Aruco_test_2.py

*-------------------------------------------Kamera detection
-------

Načtení simulace nový terminál
-------
cd gazebo_simulation/run

./run_new_model_camera.sh

Spuštění Gazebo clienta pro vizualizaci nový terminál
-------
cd gazebo_simulation/run

./run_gzclient.bash

Zahájení mise nový terminál
-------
cd gazebo_simulation/scripts/aruco_landing 

python3 Kamera.py

*-------------------------------------------Kombinace
-------

Načtení simulace nový terminál
-------
cd gazebo_simulation/run

./run_new_model_camera.sh

Spuštění Gazebo clienta pro vizualizaci nový terminál
-------
cd gazebo_simulation/run

./run_gzclient.bash

Zahájení mise nový terminál
-------
cd gazebo_simulation/scripts/aruco_landing 

python3 Kombinace.py
