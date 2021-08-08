First prototype

First drone prototype was built based on Visuo SG700 arms.
The idea was to build foldable drone which could fly 20 minutes on 1S Li-ion battery (failed!).
Motors have been chosen due low price and spare parts availablility on Aliexpress.
I have chosen motors with large propellers hoping to get more thrust to lift up LiIon battery.
Parts for case was 3d printed using ABS and glued together. Case was designed to have minimum weight. Every gram counts. Case is still very stiff thanks to triangle mesh and U shapes.

On first stage, transmitter was based on nrf24 module, using Cable protocol. Later moved to HX-ESPNOW-RC.

Total weight without battery is 130g. Li-Ion battery is 49g.
Unfortunatelly, this is too heavy for the SG700 mottors.
Litokala HG2 battery can provide 3500mAh, but only with 0.35A current and if discharged down to 2.5V.
Battery drop voltage to 3.6V after 1000mah. 4 motors have power consumption ~5.5A in hower state. 
Due to voltage drop in battery connector, wires and battery internal resistance, motors get 3.3V which is not enough to hold altitude of 180g drone. Best time achieved is 7 minutes. I could achieve 10-12 minutes with
better battry and wiring, but there would be still no benefit over 1200mAh Lipo battery.
So after the first motor failue, I gave up idea to build 1S Li-Ion drone.
1S Li-ion would be effective if drone could keep altitute down to 3.0V at least. All cheap brushed motors available are designed to run on Li-po voltages: 4.2-3.2V.
So I moved to more compact design based on Eacihne E58 motors and 1000mAh Lipo battery.


