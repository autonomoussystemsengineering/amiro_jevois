# AMiRo_Jevois
Ziel dieses Projektes ist es, eine Demo Applikation für die Anbindung der [Jevois smart machine vision camera](http://jevois.org/) für den AMiRo zu implementieren. Dabei wird die Jevois Kamera über eine serielle Schnittstelle mit dem Cognition Board des AMiRo verbunden. Implementiert werden zwei Demo Applikationen:
1. ArUco Marker Tracking
2. Obstacle Avoidance mittels Optical Flow

# Vorbereitung 
1. [AMiRo_Jevois Repo](https://github.com/kevinp1993/AMiRo_Jevois) auf Rechner clonen 
2. Einrichtung der Cross-Compiler Umgebung mit **poky-glibc-x86_64-meta-toolchain-openrobotix-cortexa8hf-vfp-neon-toolchain-openrobotix-1.7.2.sh** auf dem Rechner. Das benötigte shell script liegt im [ks-temp Ordner](sftp://twix.techfak.uni-bielefeld.de/vol/ks-temp/). 
3. Sourcen des Cross-Compilers:
```
source /opt/poky/1.7.2/environment-setup-cortexa8hf-vfp-neon-poky-linux-gnueabi
```
4. Im Ordner [Jevois_AMiRo](https://github.com/kevinp1993/AMiRo_Jevois/tree/master/Jevois_AMiRo) des AMiRo_Jevois Repo das cmake ausführen:
```
cmake .
```
5. Das erstellte Makefile ausführen:
```
make
```
6. Das AMiRo Cognition Board über die serielle Schnittstelle starten (z.B. mittels gtkterm) und einloggen mit 
* **Benutzer**: *root* 
* **pw**: *amiropower*
7. Das AMiRo Cognition Board über ein Micro-USB Kabel mit dem Rechner verbinden und Netzwerk einrichten  
8. Die beiden Programme **jevoisAruco** und **jevoisOpticalFlow** können nun  auf das Cognition Board über eine SSH-Schnittstelle kopiert werden. Dazu vorher über die serielle Schnittstelle die IP-Adresse des Cognition Boards herausfinden (standardmäßig ist diese *192.168.1.1*):
```
scp jevoisAruco root@192.168.1.1:~
scp jevoisOpticalFlow root@192.168.1.1:~
```
9. Über eine SSH-Verbidnung mit dem Cognition Board verbinden:
```
ssh root@192.168.1.1
```
10. Programme können nun auf dem AMiRo ausgeführt werden

# ArUco Marker Tracking
In diesem Demo Programm wird ein 4x4 ArUco Marker getrackt ([ArUco Generator](http://chev.me/arucogen/). Das Programm kann mit folgenden Parametern ausgeführt werden:
```
./jevoisAruco P_linear P_angular desired_dist desired_Marker_ID start_Marker_ID
```
* P_linear: Konstante für Regler zum Annähern an den Marker, bis gewünschte Distanz erreicht ist (Default: 500)
* P_angular: Konstante für Regler zum mittig Ausrichten an den Marker (Default: 5000)
* desired_dist: Abstand in mm, welcher zum getrackten Marker eingehalten wird (Default: 300)
* desired_Marker_ID: Zu Trackende Marker ID (Default: 42)
* start_Marker_ID: Marker ID, welche für den Start des Programms nötig ist (Default: 0)

Die wahre Markergröße kann dabei im Code von [jevoisAruco_AmiroDist.cpp](https://github.com/kevinp1993/AMiRo_Jevois/blob/master/Jevois_AMiRo/jevoisAruco_AmiroDist.cpp) in folgender Zeile geändert werden:
```c++
system("echo setpar markerlen 94 > /dev/ttyACM0");
```



