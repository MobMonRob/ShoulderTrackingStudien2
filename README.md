# ShoulderTrackingStudien2
Tracking of shoulder motion with 3d cameras.


Diese Repository stellt Skripte zur Erkennung und Verfolgung von retroreflektierenden Markern bereit.

Verwendung:
- Repository klonen
- Folgende Dependencys müssen (z.b. über pip install ) installiert werden:
	- numpy
	- opencv
	- pyrealsense2
	- sklearn

- Gestartet wird das Programm über das "tracking.py" Skript. 

Wenn die Kamera(s) neu positioniert wurden muss eine erneute Kalibrierung vorgenommen werden. Dies geschieht mit Hilfe eines Schwarz/Weiß-Schachbretts.
Wir empehlen dazu dieses hier https://github.com/opencv/opencv/blob/3.4/doc/pattern.png, wird ein anderes Muster verwendet müssen in dem Skript "calibrate_cams.py" die entsprechenden Variablen angepasst werden.
Das Speichern und Laden von Kalibrierungsinformationen ist möglich. Dazu die entsprechenden Methodenaufrufe in der main-Methode aus/ein-kommentieren.


Die Skripte wurden mit Python 3.7 erstellt und getestet.
