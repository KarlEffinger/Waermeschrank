# Wärmeschrank
Ein ESP32 Projekt zur Ansteuerung einer Heizung (bei mir getestet mit einer Heizmatte wie auch mit einem Heißluftgebläse).
Ich nutze diesen Wärmeschrank zum Verflüssigen meines Honigs, damit ich ihn dann mit meinem HaniMandl abfüllen kann ;-)
Per Drehgeber wird sowohl eine Solltemperatur als auch eine Ziel-Laufzeit eingestellt.
Sobald die Zieltemperatur das erste Mal erreicht wurde, beginnt die Laufzeit. Wird die Ziellaufzeit erreicht, schaltet sich die Heizung dauerhaft ab.
(Dauerhaft, weil aktuell noch eine 2-Punkt-Regelung mit +/- 0.5°C Hysterese und Relaissteuerung implementiert sind. Plan wäre, ggf auf eine PID-Regelung umzustellen).

Auf dem angeschlossenen LCD werden die Soll- und Ist-Temperatur, die Ziel-Laufzeit, die bisherige Laufzeit (auf Zieltemperatur) und die Restlaufzeit angezeigt, verbunden mit dem aktuellen Relaisstatus. Darunter gibt es eine Statuszeile mit Informationen zur WLAN- und MQTT-Verbindung, denn der ESP sendet alle 2 Minuten per MQTT seinen aktuellen Status und kann über eingehende MQTT-Nachrichten auch gesteuert werden.
Die Einrichtung der WLAN- und MQTT-Verbindung erfolgt über einen Accesspoint, den der ESP aufspannt, wenn der Taster des Drehgebers länger als 3 Sekunden gedrückt wird.

