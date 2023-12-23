# Garagensteuerung
Steuerung eines Garagentorantriebs (Platine und Software) via Relaistrigger
Beschreibung der Funktion:
Garagentorantriebe haben einen potentialfreien Relais-Kontakt. Wird dieser gebrückt, so wird der Antrieb aktiviert für eine Öffnen/Schließen -> das Tor fährt auf den nächsten Anschlag (oben oder unten). Wo das Tor aktuell steht und welche Richtung die Fahrt nehmen wird ist so nicht sensierbar.
Im dazugekörigen blockly-Skript für ioBroker wird daher geprüft, welcher Endanschlag angefahren wird, um ggf. über einen weiteren Trigger (Triggerimpuls über das Relais) das Tor in die gewünschte Position zu bringen. Eine weitere Funktion ist - analog zu Rolladensteuerungen - eine Behanghöhe einzustellen. die wird über die Öffnungszeit / Schließzeit bestimmt -> 10% Behanghöhe entspricht damit 10% Öffnungszeit bis das Relais den Trigger zum stoppen des Torantriebs sendet- bzw. 90% Schließzeit, sollte das Tor am oberen Anschlag sein. (steht das Tor irgendwo, dann wird zuerst ein Anschlag angefahren, um dann den Behang einzustellen).

Hardware:
Die Platine ist in dieser Version für eine Lochrasterplatine ausgelegt -> Leiterbahnen können mit einem Draht und Lötzinn erstellt werden. Dies ermöglicht es eigentlich jedem, der einen Lötkolben sein Eigen nennt es einfach nachzubauen :-) Die Eagle-Dateien liegen im Repository. Ein Überblick gibt das Fritzing-File. 
An der Platine können 4 Reed-Kontakte angeschlossen werden (oberer und unterer Anschlag, sowie zwei optionale Positionen). Über die Reedkontakte "Oben und Unten" wird die aktuelle Position des Tors bestimmt. Ein Relais dient dem Triggern des Garagenantriebs. Die STromversorgung kann entweder über ein Mini-Netzteil, oder einfach über ein USB-Netzteil realisiert werden (Letzteres ist bei mir im Einsatz). Drei LEDS zeigen ggf. Fehler, das Senden von MQTT-Botschaften und generell den Betrieb an. Optional kann ein DS18B20-Temperatursensor angeschlossen werden, sollte die Garagentemperatur von Interesse sein (weil der Wohnwagen dort steht, oder die Pflanzen überwintern sollen...).

Software:
Der ESP32 benötigt natürlich seine Software. Das C++-File liegt im entsprechenden Verzeichnis und muss bezüglich MQTT-Broker-Account und Adresse angepasst werden. Zusätzlich ist natürlich auch das WiFi-Passwort und die IP-Adresse zu setzen. Danach kann das File z.B. mit der Arduino-IDE auf den ESP32DevKitV4-Modul aufgespielt werden. Compile und Upload brauchen nur ein paar Sekunden. Nach dem Start des ESP32 versucht der ESP32 den MQTT-Broker zu erreichen und übermittelt zyklisch den Status. Das angehängte blockly-Skript für den ioBroker liest die Botschaften aus und steuert das Tor an. Hier sind die Funktionen TorZu(um 23Uhr)... verortet. Der gesendete Torzustand kann im ioBroker zur Visualisierung verwendet werden. Ich steure den Telegramadapter an, sollte z.B. das Tor um 23Uhr nicht zu schließen sein (weil z.B. mein Sohn das Fahrad voll in den Fahrweg des Tores gestellt hat...).

Benötigte Hardware:
- Relais-Modul
- ESP32 Dev Kit V4
- optional Temperatursensor DS18B20
- JST Stecker und Buchsen
- Level-Shifter (3.3V - 5V)
- Reed-Kontakte
- USB-Netzteil oder Mininetzteil
- LED und Widerstände
- MC1490P (debouncer) auf Sockel, wenn gewünscht
- Kondensatoren

(Bezugslinks füge ich später noch hinzu)


