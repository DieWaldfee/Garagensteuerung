# Garagensteuerung
Steuerung eines Garagentorantriebs (Platine und Software) via Relaistrigger

### Beschreibung der Funktion:
Garagentorantriebe haben einen potentialfreien Relais-Kontakt. Wird dieser gebrückt, so wird der Antrieb aktiviert für eine Öffnen/Schließen -> das Tor fährt auf den nächsten Anschlag (oben oder unten). Wo das Tor aktuell steht und welche Richtung die Fahrt nehmen wird ist so nicht sensierbar.<br>
Im dazugekörigen blockly-Skript für ioBroker wird daher geprüft, welcher Endanschlag angefahren wird, um ggf. über einen weiteren Trigger (Triggerimpuls über das Relais) das Tor in die gewünschte Position zu bringen. Eine weitere Funktion ist - analog zu Rolladensteuerungen - eine Behanghöhe einzustellen. die wird über die Öffnungszeit / Schließzeit bestimmt -> 10% Behanghöhe entspricht damit 10% Öffnungszeit bis das Relais den Trigger zum stoppen des Torantriebs sendet- bzw. 90% Schließzeit, sollte das Tor am oberen Anschlag sein. (steht das Tor irgendwo, dann wird zuerst ein Anschlag angefahren, um dann den Behang einzustellen).

### Hardware:
Die Platine ist in dieser Version für eine Lochrasterplatine ausgelegt -> Leiterbahnen können mit einem Draht und Lötzinn erstellt werden. Dies ermöglicht es eigentlich jedem, der einen Lötkolben sein Eigen nennt es einfach nachzubauen :-) Die Eagle-Dateien liegen im Repository. Ein Überblick gibt das Fritzing-File. <br>
An der Platine können 4 Reed-Kontakte angeschlossen werden (oberer und unterer Anschlag, sowie zwei optionale Positionen). Über die Reedkontakte "Oben und Unten" wird die aktuelle Position des Tors bestimmt. Ein Relais dient dem Triggern des Garagenantriebs. Die STromversorgung kann entweder über ein Mini-Netzteil, oder einfach über ein USB-Netzteil realisiert werden (Letzteres ist bei mir im Einsatz). Drei LEDS zeigen ggf. Fehler, das Senden von MQTT-Botschaften und generell den Betrieb an. Optional kann ein DS18B20-Temperatursensor angeschlossen werden, sollte die Garagentemperatur von Interesse sein (weil der Wohnwagen dort steht, oder die Pflanzen überwintern sollen...).

### Software:
Der ESP32 benötigt natürlich seine Software. Das C++-File liegt im entsprechenden Verzeichnis und muss bezüglich MQTT-Broker-Account und Adresse angepasst werden. Zusätzlich ist natürlich auch das WiFi-Passwort und die IP-Adresse zu setzen. Danach kann das File z.B. mit der Arduino-IDE auf den ESP32DevKitV4-Modul aufgespielt werden. Compile und Upload brauchen nur ein paar Sekunden. Nach dem Start des ESP32 versucht der ESP32 den MQTT-Broker zu erreichen und übermittelt zyklisch den Status. Das angehängte blockly-Skript für den ioBroker liest die Botschaften aus und steuert das Tor an. Hier sind die Funktionen TorZu(um 23Uhr)... verortet. Der gesendete Torzustand kann im ioBroker zur Visualisierung verwendet werden. Ich steure den Telegramadapter an, sollte z.B. das Tor um 23Uhr nicht zu schließen sein (weil z.B. mein Sohn das Fahrad voll in den Fahrweg des Tores gestellt hat...).

### Benötigte Hardware:
* Relais <a href="https://www.amazon.de/gp/product/B0B5816YJ7/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&th=1"> Amazon </a> oder <a href="https://www.az-delivery.de/products/relais-modul"> AZ Delivery </a>
* ESP32 Dev Kit V4 <a href="https://www.azdelivery.de/products/esp-32-dev-kit-c-v4"> AZ Delivery </a>
* optional Temperatursensoren DS18B20 <a href="https://www.az-delivery.de/products/2er-set-ds18b20-mit-3m-kabel"> AZ Delivery </a>
* JST-Buchse <a href="https://www.amazon.de/gp/product/B0B2R99X99/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1"> Amazon </a>
* Klemmbuchse <a href="https://www.amazon.de/gp/product/B087RN8FDZ/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1"> Amazon </a>
* Widerstände 4,7kOhm, 220 Ohm, 330 Ohm, Led grün, gelb und rot Amazon / eBay / Conrad...
* Levelshifter (3.3V <-> 5V)  <a href="https://www.amazon.de/RUNCCI-YUN-Pegelwandler-Converter-BiDirektional-Mikrocontroller/dp/B082F6BSB5/ref=sr_1_2?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=45TPZ9B8CUP9&keywords=level+shifter&qid=1699045033&sprefix=level+shifter%2Caps%2C103&sr=8-2"> Amazon </a>
* Reed-Kontakte <a href="https://www.amazon.de/schwarz-Sicherheitskontaktsensor-magnetischer-Reed-Schalter-T%C3%BCrsensor/dp/B07Z4NCWDD/ref=sr_1_3_sspa?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=39FKQI24X72OK&keywords=reedkontakt&qid=1703368970&sprefix=reedkontakt%2Caps%2C111&sr=8-3-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1)"> Amazon </a>
* USB-Netzteil Amazon / eBay / Conrad...
* optional: Platinennetzteil AC-05-3    <a href="https://www.azdelivery.de/products/copy-of-220v-zu-5v-mini-netzteil"> AZ-Delivery </a>
* MC1490P (debouncer) auf Sockel, wenn gewünscht <a href="https://www.ebay.de/itm/201551880799?_trkparms=amclksrc%3DITM%26aid%3D1110006%26algo%3DHOMESPLICE.SIM%26ao%3D1%26asc%3D257778%26meid%3Db537b25c7a0745c29ea2a72120fca1d0%26pid%3D101195%26rk%3D5%26rkt%3D6%26sd%3D191674185616%26itm%3D201551880799%26pmt%3D1%26noa%3D0%26pg%3D4429486%26algv%3DSimPLWebV1EmbeddedAuctionsCPCAuto%26brand%3DBrand&_trksid=p4429486.c101195.m1851&amdata=cksum%3A201551880799b537b25c7a0745c29ea2a72120fca1d0%7Cenc%3AAQAIAAABAKQWTaSuKxcpq1j5hG%252Bz9F%252B8zK5%252FOpV48i2l8raehpQfSfTJViiimjfvkWmDgf%252BDpe3yyQwlK3DiDnYY85z1SxQJboT7HuhUjq4JgvlcaxtWrksoQQD9wDjrKbegZu4xYn3PZViDDDYymbWuHYvibUP57v0yaiUMN0YTUgeor4tEO0YIhs7PQ6%252BlbaxxWmKa98vDVPkxH4swvs1LeiFRFmmZbEV7QLNVZZrDm%252FVKXslR3RU%252FaKtNAYSS467lqopMuORWt353Z%252F97IyDlsTditsZk7nQ5%252BrWSFDjYWzKfd28Sj%252BQeszMJWaCxAHMu4uSDmqWne6J1qTMShVB5vi5c19k%253D%7Campid%3APL_CLK%7Cclp%3A4429486"> eBay </a>
* Kondensatoren <a href="https://www.amazon.de/AUKENIEN-Kondensator-Kondensatoren-Keramikkondensator-Kit/dp/B09NLZBC7R/ref=sr_1_1_sspa?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=HKCZID75QOX1&keywords=kondensator&qid=1703369261&sprefix=kondensator%2Caps%2C94&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1"> Amazon </a>

<br>(Bezugslinks füge ich später noch hinzu)


