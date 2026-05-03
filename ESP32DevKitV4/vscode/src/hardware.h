// hardware.h
#pragma once

// ---------------------------------------------------------------------------
// Torspezifische Einstellungen
// ---------------------------------------------------------------------------
// Verfahrzeiten des Tors - diese Werte müssen an die individuellen Gegebenheiten angepasst werden
// Zeit in Sekunden, die das Tor zum vollständigen Öffnen (ZU -> AUF) bzw. Schließen (AUF -> ZU)
// benötigt. Die Zeit ist in Sekunden anzugeben und sollte mit einem realen Testlauf des Tors ermittelt
// werden. Ein geringer Zeitpuffer (z.B. 1s - je nach eingesetzter Torsteuerung) ist empfehlenswert,
// um die tatsächliche Bewegung des Tors und ggf. Totzeiten in der Torsteuerung zu berücksichtigen.
#define DEFAULT_TIME_TOR_AUF 15.0f
#define DEFAULT_TIME_TOR_ZU 20.0f
// Hysteresezeit bei schnellen Schaltvorgängen Zu-Zu, Auf-Auf oder schnellen Zu-Auf-Schaltungen
// Hier muss ggf. experimentell ermittelt werden. Anpassungen sind nur bei größeren Totzeiten
// in der Torantriebssteuerung nötig.
#define DEFAULT_TIME_HYSTERESE 1.0f

// ---------------------------------------------------------------------------
// Hardwarenahe Einstellungen
// ----------------------------------------------------------------------------
// Einstellungen für das Grundverhalten der Hardware
// Triggerteit = Zeit in ms, die das Relais für die Ansteuerung des Garagentors aktiviert wird
#define TRIGGERTIME 800  // 800ms Triggerzeit für das Garagentor

// Sensorreaktion der Magnetsensoren invertieren - ist herstellerabhängig ob NC oder NO
#define SENSORINVERT 1  // 0 = Magnetsensoren werden nicht invertiert; 1 = Sensoren werden invertiert

// Pinbelegung der Sensoren und Relais
// hier nichts ändern - siehe auch Schaltplan in der Projektbeschreibung
#define LED_ERROR 23
#define LED_MSG 22
#define LED_OK 19
#define REED1 27   // Tor-Zu-Sensor
#define REED2 26   // Zwischenstufe 1
#define REED3 32   // Zwischenstufe 2
#define REED4 33   // Tor-Auf-Sensor
#define RELAIS 18  // Relais zur Torsteuerung
#define ONE_WIRE_BUS 25
