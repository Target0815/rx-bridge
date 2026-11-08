# rx-bridge 0.3.2

FrSky-Empfänger (S.Port, F.Port, FBUS) an Flight-Controllern ohne eigenen Signal-Inverter,
zum Beispiel F405-Boards mit INAV oder ArduPilot. Läuft auf einem fertigen RP2040- oder
RP2350-Modul, wahlweise als Inverter oder als Konverter auf CRSF.

**Firmware herunterladen:** unter [Releases](https://github.com/target0815/rx-bridge/releases)
die passende `.uf2`-Datei für euer Modul.

- **[Anleitung](ANLEITUNG.md)**: aufspielen, anschließen, einstellen, testen
- **[Technische Dokumentation](docs/TECHNIK.md)**: Protokolle, Telemetrie-Kodierung, Aufbau der
  Firmware, Tests, selbst bauen

| Ordner | Inhalt |
|---|---|
| `web/` | Konfigurationsseite, in Chrome oder Edge öffnen |
| `docs/` | technische Dokumentation und Schaltbilder |
| `src/` | Quellcode der Firmware |
| `test/` | automatische Tests |

## Lizenz

GNU General Public License v3.0, siehe [LICENSE](LICENSE).
