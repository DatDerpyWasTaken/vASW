# Acoustic Profiles

This page lists the acoustic signatures currently modeled in `vASW.py`. Frequencies are nominal class tones before passive Doppler shift.

Most vessel machinery tones stay near their class frequency so contacts remain identifiable. The sim adds a small frequency jitter:

`actual_frequency = base_frequency * random(0.96..1.04)`

Source levels also get a small random jitter of about +/-2 dB. Flow noise is still speed-gated. The sim does not store a separate `blade_count` field; propeller or machinery blade-rate cues are represented by the tone harmonic count and harmonic drop.

## Sub-surface

| Class | Tone | Base Hz | dB | Harmonics | Harmonic drop | Cavitation speed |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Kilo | Propeller | 250 | 87.5 | 3 | 6 | 6 kt |
| Kilo | Pump | 1000 | 77.5 | 2 | 4 | 6 kt |
| Kilo | Cavitation | 1500 | 90 | 1 | default | 6 kt |
| Akula | Propeller | 350 | 97.5 | 3 | 6 | 9 kt |
| Akula | Pump | 1500 | 87.5 | 2 | 4 | 9 kt |
| Akula | Cavitation | 2500 | 95 | 1 | default | 9 kt |
| Delta IV | Propeller | 500 | 107.5 | 3 | 6 | 10 kt |
| Delta IV | Pump | 2000 | 97.5 | 2 | 4 | 10 kt |
| Delta IV | Cavitation | 2800 | 102.5 | 1 | default | 10 kt |
| Borei | Propeller | 300 | 92.5 | 3 | 6 | 12 kt |
| Borei | Pump | 1500 | 82.5 | 2 | 4 | 12 kt |
| Borei | Cavitation | 2500 | 90 | 1 | default | 12 kt |
| Yasen | Propeller | 500 | 95 | 3 | 6 | 14 kt |
| Yasen | Pump | 1700 | 82.5 | 2 | 4 | 14 kt |
| Yasen | Cavitation | 3000 | 87.5 | 1 | default | 14 kt |
| Emitter | Propeller | 500 | 100 | 3 | 6 | 0 kt |
| Emitter | Pump | 1000 | 100 | 2 | 4 | 0 kt |
| Emitter | Cavitation | 1500 | 100 | 1 | default | 0 kt |

## Military Surface Ships

| Profile | Applies to | Tone | Base Hz | dB | Ref speed | Harmonics | Harmonic drop |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| Destroyer | Destroyer | Gas Turbine | 120 | 112 | 20 kt | 4 | 5 |
| Destroyer | Destroyer | Propeller | 245 | 108 | 20 kt | 6 | 5 |
| Destroyer | Destroyer | Gearbox | 980 | 92 | 20 kt | 3 | 6 |
| Destroyer | Destroyer | Pump | 1850 | 84 | 22 kt | 2 | 6 |
| Frigate | Frigate | Diesel Turbine | 105 | 108 | 18 kt | 4 | 5 |
| Frigate | Frigate | Propeller | 215 | 104 | 18 kt | 5 | 5 |
| Frigate | Frigate | Generator | 820 | 90 | 18 kt | 3 | 6 |
| Frigate | Frigate | Pump | 1650 | 82 | 20 kt | 2 | 6 |
| Carrier | Carrier | Main Machinery | 60 | 118 | 18 kt | 5 | 4 |
| Carrier | Carrier | Propeller | 125 | 114 | 18 kt | 6 | 5 |
| Carrier | Carrier | Reduction Gear | 360 | 101 | 18 kt | 4 | 5 |
| Carrier | Carrier | Aux Machinery | 720 | 94 | 18 kt | 3 | 6 |
| Warship | Unknown military surface class | Machinery | 100 | 106 | 18 kt | 4 | 5 |
| Warship | Unknown military surface class | Propeller | 205 | 102 | 18 kt | 5 | 5 |
| Warship | Unknown military surface class | Generator | 760 | 88 | 18 kt | 2 | 6 |

Military surface ships at 18 kt or faster also add `Flow Noise`: 2400 Hz, 86 dB, reference speed 22 kt, 1 harmonic, harmonic drop 6.

## Civilian Surface Ships

Civilian GAIST model titles are matched by keyword, then assigned one of these profiles.

| Profile | GAIST title keywords | Tone | Base Hz | dB | Ref speed | Harmonics | Harmonic drop |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| Fishing | fishing, trawler, shen | Diesel | 95 | 94 | 10 kt | 4 | 5 |
| Fishing | fishing, trawler, shen | Propeller | 185 | 91 | 10 kt | 5 | 6 |
| Fishing | fishing, trawler, shen | Generator | 740 | 78 | 10 kt | 2 | 5 |
| Yacht | yacht, motoryacht, pleasure, sail | Propeller | 145 | 84 | 16 kt | 3 | 7 |
| Yacht | yacht, motoryacht, pleasure, sail | Generator | 520 | 74 | 16 kt | 2 | 6 |
| Yacht | yacht, motoryacht, pleasure, sail | Aux Pump | 1180 | 68 | 16 kt | 1 | 6 |
| Tug | tug, asd, supply, psv | Diesel | 80 | 102 | 12 kt | 5 | 5 |
| Tug | tug, asd, supply, psv | Propeller | 160 | 100 | 12 kt | 6 | 5 |
| Tug | tug, asd, supply, psv | Hydraulic | 620 | 86 | 12 kt | 2 | 6 |
| Ferry | ferry, ro-ro, roro, passenger | Diesel | 115 | 99 | 18 kt | 4 | 5 |
| Ferry | ferry, ro-ro, roro, passenger | Propeller | 235 | 96 | 18 kt | 5 | 6 |
| Ferry | ferry, ro-ro, roro, passenger | Generator | 900 | 82 | 18 kt | 2 | 6 |
| Tanker | tanker, lng, lpg, oil | Slow Diesel | 55 | 108 | 14 kt | 5 | 4 |
| Tanker | tanker, lng, lpg, oil | Propeller | 110 | 104 | 14 kt | 6 | 5 |
| Tanker | tanker, lng, lpg, oil | Cargo Pump | 410 | 86 | 14 kt | 3 | 6 |
| Cargo | default civilian fallback | Diesel | 70 | 104 | 16 kt | 5 | 5 |
| Cargo | default civilian fallback | Propeller | 140 | 100 | 16 kt | 6 | 5 |
| Cargo | default civilian fallback | Generator | 480 | 84 | 16 kt | 2 | 6 |

Civilian surface ships at 18 kt or faster also add `Flow Noise`: 2100 Hz, 78 dB, reference speed 20 kt, 1 harmonic, harmonic drop 6.

## Biological Contacts

The contact creation menu has `Whale`, `Dolphin`, and `Krill` under `Biological`, but the current simulation does not assign acoustic tones to them yet. They also do not map to GAIST models. As implemented, a manually created biological contact is acoustically silent until a biological profile is added.

## Weapons And Synthetic Sources

| Source | Tone | Base Hz | dB | Harmonics | Harmonic drop |
| --- | --- | ---: | ---: | ---: | ---: |
| DICASS ping contact | FMCW sweep | current DICASS sweep frequency | current source level | default | default |
| Torpedo runner | Torpedo Runner | `650 + speed_kt * 8` | 108 | 2 | 7 |
| Torpedo explosion | Torpedo Explosion | 120 | 145 | default | default |
