# Stanowisko odwroconego wahadla - warstwa I/O ESP32 (ESP-IDF / FreeRTOS)

ESP32 pelni role **warstwy czasu rzeczywistego I/O**: naped (stepper), enkoder,
krancowka, safety, telemetria. Regulator wahadla bedzie liczony na PC -
**ESP32 nie liczy zadnego regulatora**.

## Struktura

```
ESP-IDF-PRODUCTION/
  platformio.ini            # env upesy_wroom, framework = espidf
  CMakeLists.txt            # projekt IDF
  sdkconfig.defaults        # m.in. CONFIG_FREERTOS_HZ=1000
  src/
    main.c                  # app_main + io_task (UART <-> PC)
    board_config.h          # piny, polaryzacje, timingi, UART
    CMakeLists.txt
  components/
    proto/                  # framing + CRC16 + struct telemetrii
    encoder/                # ISR quadrature A/B, licznik 32-bit signed
    motor/                  # motor_driver (backend LEDC) + motor_task
    safety/                 # odczyt LIMIT + fault latch + safety_task
  scripts/
    pc_console.py           # konsola testowa PC (pyserial)
    collect_near_upright.py # PC-loop: prosta regulacja + zapis danych CSV
    collect_motion_sequence.py # open-loop sekwencje ruchu + CSV
    collect_smooth_excitation.py # plynne wzbudzenie wielosinusoida + CSV
```

## Taski FreeRTOS

| Task          | Rdzen | Prio | Rola |
|---------------|:----:|:----:|------|
| `safety_task` | 1 | 6 | odczyt LIMIT 500 Hz, zatrzask fault |
| `motor_task`  | 1 | 5 | komendy -> naped, rampa, blokada przy fault |
| `io_task`     | 0 | 4 | parsowanie RX + CRC, kolejka komend, telemetria TX, timeout |
| `encoder`     | - | ISR | przerwania A/B, atomowy snapshot count |

Synchronizacja: kolejka FreeRTOS na komendy + atomowe flagi (portMUX) dla
fault/enable. ISR sa minimalne (odczyt 2 pinow + tablica + inkrement).

## Pinout (ESP32 WROOM)

| Sygnal | GPIO | Uwagi |
|--------|:----:|-------|
| STEP   | 21 | wyjscie LEDC (square wave), czytane zwrotnie przez PCNT |
| DIR    | 19 | wyjscie, jednoczesnie wejscie kierunku PCNT |
| EN     | 18 | wyjscie, **software**, domyslnie aktywny LOW |
| ENC_A  | 13 | wejscie, pull-up, ISR |
| ENC_B  | 25 | wejscie, pull-up, ISR |
| LIMIT  | 22 | wejscie, **aktywne HIGH**, pull-down |
| START  | 35 | wejscie lokalne, aktywne LOW (GPIO35 input-only: **wymaga zewn. pull-up**) |
| UART0 TX | 1 | port USB (mostek devkitu) |
| UART0 RX | 3 | port USB (mostek devkitu) |

> Cala komunikacja z PC idzie po **UART0 = tym samym porcie USB** co
> programowanie (460800). Aby logi nie mieszaly sie z binarna telemetria,
> ESP ma dwa **tryby wyjscia** przelaczane komenda `SET_OUTPUT_MODE` (0x08):
> `BINARY` (czyste ramki, logi wyciszone) i `DEBUG` (czytelny tekst + `ESP_LOG`).
> Po starcie tryb = DEBUG; `pc_console.py` sam przelacza na BINARY.

## Backend napedu

Warstwa abstrakcji `motor_driver` (`components/motor`). Backend v1 generuje
stala czestotliwosc STEP przez **LEDC** (timer0/ch0, 50% duty), kierunek przez
DIR, EN software'owo. Przyspieszenie realizuje rampa w `motor_task`
(`motor_driver_tick`). API (`motor_set_speed_hz`, `motor_set_acc`,
`motor_enable`, `motor_stop`) jest stabilne pod podmiane backendu na RMT /
timer pulse / dedykowany sterownik bez zmian w warstwie wyzszej.

Glownym sygnalem sterujacym jest **predkosc** (Hz krokow, znak = kierunek),
`SET_ACC` ogranicza rampe `dHz/dt`. ESP nie zamyka zadnej petli.

### Pozycja wozka i soft-limity

Pozycja wozka liczona jest **sprzetowo przez PCNT**: impulsy STEP (GPIO21,
odczyt zwrotny wyjscia LEDC) zliczane z kierunkiem z DIR (GPIO19). Licznik HW
jest 16-bit, wiec na progach +/-10000 ISR akumuluje do **32-bit signed**
(`motor_get_position_steps`). To dokladny licznik realnych krokow, nie estymata.

`MOTOR_SOFT_LIMIT_STEPS` (`board_config.h`, domyslnie 12000, 0 = wyl.) blokuje
ruch "na zewnatrz" poza +/- limit, **zanim** wozek dojedzie do krancowki -
ruch powrotny do srodka jest dozwolony. Soft-limit NIE jest faultem (krancowka
LIMIT to osobny, zatrzaskowy fault).

Zero pozycji ustawia komenda **SET_ZERO** (homing realizuje PC: dojechac do
punktu odniesienia / krancowki, potem `zero`). Soft-limity maja sens dopiero po
ustawieniu zera.

## Protokol UART

Ramka:

```
[SOF=0xAA][ID][LEN][PAYLOAD ... LEN][CRC16_LO][CRC16_HI]
```

CRC16-CCITT (poly `0x1021`, init `0xFFFF`) po `[ID, LEN, PAYLOAD]`, LE.

Komendy PC -> ESP32:

| ID | Komenda | Payload |
|----|---------|---------|
| 0x01 | PING | - |
| 0x02 | SET_ENABLE | uint8 (0/1) |
| 0x03 | SET_SPEED_HZ | float32 LE (znak = kierunek) |
| 0x04 | SET_ACC | uint32 LE (Hz/s) |
| 0x05 | RESET_FAULT | - |
| 0x06 | GET_STATUS | - |
| 0x07 | SET_ZERO | - (zeruje pozycje wozka) |
| 0x08 | SET_OUTPUT_MODE | uint8 (0=binary, 1=debug) |

Odpowiedzi/telemetria ESP32 -> PC: `PONG 0x81`, `ACK 0x82`, `NACK 0x83`,
`TELEMETRY 0x84`. Telemetria binarna leci okresowo **250 Hz**, payload
(packed, LE, 29 B):

```
uint64 timestamp_us | int32 encoder_count | int32 position_steps |
float applied_speed_hz | uint32 applied_acc | uint8 limit_state |
uint8 fault_state | uint8 drive_enabled | uint8 soft_limit_state |
uint8 start_state
```

## Build / flash

```powershell
# w katalogu projektu
pio run                 # kompilacja
pio run -t upload       # wgranie
pio device monitor      # log na UART0 (460800)
```

(Jesli `pio` nie jest w PATH:
`& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run`)

## Instrukcja testow

Wystarczy **port USB devkitu** (ten sam co do wgrywania). Po stronie PC:
`pip install pyserial`. `<COM>` to ten sam port co w `pio device monitor`.

> Uwaga: port jest jeden, wiec naraz korzysta z niego albo `pc_console.py`
> (tryb BINARY), albo `pio device monitor` (tryb DEBUG). Do czytelnego podgladu
> tekstem: `python scripts/pc_console.py --port <COM> mode dbg`, zamknij skrypt
> i otworz `pio device monitor`. `pc_console.py` przy starcie wraca do BINARY.

### 1. Przykladowa komenda + zmiana predkosci

```powershell
python scripts/pc_console.py --port COM5 ping       # -> PONG
python scripts/pc_console.py --port COM5 enable 1   # -> ACK, EN aktywny
python scripts/pc_console.py --port COM5 speed 4000 # -> ACK, STEP ~4 kHz
python scripts/pc_console.py --port COM5 status     # -> TLM z applied_speed_hz
```

W telemetrii widac tez `pos` (position_steps) zmieniajacy sie z ruchem oraz
rampe `applied_speed_hz`:

```powershell
python scripts/pc_console.py --port COM5 monitor 3
```

> Uwaga: pojedyncze komendy ida bez heartbeatu, wiec po 200 ms zadziala
> bezpieczny stop. Do ciaglego ruchu uzyj `heartbeat` (ponizej).

### 2. Reakcja na LIMIT (fault latch)

1. Uruchom ruch z heartbeatem (PING co 50 ms utrzymuje komunikacje):

   ```powershell
   python scripts/pc_console.py --port COM5 heartbeat 4000 30
   ```

2. Zewrzyj krancowke (LIMIT -> HIGH, np. GPIO22 do 3V3).
3. W telemetrii natychmiast: `fault=1`, `speed=0`, `en=0` - pulsy STEP gasna.
4. Fault **trzyma sie** (latch) nawet po zwolnieniu krancowki.
5. Odblokowanie: zwolnij krancowke i wyslij `reset`, potem ponownie `enable`:

   ```powershell
   python scripts/pc_console.py --port COM5 reset    # -> ACK, fault=0
   python scripts/pc_console.py --port COM5 enable 1
   ```

   `reset` przy aktywnej krancowce nie skasuje fault (telemetria wciaz `fault=1`).

### 3. Weryfikacja timeoutu safety

1. Wlacz naped i predkosc bez heartbeatu:

   ```powershell
   python scripts/pc_console.py --port COM5 enable 1
   python scripts/pc_console.py --port COM5 speed 4000
   ```

2. Przestan wysylac komendy i obserwuj telemetrie:

   ```powershell
   python scripts/pc_console.py --port COM5 monitor 2
   ```

3. Po ~200 ms bez komend `applied_speed_hz` spada do 0 (bezpieczny stop).
   Wznowienie ruchu wymaga ponownej komendy `speed` (oraz heartbeatu).

### 4. Pozycja wozka i soft-limity

1. Ustaw zero w punkcie odniesienia (po reczny/programowym homingu):

   ```powershell
   python scripts/pc_console.py --port COM5 zero    # -> ACK, pos=0
   ```

2. Ruchem sprawdz, ze `pos` rosnie/maleje zgodnie z kierunkiem (znak speed).
3. Dojezdzajac do +/-`MOTOR_SOFT_LIMIT_STEPS` telemetria pokazuje `soft=1`,
   a `applied_speed_hz` spada do 0 w kierunku "na zewnatrz". Komenda z
   przeciwnym znakiem (powrot do srodka) dziala normalnie.

### 5. Zbieranie danych blisko pionu z PC

`scripts/collect_near_upright.py` uruchamia prosta petle PC: odbiera binarna
telemetrie, estymuje `theta`, `theta_dot`, `x_dot`, wysyla `SET_SPEED_HZ` i
zapisuje probki state/action do CSV. Domyslna procedura: START musi byc OFF,
uruchom skrypt, ustaw recznie wahadlo blisko pionu, nacisnij lokalny START.
Dopiero po przejsciu `start_state` z 0 na 1 skrypt wysyla `enable=1` i zaczyna
regulacje z PC. Jesli skrypt od razu widzi `start_state=1`, konczy prace bez
wlaczania napedu.

Pierwszy test wykonaj z malym limitem predkosci i reka przy awaryjnym
wylaczeniu. Domyslne znaki `--k-theta -160000 --k-omega -12000` sa dobrane do
aktualnego kierunku enkodera/napedu. Jesli po zmianach okablowania reakcja
ucieka od pionu, odwroc znaki obu czlonow katowych.

```powershell
python scripts/collect_near_upright.py --port COM5 --seconds 10 --max-speed-hz 3000
```

Tryb pasywny tylko loguje telemetrie i wysyla zerowa predkosc:

```powershell
python scripts/collect_near_upright.py --port COM5 --mode passive --seconds 10
```

Przydatne parametry startowe:

- `--theta-center-counts 4001` - licznik enkodera dla pionu ze starego testu.
- `--encoder-ppr 2000 --decode x4` - odpowiada `ENCODER_CPR=8000`.
- `--zero-cart` - wysyla `SET_ZERO`; uzywac tylko po ustawieniu wozka w punkcie
  odniesienia.
- `--arm start` - domyslnie czeka na lokalny START z telemetrii.
- `--arm immediate` - start bez czekania na lokalny START, tylko do testow.
- `--ignore-start-off` - nie konczy sesji po powrocie START do OFF.
- `--allow-start-already-on` - pozwala wystartowac mimo START=ON przy starcie
  skryptu; tylko do diagnostyki.
- `--max-angle-rad`, `--max-position-steps` - limity bezpiecznego przerwania.
- `--excite-hz` - maly losowy sygnal dodany do komendy, przydatny pozniej do
  zbierania bogatszych danych identyfikacyjnych.

### 6. Sekwencje ruchu od dolnej pozycji

`scripts/collect_motion_sequence.py` wykonuje otwarta cykliczna sekwencje
predkosci wozka i zapisuje osobny CSV dla kazdego START. Procedura: START=OFF,
uruchom skrypt, ustaw wahadlo w pozycji poczatkowej, nacisnij START. Po
sekwencji skrypt zatrzymuje naped i czeka na kolejne OFF -> ON dla nastepnej
proby.

Domyslnie jeden START wykona 3 cykle:
`+1500 Hz przez 0.25 s`, `stop 0.40 s`, `-1500 Hz przez 0.25 s`,
`stop 0.40 s`.

```powershell
python scripts/collect_motion_sequence.py --port COM5
```

Parametry eksperymentu:

```powershell
python scripts/collect_motion_sequence.py --port COM5 --speed-hz 2000 --move-s 0.20 --stop-s 0.50 --cycles 5
```

`--experiments 3` zbierze trzy osobne pliki CSV, po jednym na kazdy START.
`--repeat` czeka na kolejne START bez limitu. Jesli pierwszy ruch jest w zla
strone, uzyj `--first-direction negative`.

Opcjonalnie mozna dodac gladki losowy skladnik do predkosci podczas ruchu:

```powershell
python scripts/collect_motion_sequence.py --port COM5 --speed-hz 2000 --move-s 0.20 --stop-s 0.50 --cycles 5 --noise-hz 300 --noise-period-s 0.15 --noise-alpha 0.2
```

`--noise-hz` to amplituda zaklocenia w Hz, `--noise-period-s` jak czesto
losowany jest nowy cel szumu, a `--noise-alpha` okresla gladkosc dochodzenia do
tego celu. Domyslnie szum nie dziala podczas postojow; `--noise-on-stop`
wlacza go takze dla faz `stop_*`.

Pliki trafiaja domyslnie do `scripts/logs_sequence/`. W CSV jest m.in.
`trial_idx`, `cycle_idx`, `phase`, `segment_idx`, `segment_t_s`,
`base_speed_hz`, `noise_speed_hz`, `command_speed_hz`, `applied_speed_hz`,
`theta_rad`, `position_steps` i `stop_reason`.

### 7. Plynne wzbudzenie do identyfikacji modelu

`scripts/collect_smooth_excitation.py` generuje deterministyczna sume sinusoid
o roznych czestotliwosciach i losowych fazach. Dla tego samego `--seed` sygnal
jest identyczny. Jest plynny, ograniczony amplitudowo i pobudza kilka skal
czasowych jednoczesnie, co daje bogatsze dane do identyfikacji niz pojedyncza
stala predkosc lub skok lewo/prawo.

Skrypt czeka na START OFF -> ON, lagodnie zwieksza i zmniejsza amplitude oraz
dodaje mala korekcje utrzymujaca wozek blisko srodka. W CSV osobno zapisywane
sa: wzbudzenie, korekcja pozycji, tlumienie predkosci, finalna komenda i
predkosc rzeczywiscie wykonana przez ESP32.

Bezpieczna komenda poczatkowa:

```powershell
python scripts/collect_smooth_excitation.py --port COM5 --duration-s 20 --peak-hz 3000 --min-freq-hz 0.12 --max-freq-hz 2.0 --components 9 --seed 1
```

Kilka roznych, powtarzalnych eksperymentow:

```powershell
python scripts/collect_smooth_excitation.py --port COM5 --duration-s 25 --peak-hz 3500 --experiments 5 --seed 10
```

Kolejne proby automatycznie uzywaja `seed`, `seed+1`, itd. Pliki sa zapisywane
do `scripts/logs_excitation/`.

## Kryteria akceptacji - mapowanie

- Kompiluje sie w ESP-IDF bez recznego latania - `pio run` = SUCCESS.
- `SET_SPEED_HZ` zmienia predkosc - `motor_task` + LEDC, widoczne w telemetrii.
- LIMIT HIGH natychmiast zatrzymuje i latchuje - `safety_task` (500 Hz) + latch.
- `RESET_FAULT` odblokowuje - `safety_reset_fault` (gdy krancowka nieaktywna).
- Telemetria okresowa, spojne pola - `io_task`, ~250 Hz, struct packed.
- Brak regulatora po stronie ESP - ESP robi tylko I/O.
- Pozycja wozka + soft-limity - PCNT (STEP/DIR), `motor_get_position_steps`.
