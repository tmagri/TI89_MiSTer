# TI89_MiSTer

A [TI-89 Titanium](https://en.wikipedia.org/wiki/TI-89) graphing calculator core for the [MiSTer](https://github.com/MiSTer-devel) FPGA platform.

The core recreates the TI-89 Titanium (HW3) on the Terasic DE10-Nano board: a cycle-accurate Motorola 68000 (via the FX68K core), the 256 KB RAM / 4 MB flash memory map, the memory-mapped I/O registers, the 10×8 keyboard scan matrix, the auto-interrupt/timer logic, and the 160×100 monochrome LCD controller with DMA — all running on real hardware logic rather than software emulation.

> **Legal note:** The calculator's operating system image (`TI89Titanium_OS.89u`) is copyrighted by Texas Instruments. This project does **not** contain or distribute the OS image. You must supply your own `.89u` file and load it at runtime through the MiSTer OSD — the same pattern used for Amiga Kickstart ROMs in Minimig.

## How this was built

The RTL in this repository was written by an LLM working under human direction; all references listed below were used as part of the research. My contributions included research, architectural direction, and the use of existing software and hardware simulations to replicate the processes. Currently, research is focused on loading the OS image into memory and booting the OS from it.

This is stated upfront because it is fair for anyone evaluating the code to know. It is not an endorsement of the approach — draw your own conclusions.

## Core features

* CPU: Motorola 68000 implemented with the [FX68K](https://github.com/ijor/fx68k) cycle-accurate core (~10.67 MHz effective clock)
* RAM: 256 KB (mirrored at $000000 / $200000 / $400000, as on real HW3 hardware)
* Flash: 4 MB window ($800000–$BFFFFF) backed by the DE10-Nano SDRAM, including the Sharp LH28F320BF Write State Machine behavior
* LCD: 160×100 monochrome display with hardware DMA base-address logic, selectable color emulation (green / blue / amber / black & white) and integer scaling (1×–4×)
* Keyboard: full TI-89 keypad mapped from a USB keyboard through the MiSTer I/O stack, with selectable **Mapped** (legacy) or **Natural** (sequenced modifier injection with N-key rollover) keyboard modes
* Interrupts: HW2+ auto-interrupt sources (AI1 timer, AI2 keyboard, AI6 ON key, etc.)
* OS loading: `.89u` OS upgrade files streamed from the SD card via the OSD

## Usage

1. Install the core on your MiSTer (see [Building](#building-from-source) below, or use a prebuilt release `.rbf`).
2. Launch the core from the MiSTer menu.
3. Open the OSD (F12) and select **Load OS Image**, then choose a TI-89 Titanium OS file (`*.89u`).
4. The core parses the image into SDRAM, copies the OS header into RAM, releases the CPU and boots the OS from the flash window.
5. Press **Insert** (the calculator's ON key) if the calculator needs to be powered on.

### OSD options

| Option | Values | Notes |
|---|---|---|
| Load OS Image | `*.89u` | Streams the OS upgrade file into the flash window |
| LCD Color | Green / Blue / Amber / B&W | Emulated LCD tint |
| LCD Scale | 4× / 3× / 2× / 1× | Integer upscale of the 160×100 LCD raster |
| Keyboard Mode | Natural / Mapped | Natural (default): logical translation with sequenced modifier injection and N-key rollover. Mapped: legacy direct-matrix mapping (TiEmu-style) |
| Numpad Mode | Cursor / Digits | Digits forces the numpad to type digits even when Num Lock is off (see [Keyboard](#keyboard)) |
| Reset | — | Cold reset of the calculator |

### Keyboard

Host keyboard input arrives as PS/2 scancodes through the MiSTer `hps_io` stack and is translated in `rtl/keyboard.sv` into the calculator's **10×8 key matrix** (layout taken from TiEmu's `kbd.c`), which the OS polls through the I/O registers at `$600018`–`$60001B`. The ON key is wired separately and triggers the AI6 interrupt directly; every other key press raises AI2.

Two translation modes are available from the OSD (**Natural** is the default):

| | **Natural mode** (default) | **Mapped mode** (legacy) |
|---|---|---|
| Philosophy | Type on a PC keyboard as if it were a TI-89 | Physical wiring of the PC keys onto the matrix |
| PC Shift | Tracked internally; *not* wired to TI SHIFT | Wired directly to TI SHIFT (held = asserted) |
| Modifiers for letters/symbols | Injected automatically per keystroke | ALPHA auto-asserted while a letter key is down; 2ND/♦ held by hand |
| N-key rollover | Yes (16-event FIFO + sequencer) | No — overlapping keys can conflict |
| Best for | Everyday typing and programming | Software that expects TiEmu-style timing |

#### Modifier keys (both modes)

| PC key | TI-89 key |
|---|---|
| Caps Lock | ALPHA |
| Left/Right Ctrl | ♦ (Diamond) |
| Left/Right Alt | 2ND |
| Left/Right Shift | SHIFT |
| Windows/GUI key | SHIFT (Native mode) |

In **Mapped mode** these write directly into the matrix and stay asserted exactly as long as they are held on the PC keyboard. In **Natural mode** Ctrl/Alt/Win/Caps Lock behave the same way (they bypass the sequencer), while PC Shift is *decoupled* from TI SHIFT and is only used to decide *what* each keystroke should produce — so holding Shift does not hold the TI SHIFT key down.

#### Natural mode (default)

Natural mode implements a logical translation pipeline in hardware (`rtl/keyboard.sv`):

1. **Scancode decoder** — every PS/2 scancode is decoded — using the current PC Shift state — into a target matrix position plus a 4-bit *required modifier mask* `{ALPHA, SHIFT, 2ND, ♦}`. For example, `A` decodes to the `=` key requiring ALPHA; `Shift+A` decodes to `=` requiring ALPHA+SHIFT (uppercase); `Shift+8` decodes to the `×` key directly.
2. **Key tracker** — the decoded assignment for each key currently held is remembered per-scancode, so a key *release* clears the same matrix cell it pressed, even if Shift state changed in between (this prevents stuck keys on Shift-release).
3. **Event FIFO** — press/release events enter a 16-deep circular buffer, so nothing is dropped while the sequencer is busy (full N-key rollover at typing speed; auto-repeat is filtered).
4. **Modifier sequencer** — a non-blocking state machine dequeues events and, when a key needs modifiers that are not currently asserted, presses them into the matrix first and waits **5 ms** so the OS scan loop observes them before the key itself. On release, modifiers are only removed after the last key that needed them is released — per-modifier reference counters make overlapping sequences (e.g. `Shift+A` followed immediately by `Shift+B`) work correctly.

Because the decoder is scancode-complete, most of the PC keyboard "just works":

* **Shifted symbols** produce the real TI-89 keystrokes: `!` `^` `*` `(` `)` `+` from the number row, `<`/`>` (2ND+`0` / 2ND+`.`), `[`/`]` (2ND+`,` / 2ND+`÷`), `∠` (2ND+EE) and `π` (2ND+`^`).
* **Letters** produce lowercase (ALPHA + key) and **Shift+letter** produces uppercase (ALPHA+SHIFT+key); `T` `X` `Y` `Z` use their dedicated keys and only need SHIFT when capitalized.
* **Space** inserts a space (ALPHA+(−)); **Shift+Space** or the **`-`** key gives the negate/(−) key, and **Shift+`-`** gives the binary minus operator.

#### Mapped mode (legacy)

Mapped mode mirrors the behavior of the TiEmu software emulator: modifier keys are mapped directly onto the keyboard matrix and remain asserted for as long as they are held on the PC keyboard, and letter keys (`A`–`W`) auto-assert ALPHA with a reference counter while pressed. This works well for single-key input but does not support N-key rollover — pressing a second alpha key while one is already down can mis-track the shared ALPHA assertion. Use it for software that expects TiEmu-style timing, or if you prefer to press 2ND/♦/ALPHA by hand exactly like on the calculator.

#### Key mapping

The table below applies to both modes unless noted. In Natural mode the *character* you type is what matters; in Mapped mode the *physical PC key* maps to the listed TI key.

**Modifiers, control and system keys**

| PC key | TI-89 key |
|---|---|
| Caps Lock | ALPHA (hold like on the calculator) |
| Left/Right Ctrl | ♦ (Diamond) |
| Left/Right Alt | 2ND |
| Left/Right Shift | SHIFT (Mapped mode) / logical shift (Natural mode) |
| Windows/GUI key | SHIFT (Natural mode) |
| F1–F5 | F1–F5 |
| F6 / End | CATALOG |
| F7 / Home | HOME |
| F8 | MODE |
| Page Up | APPS |
| Page Down | EE |
| Arrow keys | Cursor pad |
| Insert | ON (power) |
| Delete | CLEAR |
| Backspace | BACKSPACE (⌫) |
| Enter | ENTER |
| ESC | ESC |
| Tab | STO→ |

**Typing keys**

| PC key | TI-89 key |
|---|---|
| 0–9 (top row) | 0–9 |
| T / X / Y / Z | T / X / Y / Z (dedicated keys — no ALPHA needed) |
| A | ALPHA + = |
| B | ALPHA + ( |
| C | ALPHA + ) |
| D | ALPHA + , |
| E | ALPHA + ÷ |
| F | ALPHA + \| (pipe) |
| G | ALPHA + 7 |
| H | ALPHA + 8 |
| I | ALPHA + 9 |
| J | ALPHA + × |
| K | ALPHA + EE |
| L | ALPHA + 4 |
| M | ALPHA + 5 |
| N | ALPHA + 6 |
| O | ALPHA + − |
| P | ALPHA + STO→ |
| Q | ALPHA + 1 |
| R | ALPHA + 2 |
| S | ALPHA + 3 |
| U | ALPHA + + |
| V | ALPHA + 0 |
| W | ALPHA + . |
| `,` / `.` | , / . (Shift+`,` = `<`, Shift+`.` = `>` in Natural mode) |
| `/` | ÷ |
| `\` | \| (pipe) |
| `=` | = (Shift+= = + in Natural mode) |
| `` ` `` (backtick) | ^ (power) |
| `[` / `]` | ( / ) in Mapped mode; `[` / `]` via 2ND in Natural mode |
| `-` | − (minus) in Mapped mode; (−) negate in Natural mode (Shift+`-` = −) |
| `'` (apostrophe) | + (plus) in Mapped mode; 2ND sequence in Natural mode |
| Space | (−) negate in Mapped mode; space in Natural mode (Shift+Space = (−)) |
| `;` | 9 in Mapped mode; `:` (2ND+9) in Natural mode |
| Numpad 0–9 | 0–9 |
| Numpad `.` | . |
| Numpad `+` `−` `*` `/` | + − × ÷ |
| Numpad Enter | ENTER |

**Numpad and Num Lock.** The numpad tables above apply when the keyboard's Num Lock is on. Num Lock is tracked by the *host* via LED state, and MiSTer never sets it — so many USB keyboards stay in cursor mode permanently, and the numpad then sends navigation codes (arrows, Home/End, Page Up/Down, Delete, Insert) instead of digits, with `/` being the only key that works as expected. If that happens:

* Try pressing **Num Lock** — some keyboards toggle it themselves.
* Otherwise, set the OSD option **Numpad Mode → Digits**. The core then decodes those navigation codes as keypad digits 1–9 (the main arrow keys are also affected and will type digits). Numpad `0` remains the ON key and numpad `.` remains CLEAR in this mode, since those have no duplicate on the PC keyboard, while digits do (top number row).

Any PC key not listed here is currently unmapped and produces no keystroke.

## Building from source

**Toolchain:** Intel/Altera **Quartus Prime Lite 17.0.x** (the project was created with 17.0.0 Lite Edition; the MiSTer `sys/` framework targets that toolchain). Target device is the DE10-Nano's Cyclone V `5CSEBA6U23I7`.

```sh
git clone <this repository>
cd TI89_MiSTer
quartus_sh --flow compile TI89
```

The compiled bitstream is written to `output_files/TI89_<date>.rbf`. Copy it to your MiSTer SD card (e.g. `/media/fat/_Console/`) to run it. Build artifacts (`output_files/`, `db/`, …) are not tracked in git.

Note: the `references/` directory (third-party material consulted during development) is not part of the repository; see `references/README.md` locally and [Third-party code](#third-party-code--dependencies) below for where to obtain it.

## Repository layout

```
TI89.sv             Top-level "emu" module (MiSTer core entry point)
TI89.qpf/.qsf/.sdc  Quartus project, settings and timing constraints
files.qip           Source file list for the Quartus project
jtag.cdf            JTAG chain description for direct FPGA programming
TI89Titanium_OS.89u (NOT included — copyrighted TI OS image, git-ignored)
rtl/                Core RTL
  cpu_wrapper.sv      fx68k clock-enable generation, bus bridge, IRQ encoder
  mem_ctrl.sv         Address decoder, RAM/bus controller (TI-89 TM memory map)
  sdram.sv            SDRAM controller backing the 4 MB flash window
  rom_loader.sv       .89u OS image loader (hps_io ioctl → SDRAM)
  flash_ctrl.sv       Sharp WSM flash write state machine
  io_ports.sv         Memory-mapped I/O banks 1–3 ($600000/$700000/$710000)
  timer_int.sv        HW2+ auto-interrupt / timer logic
  keyboard.sv         10×8 keyboard matrix, PS/2 scancode mapping, Native Mode sequencer (FIFO + modifier injection)
  lcd_ctrl.sv         LCD DMA controller
  video_scaler.sv     LCD raster integer scaler for the MiSTer video path
  fx68k/              FX68K 68000 CPU core (GPLv3, Jorge Cwik) — see LICENSE
  pll/                PLL IP instance (64 MHz from 50 MHz)
sys/                MiSTer system framework (hps_io, video, audio, OSD, …)
                    from the Minimig-AGA_MiSTer template (GPLv3)
output_files/       (NOT included) Quartus build outputs
references/         (NOT included) third-party reference material
```

## Third-party code & dependencies

Code actually built into the bitstream:

| Component | Location | Author | License |
|---|---|---|---|
| MiSTer system framework (`sys/`: hps_io, video mixer/scaler, OSD, audio, SDRAM bridge, …) | `sys/` | MiSTer-devel / Sorgelig, based on Minimig-AGA_MiSTer (Rok Krajnc, Dennis van Weeren et al.) | [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html) |
| FX68K — cycle-accurate Motorola 68000 core | `rtl/fx68k/` | Jorge Cwik | [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html) |
| MiSTer board build files (`sys/sys.tcl`, `sys/sys_analog.tcl`, build_id script) | `sys/` | MiSTer-devel | GPLv3 |

Build-time dependencies (not code dependencies, not included):

* **Quartus Prime Lite 17.0.x** — free toolchain from Intel/Altera, required to compile the project.

Runtime requirements:

* A **MiSTer** setup on a Terasic **DE10-Nano** board.
* A **TI-89 Titanium OS upgrade file** (`*.89u`), which you must obtain yourself; it is copyrighted by Texas Instruments and is not distributed with this project.

## References (not included in this repository)

The `references/` directory is excluded from git (see `.gitignore`). If you want the material that informed this implementation, fetch it from upstream:

| Reference | Upstream | License | Used for |
|---|---|---|---|
| Minimig-AGA_MiSTer | <https://github.com/MiSTer-devel/Minimig-AGA_MiSTer> | GPLv3 | Template for the MiSTer `sys/` framework and project structure |
| fx68k | <https://github.com/ijor/fx68k> | GPLv3 | Source of the 68000 CPU core vendored in `rtl/fx68k/` |
| TiEmu | <https://github.com/debrouxl/tiemu> | GPLv2+ | Hardware behavior reference: keyboard matrix, I/O ports, memory map |
| TI-89 JavaScript simulator (emu68k fork) | <https://tiplanet.org/emu68k_fork/> / Patrick Davidson's original | GPL | Timer/interrupt model, flash WSM word model |
| z80ti-fpga | <https://github.com/hellux/z80ti-fpga> | GPLv3 | FPGA calculator design reference |
| MacPlus-MiSTer | <https://github.com/MiSTer-devel/MacPlus_MiSTer> | GPLv3 | SDRAM design reference |
| MegaDrive-MiSTer | <https://github.com/MiSTer-devel/MegaDrive_MiSTer> | GPLv3 | SDRAM design reference |

None of this reference code is compiled into the bitstream; only the components in the table above are.

## License

Copyright © 2026 TI89_MiSTer contributors

The FX68K CPU core is Copyright © 2018 Jorge Cwik.
The MiSTer `sys/` framework is Copyright © its respective authors (see [Minimig-AGA_MiSTer](https://github.com/MiSTer-devel/Minimig-AGA_MiSTer) and [MiSTer-devel](https://github.com/MiSTer-devel)).

This program is free software: you can redistribute it and/or modify
it under the terms of the **GNU General Public License** as published by
the Free Software Foundation, either **version 3** of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program (see [LICENSE](LICENSE)). If not, see
<http://www.gnu.org/licenses/>.

The TI-89, TI-89 Titanium and related marks are trademarks of Texas Instruments.
This project is not affiliated with or endorsed by Texas Instruments.
