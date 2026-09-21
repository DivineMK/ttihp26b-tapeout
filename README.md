![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# TinyQV SoC (Dual Memory Backend)

TinyQV RV32EC microcontroller with two memory backends
selected by a reset strap (`ui_in[0]`): Quad-SPI Flash/PSRAM via a QSPI
PMOD, or single-SPI RAM emulated on the RP2040 (`spi-ram-emu`, 23LC512
protocol). 64 MHz system clock, 3x2 tile.

- [Read the project datasheet](docs/info.md) — how it works, pinout,
  address map, external hardware, and post-tapeout bring-up tasks.

## Status

- RTL sim: 2/2 cocotb tests pass at 64 MHz (`cd test && make clean && make`)
- Functional gate-level sim: 2/2 pass on the hardened netlist (`GATES=yes make`)
- Hardened with LibreLane 3.0.5 (IHP SG13G2): LVS/DRC/antenna clean,
  +3.02 ns setup / +0.11 ns hold slack, no violations across 3 corners
- Timed SDF sim is experimental only (`GATES=yes SDF=1 make`) — icarus
  cannot fully parse the OpenSTA SDF, so STA remains the timing signoff

Built on upstream [TinyQV](https://github.com/TinyTapeout/ttsky25a-tinyQV)
by Michael Bell.
- **Reused**: CPU core (`cpu/core.v`, `cpu/cpu.v`,
  `cpu/decode.v`, `cpu/alu.v`, `cpu/register.v`, `cpu/counter.v`), the
  QSPI controller (`cpu/qspi_ctrl.v`), and the UART transmitter
  (`peri/uart/uart_tx.v`)
- **Modified**: `cpu/mem_ctrl.v` (instantiates both memory backends and
  multiplexes them on the strap), `tinyqv.v` (plumbs the strap through)
- **New**: `src/cpu/spi_mem_ctrl.v` (single-SPI 23LC512 master for
  `spi-ram-emu`: command/address framing, 16-bit address windowing,
  48-cycle CS cooldown), `src/project.v` (Tiny Tapeout top: strap
  sampling, PMOD pin mapping, GPIO/UART peripherals), and the testbench
  (`test/test.py` SPI-emu + QSPI bus models and boot tests,
  `test/tb_sdf.v` timed-sim variant)

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Set up your Verilog project

1. Add your Verilog files to the `src` folder.
2. Edit the [info.yaml](info.yaml) and update information about your project, paying special attention to the `source_files` and `top_module` properties. If you are upgrading an existing Tiny Tapeout project, check out our [online info.yaml migration tool](https://tinytapeout.github.io/tt-yaml-upgrade-tool/).
3. Edit [docs/info.md](docs/info.md) and add a description of your project.
4. Adapt the testbench to your design. See [test/README.md](test/README.md) for more information.

The GitHub action will automatically build the ASIC files using [LibreLane](https://www.zerotoasiccourse.com/terminology/librelane/).

## Enable GitHub actions to build the results page

- [Enabling GitHub Pages](https://tinytapeout.com/faq/#my-github-action-is-failing-on-the-pages-part)

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)

## What next?

- [Submit your design to the next shuttle](https://app.tinytapeout.com/).
- Edit [this README](README.md) and explain your design, how it works, and how to test it.
- Share your project on your social network of choice:
  - LinkedIn [#tinytapeout](https://www.linkedin.com/search/results/content/?keywords=%23tinytapeout) [@TinyTapeout](https://www.linkedin.com/company/100708654/)
  - Mastodon [#tinytapeout](https://chaos.social/tags/tinytapeout) [@matthewvenn](https://chaos.social/@matthewvenn)
  - X (formerly Twitter) [#tinytapeout](https://twitter.com/hashtag/tinytapeout) [@tinytapeout](https://twitter.com/tinytapeout)
  - Bluesky [@tinytapeout.com](https://bsky.app/profile/tinytapeout.com)
