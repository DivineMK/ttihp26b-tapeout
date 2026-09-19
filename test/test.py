# SPDX-FileCopyrightText: © 2026 LKhanh
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, Edge, Timer


def _safe_int(val):
    """Convert a cocotb handle value to int, returning None if it contains X/Z.

    Gate-level netlists power up with X on every flop until reset propagates,
    so models must tolerate unknown values instead of raising ValueError.
    """
    try:
        return int(val)
    except ValueError:
        return None


class SpiRamEmuModel:
    """
    Simulates RP2040 spi-ram-emu (23LC512 protocol):
    - 8-bit command, 16-bit address, byte burst until CS rises
    - 0x03 read / 0x02 write only, same subset as firmware
    Pins on PMOD uio:
      uio[0] = CS
      uio[1] = MOSI (SD0)
      uio[2] = MISO (SD1)
      uio[3] = SCK
    """
    def __init__(self, dut, mem_size=65536):
        self.dut = dut
        self.mem = bytearray(mem_size)
        self.last_cs_high_cycle = 0
        self.txn_count = 0
        self.running = True

    def _uio(self):
        return _safe_int(self.dut.uio_out.value)

    def _sck(self):
        v = self._uio()
        return None if v is None else (v >> 3) & 1

    def _mosi(self):
        v = self._uio()
        return None if v is None else (v >> 1) & 1

    async def run(self):
        self.dut.uio_in.value = 0
        while self.running:
            # Wait for CS low (uio_out[0] == 0); tolerate X before reset
            while True:
                await FallingEdge(self.dut.clk)
                uio_out = self._uio()
                if uio_out is None:
                    continue
                if (uio_out & 1) == 0:
                    break

            # CS-high check for RP2040 PIO/DMA reset (~50 clocks)
            # - fail on gap under 40 cycles, short gap drops next command
            now_cycle = cocotb.utils.get_sim_time('ns') / 15.625
            if self.txn_count > 0:
                cs_high_duration = now_cycle - self.last_cs_high_cycle
                assert cs_high_duration >= 40, (
                    f"CS was held high for only {cs_high_duration} cycles between txns, "
                    f"violating the spi-ram-emu RP2040 PIO rearm constraint (minimum 40 cycles required)!"
                )

            # Transaction started
            self.txn_count += 1
            cmd = 0
            # Read 8 command bits on SCK rising edge
            for _ in range(8):
                await self._wait_sck_edge(rising=True)
                mosi = self._mosi()
                assert mosi is not None, "MOSI is X during active SPI transaction"
                cmd = (cmd << 1) | mosi

            # Read 16 address bits on SCK rising edge
            addr = 0
            for _ in range(16):
                await self._wait_sck_edge(rising=True)
                mosi = self._mosi()
                assert mosi is not None, "MOSI is X during active SPI transaction"
                addr = (addr << 1) | mosi

            if cmd == 0x03:
                # READ command: stream bytes out on MISO
                while True:
                    data_byte = self.mem[addr]
                    addr = (addr + 1) & 0xFFFF
                    stopped = False
                    for b in range(8):
                        bit = (data_byte >> (7 - b)) & 1
                        # Drive MISO onto uio_in[2]
                        self.dut.uio_in.value = (bit << 2)
                        # Wait for SCK rising edge where master samples
                        edge = await self._wait_sck_edge_or_cs_high()
                        if edge == "cs_high":
                            stopped = True
                            break
                    if stopped:
                        break
            elif cmd == 0x02:
                # WRITE command: stream bytes in from MOSI
                while True:
                    data_byte = 0
                    stopped = False
                    for _ in range(8):
                        edge = await self._wait_sck_edge_or_cs_high()
                        if edge == "cs_high":
                            stopped = True
                            break
                        mosi = self._mosi()
                        assert mosi is not None, "MOSI is X during active SPI transaction"
                        data_byte = (data_byte << 1) | mosi
                    if stopped:
                        break
                    self.mem[addr] = data_byte
                    addr = (addr + 1) & 0xFFFF

            # Clear MISO and record time CS went high
            self.dut.uio_in.value = 0
            self.last_cs_high_cycle = cocotb.utils.get_sim_time('ns') / 15.625

    async def _wait_sck_edge(self, rising=True):
        last_sck = self._sck()
        while True:
            await FallingEdge(self.dut.clk)
            sck = self._sck()
            if sck is None:
                continue
            if last_sck is None:
                last_sck = sck
                continue
            if rising and last_sck == 0 and sck == 1:
                return
            elif (not rising) and last_sck == 1 and sck == 0:
                return
            last_sck = sck

    async def _wait_sck_edge_or_cs_high(self):
        last_sck = self._sck()
        while True:
            await FallingEdge(self.dut.clk)
            uio_out = self._uio()
            if uio_out is None:
                continue
            if (uio_out & 1) == 1:
                return "cs_high"
            sck = (uio_out >> 3) & 1
            if last_sck is None:
                last_sck = sck
                continue
            if last_sck == 0 and sck == 1:
                return "sck_rising"
            last_sck = sck


class QspiPmodModel:
    """
    Simulates Mole99 QSPI PMOD (W25Q128 Flash + APS6404 PSRAM).
      uio[0] = Flash CS
      uio[1] = SD0 / MOSI
      uio[2] = SD1 / MISO
      uio[3] = SCK
      uio[4] = SD2
      uio[5] = SD3
      uio[6] = RAM A CS
      uio[7] = RAM B CS
    """
    def __init__(self, dut):
        self.dut = dut
        self.flash_mem = bytearray(16 * 1024 * 1024)
        self.ram_mem = bytearray(8 * 1024 * 1024)
        self.running = True

    def _pack_uio(self, nibble):
        # Places nibble onto SD0..SD3: uio[1], uio[2], uio[4], uio[5]
        b0 = (nibble >> 0) & 1
        b1 = (nibble >> 1) & 1
        b2 = (nibble >> 2) & 1
        b3 = (nibble >> 3) & 1
        return (b0 << 1) | (b1 << 2) | (b2 << 4) | (b3 << 5)

    def _unpack_uio(self, val):
        b0 = (val >> 1) & 1
        b1 = (val >> 2) & 1
        b2 = (val >> 4) & 1
        b3 = (val >> 5) & 1
        return (b3 << 3) | (b2 << 2) | (b1 << 1) | b0

    def _uio(self):
        return _safe_int(self.dut.uio_out.value)

    def _sck(self):
        v = self._uio()
        return None if v is None else (v >> 3) & 1

    def _nib(self):
        """Read the current QSPI output nibble; must be driven (post-reset)."""
        v = self._uio()
        assert v is not None, "QSPI bus is X during active transaction"
        return self._unpack_uio(v)

    async def run(self):
        self.dut.uio_in.value = 0
        while self.running:
            # Wait for either Flash CS (uio[0] == 0) or RAM A CS (uio[6] == 0)
            # Tolerate X before reset propagates in gate-level sims.
            while True:
                await FallingEdge(self.dut.clk)
                uio_out = self._uio()
                if uio_out is None:
                    continue
                flash_cs = uio_out & 1
                ram_cs = (uio_out >> 6) & 1
                if flash_cs == 0 or ram_cs == 0:
                    break

            if flash_cs == 0:
                # Flash Continuous Read
                addr = 0
                for _ in range(6):
                    await self._wait_sck_rising()
                    addr = (addr << 4) | self._nib()
                # 2 mode nibbles (0xA0)
                for _ in range(2):
                    await self._wait_sck_rising()
                # 4 dummy nibbles
                for _ in range(4):
                    await self._wait_sck_rising()

                # Stream data nibbles
                while True:
                    data_byte = self.flash_mem[addr & 0xFFFFFF]
                    addr += 1
                    # Low nibble then high nibble or MSB first? In QSPI: high nibble then low nibble
                    nibs = [(data_byte >> 4) & 0xF, data_byte & 0xF]
                    stopped = False
                    for nib in nibs:
                        self.dut.uio_in.value = self._pack_uio(nib)
                        edge = await self._wait_sck_or_cs0_high()
                        if edge == "cs_high":
                            stopped = True
                            break
                    if stopped:
                        break

            elif ram_cs == 0:
                # RAM: Fast Read (0x0B) or Write (0x02)
                cmd = 0
                for _ in range(2):
                    await self._wait_sck_rising()
                    cmd = (cmd << 4) | self._nib()

                addr = 0
                for _ in range(6):
                    await self._wait_sck_rising()
                    addr = (addr << 4) | self._nib()

                if cmd == 0x0B:
                    # 4 dummy nibbles
                    for _ in range(4):
                        await self._wait_sck_rising()
                    while True:
                        data_byte = self.ram_mem[addr & 0x7FFFFF]
                        addr += 1
                        nibs = [(data_byte >> 4) & 0xF, data_byte & 0xF]
                        stopped = False
                        for nib in nibs:
                            self.dut.uio_in.value = self._pack_uio(nib)
                            edge = await self._wait_sck_or_cs1_high()
                            if edge == "cs_high":
                                stopped = True
                                break
                        if stopped:
                            break
                elif cmd == 0x02:
                    # Write data
                    while True:
                        stopped = False
                        edge = await self._wait_sck_or_cs1_high()
                        if edge == "cs_high":
                            break
                        hi = self._nib()
                        edge = await self._wait_sck_or_cs1_high()
                        if edge == "cs_high":
                            break
                        lo = self._nib()
                        self.ram_mem[addr & 0x7FFFFF] = (hi << 4) | lo
                        addr += 1

            self.dut.uio_in.value = 0

    async def _wait_sck_rising(self):
        last_sck = self._sck()
        while True:
            await FallingEdge(self.dut.clk)
            sck = self._sck()
            if sck is None:
                continue
            if last_sck is None:
                last_sck = sck
                continue
            if last_sck == 0 and sck == 1:
                return
            last_sck = sck

    async def _wait_sck_or_cs0_high(self):
        last_sck = self._sck()
        while True:
            await FallingEdge(self.dut.clk)
            uio_out = self._uio()
            if uio_out is None:
                continue
            if (uio_out & 1) == 1:
                return "cs_high"
            sck = (uio_out >> 3) & 1
            if last_sck is None:
                last_sck = sck
                continue
            if last_sck == 0 and sck == 1:
                return "sck_rising"
            last_sck = sck

    async def _wait_sck_or_cs1_high(self):
        last_sck = self._sck()
        while True:
            await FallingEdge(self.dut.clk)
            uio_out = self._uio()
            if uio_out is None:
                continue
            if ((uio_out >> 6) & 1) == 1:
                return "cs_high"
            sck = (uio_out >> 3) & 1
            if last_sck is None:
                last_sck = sck
                continue
            if last_sck == 0 and sck == 1:
                return "sck_rising"
            last_sck = sck


# Pack 32-bit RISC-V instruction in little-endian for byte-addressed model
def write_insn(mem, addr, insn):
    mem[addr + 0] = insn & 0xFF
    mem[addr + 1] = (insn >> 8) & 0xFF
    mem[addr + 2] = (insn >> 16) & 0xFF
    mem[addr + 3] = (insn >> 24) & 0xFF


@cocotb.test()
async def test_spi_ram_emu_mode(dut):
    """
    Test TinyQV boot with strap=0 (Single-SPI mode for RP2040 spi-ram-emu):
    - write known byte via gp, read back, copy to GPIO, spin
    - pass on GPIO 0x42 (fetch + store + load path)
    Program executed:
      1. addi x1, x0, 0x42      (x1 = 0x42)
      2. sw   x1, 0(x3)         (Store x1 to RAM at gp = 0x01000400 -> mapped to 0x8400 in SPI RAM)
      3. lw   x2, 0(x3)         (Load from RAM at gp into x2)
      4. sw   x2, 0(x4)         (Store x2 to GPIO out at tp = 0x80000000 -> uo_out[7:2] should show 0x10)
      5. jal  x0, 0             (Infinite loop)
    """
    dut._log.info("Starting test_spi_ram_emu_mode (strap=0)")

    # 64 MHz system clock: 15625 ps period split 7813 ps high / 7812 ps low
    # (15.625 ns = 15625 ps is odd, so cocotb requires explicit period_high)
    clock = Clock(dut.clk, 15625, unit="ps", period_high=7813)
    cocotb.start_soon(clock.start())

    spi_emu = SpiRamEmuModel(dut)

    # 1. addi x1, x0, 0x42
    write_insn(spi_emu.mem, 0x0000, 0x04200093)
    # 2. sw x1, 0(x3) (gp)
    write_insn(spi_emu.mem, 0x0004, 0x0011A023)
    # 3. lw x2, 0(x3) (gp)
    write_insn(spi_emu.mem, 0x0008, 0x0001A103)
    # 4. sw x2, 0(x4) (tp)
    write_insn(spi_emu.mem, 0x000C, 0x00222023)
    # 5. jal x0, 0
    write_insn(spi_emu.mem, 0x0010, 0x0000006F)

    cocotb.start_soon(spi_emu.run())

    # Set strap_mode = 0 on ui_in[0]
    dut.ena.value = 1
    dut.ui_in.value = 0x00  # ui_in[0] = 0 (SPI RAM emu mode)
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Verify strap_mode output pin (uo_out[1]) is 0
    assert (int(dut.uo_out.value) >> 1) & 1 == 0, "strap_mode output should be 0"

    # Wait for CPU to execute instructions
    # Each instruction fetch and data transfer takes ~150-300 cycles in SPI mode
    dut._log.info("Waiting for CPU execution...")

    success = False
    for cycle in range(5000):
        await ClockCycles(dut.clk, 1)
        uo = int(dut.uo_out.value)
        # Check if uo_out[7:2] == (0x42 >> 2) = 0x10 and RAM at 0x8400 has 0x42
        if ((uo >> 2) & 0x3F) == (0x42 >> 2):
            dut._log.info(f"Success! GPIO output observed 0x42 at cycle {cycle}")
            success = True
            break

    assert success, f"Did not observe expected GPIO output 0x42, uo_out = {hex(int(dut.uo_out.value))}"
    assert spi_emu.mem[0x8400] == 0x42, f"RAM at 0x8400 was not written with 0x42: got {hex(spi_emu.mem[0x8400])}"
    spi_emu.running = False
    dut._log.info("test_spi_ram_emu_mode passed successfully!")


@cocotb.test()
async def test_qspi_pmod_mode(dut):
    """
    Test TinyQV boot with strap=1 (QSPI PMOD mode):
    - flash fetch plus peripheral store via qspi_ctrl
    - pass on GPIO 0x55
    Verifies that when strap_mode=1 is sampled, the system switches to qspi_ctrl.
    """
    dut._log.info("Starting test_qspi_pmod_mode (strap=1)")

    # 64 MHz system clock: 15625 ps period split 7813 ps high / 7812 ps low
    # (15.625 ns = 15625 ps is odd, so cocotb requires explicit period_high)
    clock = Clock(dut.clk, 15625, unit="ps", period_high=7813)
    cocotb.start_soon(clock.start())

    qspi = QspiPmodModel(dut)

    # 1. addi x1, x0, 0x55
    write_insn(qspi.flash_mem, 0x0000, 0x05500093)
    # 2. sw x1, 0(x4) (tp - GPIO out)
    write_insn(qspi.flash_mem, 0x0004, 0x00122023)
    # 3. jal x0, 0
    write_insn(qspi.flash_mem, 0x0008, 0x0000006F)

    cocotb.start_soon(qspi.run())

    # Set strap_mode = 1 on ui_in[0]
    dut.ena.value = 1
    dut.ui_in.value = 0x01  # ui_in[0] = 1 (QSPI PMOD mode)
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Verify strap_mode output pin (uo_out[1]) is 1
    assert (int(dut.uo_out.value) >> 1) & 1 == 1, "strap_mode output should be 1"

    dut._log.info("Waiting for QSPI execution...")

    success = False
    for cycle in range(2000):
        await ClockCycles(dut.clk, 1)
        uo = int(dut.uo_out.value)
        if ((uo >> 2) & 0x3F) == (0x55 >> 2):
            dut._log.info(f"Success! GPIO output observed 0x55 at cycle {cycle}")
            success = True
            break

    assert success, f"Did not observe expected GPIO output 0x55, uo_out = {hex(int(dut.uo_out.value))}"
    qspi.running = False
    dut._log.info("test_qspi_pmod_mode passed successfully!")
