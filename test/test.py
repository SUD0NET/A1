# SPDX-FileCopyrightText: © 2026 Adnan Saad
# SPDX-License-Identifier: Apache-2.0

"""
cocotb tests for tt_um_a1 (A1 MPU)
External hardware (e.g. address latches, 8 KB SRAM preloaded with nuc.hex, wait-state
injector, bus protocol checker, UART TX monitor) lives in tb.v; these tests drive the
chip from outside by typing at its UART like a terminal and checking what comes back.
"""
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

CLK_NS = 40 # 25 MHz
BAUD = 115_200
BIT_PS = round(1e12 / BAUD) # One UART bit in picoseconds
HERE = Path(__file__).resolve().parent


# Helpers
def to_int(sig, default=None):
    try:
        return int(sig.value)
    except ValueError:  # X / Z
        return default


def tx_count(dut) -> int:
    return to_int(dut.txn, 0)


def tx_since(dut, mark: int) -> bytes:
    """Bytes the chip has transmitted since transmit-count `mark` (max 255)."""
    n = tx_count(dut) - mark
    if n <= 0:
        return b""
    if n > 255:
        raise AssertionError(f"Transcript too long to read back: {n} bytes")
    raw = to_int(dut.txline, 0).to_bytes(256, "big")
    return raw[256 - n:]


async def wait_tx(dut, mark: int, pattern: bytes, timeout_us: int, fail_on: bytes | None = None) -> bytes:
    """Wait until `pattern` appears in what the chip sent since `mark`."""
    waited = 0
    while True:
        tail = tx_since(dut, mark)
        if pattern in tail:
            return tail
        if fail_on is not None and fail_on in tail:
            raise AssertionError(f"Chip answered {tail!r} while waiting for {pattern!r}")
        if waited >= timeout_us:
            raise AssertionError(f"Timeout ({timeout_us} us) waiting for {pattern!r}; chip sent {tail!r}")
        await Timer(50, "us")
        waited += 50


async def send_byte(dut, value: int, bit_ps: int = BIT_PS) -> None:
    """Send one 8N1 frame on chip's RXD pin."""
    bits = [0] + [(value >> i) & 1 for i in range(8)] + [1]
    for level in bits:
        dut.rxd.value = level
        await Timer(bit_ps, "ps")


async def send_text(dut, text: str, gap_bits: float = 3.0, bit_ps: int = BIT_PS) -> None:
    """Type `text`. gap_bits = extra idle bit-times after each frame (0 = line rate)."""
    for ch in text.encode():
        await send_byte(dut, ch, bit_ps)
        if gap_bits:
            await Timer(int(gap_bits * bit_ps), "ps")


def bus_snapshot(dut) -> tuple[int, int]:
    return (to_int(dut.viol, 0), to_int(dut.frame_err, 0))


def assert_bus_clean(dut, snap: tuple[int, int]) -> None:
    viol, ferr = bus_snapshot(dut)
    viol_kind = to_int(dut.viol_kind, 0)
    assert viol == snap[0], (
        f"Bus protocol violations: {viol - snap[0]} new (kind mask 0b{viol_kind:08b}; "
        "0: RD&WR, 1: strobe in address phase, 2: ALE&ALEH, 3: AD contention, "
        "4/5: addr not held, 6: X on strobes)"
    )
    assert ferr == snap[1], f"UART TX framing errors: {ferr - snap[1]}"


async def boot(dut, banner: bool = True, new_clock: bool = True) -> int:
    """Start clock once per test, reload a nuc.hex image, reset, and wait
    for banner. Pass new_clock=False when calling boot() again inside same test."""
    if new_clock:
        cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())

    dut.ena.value = 1
    dut.rxd.value = 1
    dut.gpi.value = 0
    dut.wait_en.value = 0
    dut.wait_max.value = 0
    dut.peek_addr.value = 0
    dut.reload.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 4)

    dut.reload.value = 1  # restore SRAM image so tests remain independent
    await ClockCycles(dut.clk, 2)
    dut.reload.value = 0
    await ClockCycles(dut.clk, 6)

    dut.rst_n.value = 1
    mark = tx_count(dut)
    if banner:  # nucleus prints CR LF when ready
        await wait_tx(dut, mark, b"\r\n", 2_000)
    return tx_count(dut)


async def forth(
    dut,
    line: str,
    expect: tuple[str, ...] | list[str] = (),
    timeout_us: int = 80_000,
    gap_bits: float = 3.0,
) -> bytes:
    """type one Forth line at prompt and return what the chip printed.
    `expect` strings must all appear in order before final ` ok`."""
    mark = tx_count(dut)
    await send_text(dut, line, gap_bits)
    await send_byte(dut, 0x0D)
    tail = await wait_tx(dut, mark, b"ok\r\n", timeout_us, fail_on=b"?")

    pos = len(line)  # skip echo of what we typed
    for token in expect:
        idx = tail.find(token.encode(), pos)
        assert idx >= 0, f"{line!r}: expected {token!r} after echo; chip said {tail!r}"
        pos = idx + len(token)

    dut._log.info("%-34s -> %r", line, tail.replace(b"\r\n", b"|"))
    return tail


async def read_sram(dut, addr: int) -> int:
    dut.peek_addr.value = addr
    await Timer(1, "ns")
    return to_int(dut.peek_data, 0)

# Tests
@cocotb.test()
async def test_reset_outputs(dut):
    """Outputs are idle and defined during and just after reset."""
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.ena.value = 1
    dut.rxd.value = 1
    dut.gpi.value = 0
    dut.wait_en.value = 0
    dut.reload.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 8)

    uo = to_int(dut.uo_out)
    assert uo is not None, "uo_out has X/Z during reset"
    assert (uo & 0b00001111) == 0b1100, f"ALE/ALEH must be low, RD#/WR# high in reset: {uo:08b}"
    assert ((uo >> 4) & 1) == 1, "TXD must idle high in reset"
    assert ((uo >> 5) & 0b111) == 0, "GPO must reset to 0"
    assert to_int(dut.uio_oe) == 0, "AD bus must be released in reset"

    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    assert to_int(dut.uo_out) is not None, "uo_out has X/Z after reset release"


@cocotb.test()
async def test_boot_banner(dut):
    """After reset, chip fetches nucleus over bus and prints CR LF."""
    snap = bus_snapshot(dut)
    await boot(dut)
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_fetch_order(dut):
    """First reads: low byte then high byte of word 0, then of call target."""
    snap = bus_snapshot(dut)
    await boot(dut, banner=False)
    await Timer(200, "us")

    trace = to_int(dut.rdtrace, 0)
    reads = [(trace >> (13 * i)) & 0x1FFF for i in range(8)]
    dut._log.info("first read addresses: %s", [hex(r) for r in reads])

    for i in range(0, 8, 2):  # Little-endian pairs: even then odd
        assert reads[i] % 2 == 0 and reads[i + 1] == reads[i] + 1, (
            f"Reads {i}/{i + 1} are not a (low, high) byte pair: {[hex(r) for r in reads]}"
        )
    assert reads[0] == 0, f"First fetch must be at address 0, got {reads[0]:#x}"

    with open(HERE / "nuc.hex", encoding="utf-8") as f:
        word0 = int(f.readline(), 16)

    if (word0 >> 13) in (0b000, 0b010):  # jump or call at reset vector
        target = (word0 & 0x1FFF) * 2
        assert reads[2] == target, f"Expected fetch at {target:#x} after reset vector, got {reads[2]:#x}"

    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_hello_emit(dut):
    """`65 emit` prints A: typed characters are echoed and interpreted."""
    snap = bus_snapshot(dut)
    await boot(dut)
    await forth(dut, "65 emit", ["A"])
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_arithmetic(dut):
    """Number parsing, + and .x (nuc prints 4 hex digits)."""
    snap = bus_snapshot(dut)
    await boot(dut)
    await forth(dut, "1 2 + .x 123 456 + .x", ["0003", "0243"])
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_logic_ops(dut):
    """xor / negative literals"""
    snap = bus_snapshot(dut)
    await boot(dut)
    await forth(dut, "-1 255 xor .x", ["FF00"])
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_colon_definition(dut):
    """Compile a word, then call it (writes to RAM, call/return, dup *)."""
    snap = bus_snapshot(dut)
    await boot(dut)
    await forth(dut, ": sq dup * ; 7 sq .x", ["0031"])
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_do_loop_and_return_stack(dut):
    """do ... loop (return stack) and >r r> inside a definition."""
    snap = bus_snapshot(dut)
    await boot(dut)
    await forth(dut, ": t5 5 0 do 1 + loop ; 10 t5 .x", ["000F"])
    await forth(dut, ": rt >r r> ; 5 rt .x", ["0005"])
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_gpo_port(dut):
    """io! to port 4 drives uo_out[7:5]; io@ reads it back."""
    snap = bus_snapshot(dut)
    await boot(dut)
    await forth(dut, "5 4 io!")
    assert ((to_int(dut.uo_out, 0) >> 5) & 0b111) == 0b101, "GPO should be 101"
    await forth(dut, "2 4 io!")
    assert ((to_int(dut.uo_out, 0) >> 5) & 0b111) == 0b010, "GPO should be 010"
    await forth(dut, "4 io@ .x", ["0002"])
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_gpi_port(dut):
    """Port 0x2000 returns {GPI[5:0], rx_valid, !tx_busy}: shift away the 2 status bits."""
    snap = bus_snapshot(dut)
    await boot(dut)
    dut.gpi.value = 0b101010
    await forth(dut, "8192 io@ 2/ 2/ .x", ["002A"])
    dut.gpi.value = 0b010101
    await forth(dut, "8192 io@ 2/ 2/ .x", ["0015"])
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_sram_store_fetch(dut):
    """! and @ on the external SRAM: 0x1234 at byte 0x1800, little-endian."""
    snap = bus_snapshot(dut)
    await boot(dut)
    await forth(dut, "4660 6144 ! 6144 @ .x", ["1234"])
    assert await read_sram(dut, 0x1800) == 0x34, "Low byte mismatch at 0x1800"
    assert await read_sram(dut, 0x1801) == 0x12, "High byte mismatch at 0x1801"
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_wait_states(dut):
    """Random READY stalls (0-3 clocks) on every bus cycle must not change any result."""
    snap = bus_snapshot(dut)
    await boot(dut)
    dut.wait_max.value = 3
    dut.wait_en.value = 1

    await forth(dut, "1 2 + .x", ["0003"])
    await forth(dut, ": sq dup * ; 7 sq .x 4660 6144 ! 6144 @ .x", ["0031", "1234"])

    dut.wait_en.value = 0
    dut.wait_max.value = 0
    assert to_int(dut.waits_inserted, 0) > 1000, "wait-state injector did not run"
    assert_bus_clean(dut, snap)


@cocotb.test()
async def test_uart_rx_baud_tolerance(dut):
    """Type two characters with host baud rate off by +-x% and check the echo."""
    snap = bus_snapshot(dut)
    cases = [
        (0, True),
        (2000, True),
        (-2000, True),
        (20000, True),
        (-10000, False),
        (-20000, False),
    ]
    hard_failures = []
    for n, (ppm, hard) in enumerate(cases):
        await boot(dut, new_clock=(n == 0))
        mark = tx_count(dut)
        bit_ps = round(BIT_PS * (1 + ppm / 1e6))
        await send_text(dut, "z9", gap_bits=3.0, bit_ps=bit_ps)
        await Timer(1_500, "us")
        echo = tx_since(dut, mark)
        ok = echo == b"z9"
        status = "ok" if ok else ("FAIL" if hard else "margin warning (informational)")
        dut._log.info("host baud %+6.2f%%: echo %r -> %s", ppm / 1e4, echo, status)
        if not ok and hard:
            hard_failures.append((ppm, echo))

    assert not hard_failures, f"Echo wrong at host baud error: {hard_failures}"
    assert_bus_clean(dut, snap)


@cocotb.test(expect_fail=True)
async def test_paste_at_line_rate_KNOWN_LIMITATION(dut):
    """KNOWN LIMITATION (expected to fail): UART has one-byte receive buffer, so
    a paste at full line rate overruns it while CPU is echoing. Remove `expect_fail=True`
    after adding RX FIFO (depth >= 4)."""
    await boot(dut)
    mark = tx_count(dut)
    text = "1 2 + 3 4 + * .x " * 3
    await send_text(dut, text, gap_bits=0)
    await Timer(8, "ms")  # Let the echo drain
    echo = tx_since(dut, mark)
    assert echo == text.encode(), f"Echo lost/corrupted bytes at line rate: {echo!r}"
