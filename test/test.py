# test.py
#
# cocotb testbench for tt_um_rc5_72. Bit-bangs the SPI protocol directly
# via ui_in[2:0] (sclk/mosi/cs_n) and uo_out[1:0] (ready/miso) - cocotb's
# own Python code plays the SPI master role here directly, no separate
# spi_master.v/spi_relay_controller.v involved (those simulate a SECOND,
# physically separate board in this project's own FPGA-track integration
# test; this ASIC-track test only needs to exercise the core+wrapper
# itself, which only has ONE clock domain here since cocotb drives
# ui_in synchronously relative to dut.clk).
#
# Same trusted key/plaintext/ciphertext vectors used throughout this
# project's whole RTL simulation history (see project notes) - a
# no-match case, a CMC-only case, and a full-match case, matching all
# three response types the protocol defines.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# ---- pin mapping - matches tt_um_rc5_72.v's own assignments exactly ----
UI_SCLK = 0
UI_MOSI = 1
UI_CS_N = 2
UO_MISO = 0
UO_READY = 1

# dut.clk cycles per SPI half-bit-period - comfortably past the 2-cycle
# minimum spi_slave.v's own 2-stage synchronizers need to register a
# change, though with real margin since it costs little in simulation
# time (see this project's own history for why marginal SPI timing is
# worth avoiding rather than cutting close).
SCLK_HALF_CYCLES = 4


async def reset(dut):
    dut.rst_n.value = 0
    dut.ena.value = 1
    dut.ui_in.value = 0b100  # cs_n idle-high, sclk/mosi low
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


def set_ui_bit(dut, bit, value):
    cur = int(dut.ui_in.value)
    if value:
        cur |= (1 << bit)
    else:
        cur &= ~(1 << bit)
    dut.ui_in.value = cur


async def spi_send_byte(dut, data):
    # Mode 0, MSB-first - mosi changes while sclk is low, sampled by the
    # slave on the rising edge (matches spi_slave.v's own, already-
    # proven convention).
    for i in range(7, -1, -1):
        bit = (data >> i) & 1
        set_ui_bit(dut, UI_MOSI, bit)
        await ClockCycles(dut.clk, SCLK_HALF_CYCLES)
        set_ui_bit(dut, UI_SCLK, 1)
        await ClockCycles(dut.clk, SCLK_HALF_CYCLES)
        set_ui_bit(dut, UI_SCLK, 0)


async def spi_read_byte(dut):
    data = 0
    set_ui_bit(dut, UI_MOSI, 0)  # MOSI low for read
    # Dummy rising/falling edge to start the transaction
    await ClockCycles(dut.clk, SCLK_HALF_CYCLES)
    set_ui_bit(dut, UI_SCLK, 1)  # Rising edge (DUT samples MOSI, but we don't care)
    await ClockCycles(dut.clk, SCLK_HALF_CYCLES)
    set_ui_bit(dut, UI_SCLK, 0)  # Falling edge (DUT drives MISO for bit 7)
    for i in range(7, -1, -1):
        await ClockCycles(dut.clk, SCLK_HALF_CYCLES)
        set_ui_bit(dut, UI_SCLK, 1)  # Rising edge (sample MISO here)
        bit = (int(dut.uo_out.value) >> UO_MISO) & 1
        data = (data << 1) | bit
        await ClockCycles(dut.clk, SCLK_HALF_CYCLES)
        set_ui_bit(dut, UI_SCLK, 0)  # Falling edge (DUT drives MISO for next bit)
    return data


async def send_work_unit(dut, base_key, count, pt_a, pt_b, target_a, target_b):
    set_ui_bit(dut, UI_CS_N, 0)
    await spi_send_byte(dut, 0xAA)  # sync
    await spi_send_byte(dut, 0x01)  # cmd
    for i in range(9):
        await spi_send_byte(dut, (base_key >> (8 * i)) & 0xFF)
    for i in range(4):
        await spi_send_byte(dut, (count >> (8 * i)) & 0xFF)
    for i in range(4):
        await spi_send_byte(dut, (pt_a >> (8 * i)) & 0xFF)
    for i in range(4):
        await spi_send_byte(dut, (pt_b >> (8 * i)) & 0xFF)
    for i in range(4):
        await spi_send_byte(dut, (target_a >> (8 * i)) & 0xFF)
    for i in range(4):
        await spi_send_byte(dut, (target_b >> (8 * i)) & 0xFF)
    set_ui_bit(dut, UI_CS_N, 1)

    

async def wait_ready(dut, timeout_cycles=200000):
    # count=1 in every scenario below keeps this well within budget -
    # ~93 cycles for the compute core itself, plus request/response SPI
    # overhead - the large timeout is just a safety net against a
    # genuine hang, not an expected duration.
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if not (int(dut.uo_out.value) >> UO_READY) & 1:
            return
    assert False, "timed out waiting for ready"


async def recv_response(dut):
    await wait_ready(dut)
    set_ui_bit(dut, UI_CS_N, 0)
    sync = await spi_read_byte(dut)
    status = await spi_read_byte(dut)
    result = {"sync": sync, "status": status}
    if status == 0x01:
        key = 0
        for i in range(9):
            key |= (await spi_read_byte(dut)) << (8 * i)
        ct_a = 0
        for i in range(4):
            ct_a |= (await spi_read_byte(dut)) << (8 * i)
        ct_b = 0
        for i in range(4):
            ct_b |= (await spi_read_byte(dut)) << (8 * i)
        result["key"] = key
        result["ct_a"] = ct_a
        result["ct_b"] = ct_b
    elif status == 0x02:
        key = 0
        for i in range(9):
            key |= (await spi_read_byte(dut)) << (8 * i)
        cmc_count = 0
        for i in range(4):
            cmc_count |= (await spi_read_byte(dut)) << (8 * i)
        result["key"] = key
        result["cmc_count"] = cmc_count
    set_ui_bit(dut, UI_CS_N, 1)
    return result


# ---- trusted vectors, same as used throughout this project ----
CASE1_KEY, CASE1_PT_A, CASE1_PT_B, CASE1_TA, CASE1_TB = \
    0xCC55555555AAAAAAAA, 0x21436587, 0xA9CBED0F, 0x12345678, 0x9ABCDEF0
CASE2_KEY, CASE2_PT_A, CASE2_PT_B, CASE2_TA, CASE2_TB = \
    0xB6B843B603825D8BD0, 0x6CF2BD15, 0x989E1475, 0xBEFCAFE7, 0xA6EC745F
CASE3_KEY, CASE3_PT_A, CASE3_PT_B, CASE3_TA, CASE3_TB = \
    0x2A00047634BA6196CC, 0x0C13FB62, 0x7E370FCB, 0x4DA0AE1C, 0xD1C60CFB


@cocotb.test()
async def test_no_match(dut):
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    await reset(dut)
    await send_work_unit(dut, CASE1_KEY, 1, CASE1_PT_A, CASE1_PT_B, CASE1_TA, CASE1_TB)
    resp = await recv_response(dut)
    assert resp["sync"] == 0x55, f"sync={resp['sync']:#x}"
    assert resp["status"] == 0x00, f"status={resp['status']:#x} expected 0x00"


@cocotb.test()
async def test_full_match(dut):
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    await reset(dut)
    await send_work_unit(dut, CASE3_KEY, 1, CASE3_PT_A, CASE3_PT_B, CASE3_TA, CASE3_TB)
    resp = await recv_response(dut)
    assert resp["sync"] == 0x55, f"sync={resp['sync']:#x}"
    assert resp["status"] == 0x01, f"status={resp['status']:#x} expected 0x01"
    assert resp["key"] == CASE3_KEY, f"key={resp['key']:#x}"
    assert resp["ct_a"] == CASE3_TA and resp["ct_b"] == CASE3_TB, \
        f"ct=({resp['ct_a']:#x},{resp['ct_b']:#x})"


@cocotb.test()
async def test_cmc(dut):
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    await reset(dut)
    await send_work_unit(dut, CASE2_KEY, 1, CASE2_PT_A, CASE2_PT_B, CASE2_TA, CASE2_TB)
    resp = await recv_response(dut)
    assert resp["sync"] == 0x55, f"sync={resp['sync']:#x}"
    assert resp["status"] == 0x02, f"status={resp['status']:#x} expected 0x02"
    assert resp["key"] == CASE2_KEY, f"key={resp['key']:#x}"
    assert resp["cmc_count"] == 1, f"cmc_count={resp['cmc_count']}"
