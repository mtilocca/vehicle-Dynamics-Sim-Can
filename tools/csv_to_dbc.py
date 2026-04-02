#!/usr/bin/env python3
"""
csv_to_dbc.py — Convert can_map.csv to a DBC (CAN Database) file.

The CSV is the human-readable source of truth.  Run this script whenever
the CSV is updated to regenerate the DBC file used by the C++ simulation.

Usage:
    python3 tools/csv_to_dbc.py                          # uses defaults
    python3 tools/csv_to_dbc.py config/can_map.csv config/can_map.dbc

J1939 addressing (SAE J1939 — 29-bit extended CAN identifiers):
    CSV columns: priority, pgn, sa, da
    29-bit ID  = (priority<<26) | (pf<<16) | (ps<<8) | sa
      PDU2 broadcast (PF >= 0xF0): ps = GE (group extension, part of PGN)
      PDU1 peer-to-peer (PF < 0xF0): ps = DA (destination address)
    DBC BO_ ID = 0x80000000 | 29bit_id  (Vector EFF convention)

DBC attribute extensions used by this project:
    BA_ "Direction"       BO_  <id> "rx"|"tx"   — frame direction
    BA_ "GenMsgCycleTime" BO_  <id> <ms>         — transmit period
    BA_ "Target"          SG_  <id> <sig> <name> — signal target group
    BA_ "DefaultValue"    SG_  <id> <sig> <val>  — physical default value

Note on bit numbering: start_bit follows LSB-0 convention (same as the
internal bitpack library) for both Intel and Motorola signals.  Standard
DBC Motorola MSB encoding is NOT used — keep this in mind if opening the
file with third-party CAN tools.
"""

import argparse
import csv
from collections import OrderedDict


# ── helpers ──────────────────────────────────────────────────────────────────

def _fmt(v: float) -> str:
    """Format a float for DBC without needless trailing zeros."""
    s = f"{v:g}"
    # Guarantee a decimal point so parsers treat it as floating-point.
    if "." not in s and "e" not in s.lower():
        s += ".0"
    return s


def _compute_j1939_dbc_id(priority: int, pgn: int, sa: int, da: int) -> int:
    """
    Compute the 29-bit J1939 CAN identifier and return the DBC BO_ ID
    (29-bit value OR'd with 0x80000000, the Vector EFF convention).

    PDU2 broadcast (PF >= 0xF0): PS carries the Group Extension (GE),
        which is the low byte of PGN.
    PDU1 peer-to-peer (PF < 0xF0): PS carries the Destination Address (DA).
    """
    dp  = (pgn >> 17) & 0x1          # data page (usually 0)
    pf  = (pgn >>  8) & 0xFF         # PDU format
    if pf >= 0xF0:                    # PDU2 — broadcast
        ge  = pgn & 0xFF
        can_id_29 = (priority << 26) | (dp << 24) | (pf << 16) | (ge << 8) | sa
    else:                             # PDU1 — peer-to-peer
        can_id_29 = (priority << 26) | (dp << 24) | (pf << 16) | (da << 8) | sa
    return 0x80000000 | can_id_29


# ── main conversion ───────────────────────────────────────────────────────────

def csv_to_dbc(csv_path: str, dbc_path: str) -> None:
    # OrderedDict: key = dbc_id (int), value = frame dict
    frames: "OrderedDict[int, dict]" = OrderedDict()

    with open(csv_path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            pgn_str = row.get("pgn", "").strip()
            if not pgn_str or pgn_str.startswith("#"):
                continue

            priority = int(row["priority"].strip())
            pgn      = int(pgn_str, 0)
            sa       = int(row["sa"].strip(), 0)
            da       = int(row["da"].strip(), 0)
            dbc_id   = _compute_j1939_dbc_id(priority, pgn, sa, da)

            if dbc_id not in frames:
                frames[dbc_id] = {
                    "name":      row["frame_name"].strip(),
                    "dlc":       int(row["dlc"]),
                    "cycle_ms":  int(row["cycle_ms"]) if row.get("cycle_ms") else 0,
                    "direction": row["direction"].strip().lower(),
                    "signals":   [],
                }

            frames[dbc_id]["signals"].append({
                "name":       row["signal_name"].strip(),
                "target":     row["target"].strip(),
                "start_bit":  int(row["start_bit"]),
                "bit_length": int(row["bit_length"]),
                # DBC byte-order token: '1' = Intel (little-endian), '0' = Motorola (big-endian)
                "byte_order": "1" if row["endianness"].strip().lower() == "little" else "0",
                # DBC value-type token: '+' = unsigned, '-' = signed
                "value_type": "-" if row["signed"].strip().upper() == "TRUE" else "+",
                "factor":     float(row["factor"]),
                "offset":     float(row["offset"]),
                "min":        float(row["min"]),
                "max":        float(row["max"]),
                "default":    float(row["default"]),
                "unit":       row["unit"].strip(),
                "comment":    row.get("comment", "").strip(),
            })

    # ── J1939 node names by source address ────────────────────────────────────
    _SA_TO_NODE = {
        0x21: "CabCtrl",
        0x28: "IMU",
        0x29: "GNSS",
        0x2A: "WheelSensor",
        0x2B: "BMS",
        0x2C: "Radar",
        0xF0: "SimPlant",
    }

    def _sender(dbc_id: int) -> str:
        sa = dbc_id & 0xFF  # low byte of 29-bit ID = source address
        return _SA_TO_NODE.get(sa, "Vector__XXX")

    # ── emit DBC ──────────────────────────────────────────────────────────────
    lines = [
        'VERSION ""',
        "",
        "NS_ :",
        "    NS_DESC_",
        "    CM_",
        "    BA_DEF_",
        "    BA_",
        "    VAL_",
        "",
        "BS_:",
        "",
        # J1939 ECU nodes — one entry per unique source address
        "BU_: " + " ".join(sorted(set(_SA_TO_NODE.values()))),
        "",
    ]

    # ── BO_ / SG_ blocks ─────────────────────────────────────────────────────
    for dbc_id, fr in frames.items():
        sender = _sender(dbc_id)
        lines.append(f'BO_ {dbc_id} {fr["name"]}: {fr["dlc"]} {sender}')
        for sig in fr["signals"]:
            bo = sig["byte_order"]
            vt = sig["value_type"]
            sb = sig["start_bit"]
            bl = sig["bit_length"]
            fa = _fmt(sig["factor"])
            of = _fmt(sig["offset"])
            mn = _fmt(sig["min"])
            mx = _fmt(sig["max"])
            un = sig["unit"]
            lines.append(
                f' SG_ {sig["name"]} : {sb}|{bl}@{bo}{vt}'
                f' ({fa},{of}) [{mn}|{mx}] "{un}" Vector__XXX'
            )
        lines.append("")

    # ── attribute definitions ─────────────────────────────────────────────────
    lines += [
        # J1939 bus-type declaration — makes tools show frames as "J1939PG"
        'BA_DEF_ "BusType" STRING ;',
        'BA_DEF_ BO_  "Direction"       STRING ;',
        'BA_DEF_ BO_  "GenMsgCycleTime" INT    0 10000 ;',
        'BA_DEF_ SG_  "Target"          STRING ;',
        'BA_DEF_ SG_  "DefaultValue"    FLOAT  -1e38 1e38 ;',
        "",
        'BA_DEF_DEF_  "BusType"         "J1939" ;',
        'BA_DEF_DEF_  "Direction"       "tx" ;',
        'BA_DEF_DEF_  "GenMsgCycleTime" 0 ;',
        'BA_DEF_DEF_  "Target"          "" ;',
        'BA_DEF_DEF_  "DefaultValue"    0.0 ;',
        "",
        'BA_ "BusType" "J1939" ;',
        "",
    ]

    # ── attribute values ──────────────────────────────────────────────────────
    for dbc_id, fr in frames.items():
        lines.append(f'BA_ "Direction"       BO_ {dbc_id} "{fr["direction"]}" ;')
        lines.append(f'BA_ "GenMsgCycleTime" BO_ {dbc_id} {fr["cycle_ms"]} ;')
        for sig in fr["signals"]:
            lines.append(
                f'BA_ "Target"       SG_ {dbc_id} {sig["name"]} "{sig["target"]}" ;'
            )
            lines.append(
                f'BA_ "DefaultValue" SG_ {dbc_id} {sig["name"]} {_fmt(sig["default"])} ;'
            )
    lines.append("")

    # ── signal comments ───────────────────────────────────────────────────────
    for dbc_id, fr in frames.items():
        for sig in fr["signals"]:
            if sig["comment"]:
                escaped = sig["comment"].replace("\\", "\\\\").replace('"', '\\"')
                lines.append(f'CM_ SG_ {dbc_id} {sig["name"]} "{escaped}" ;')
    lines.append("")

    with open(dbc_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))

    total_sigs = sum(len(fr["signals"]) for fr in frames.values())
    rx_count = sum(1 for fr in frames.values() if fr["direction"] == "rx")
    tx_count = sum(1 for fr in frames.values() if fr["direction"] == "tx")
    print(f"Generated {dbc_path}")
    print(f"  {len(frames)} frames ({rx_count} RX, {tx_count} TX), {total_sigs} signals")
    print(f"  Frame IDs (DBC): {', '.join(f'0x{k:08X}' for k in frames)}")


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert can_map.csv (J1939) to a DBC file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "csv",
        nargs="?",
        default="config/can_map.csv",
        help="Input CSV path (default: config/can_map.csv)",
    )
    parser.add_argument(
        "dbc",
        nargs="?",
        default="config/can_map.dbc",
        help="Output DBC path (default: config/can_map.dbc)",
    )
    args = parser.parse_args()
    csv_to_dbc(args.csv, args.dbc)


if __name__ == "__main__":
    main()
