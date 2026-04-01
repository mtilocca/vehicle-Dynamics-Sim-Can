#!/usr/bin/env python3
"""
csv_to_dbc.py — Convert can_map.csv to a DBC (CAN Database) file.

The CSV is the human-readable source of truth.  Run this script whenever
the CSV is updated to regenerate the DBC file used by the C++ simulation.

Usage:
    python3 tools/csv_to_dbc.py                          # uses defaults
    python3 tools/csv_to_dbc.py config/can_map.csv config/can_map.dbc

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


# ── main conversion ───────────────────────────────────────────────────────────

def csv_to_dbc(csv_path: str, dbc_path: str) -> None:
    frames: "OrderedDict[int, dict]" = OrderedDict()

    with open(csv_path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            raw_id = row.get("frame_id", "").strip()
            if not raw_id or raw_id.startswith("#"):
                continue

            frame_id = int(raw_id, 0)  # handles 0x… and plain decimal

            if frame_id not in frames:
                frames[frame_id] = {
                    "name":      row["frame_name"].strip(),
                    "dlc":       int(row["dlc"]),
                    "cycle_ms":  int(row["cycle_ms"]) if row.get("cycle_ms") else 0,
                    "direction": row["direction"].strip().lower(),
                    "signals":   [],
                }

            frames[frame_id]["signals"].append({
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
        "BU_:",
        "",
    ]

    # ── BO_ / SG_ blocks ─────────────────────────────────────────────────────
    for fid, fr in frames.items():
        lines.append(f'BO_ {fid} {fr["name"]}: {fr["dlc"]} Vector__XXX')
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
        'BA_DEF_ BO_  "Direction"       STRING ;',
        'BA_DEF_ BO_  "GenMsgCycleTime" INT    0 10000 ;',
        'BA_DEF_ SG_  "Target"          STRING ;',
        'BA_DEF_ SG_  "DefaultValue"    FLOAT  -1e38 1e38 ;',
        "",
        'BA_DEF_DEF_  "Direction"       "tx" ;',
        'BA_DEF_DEF_  "GenMsgCycleTime" 0 ;',
        'BA_DEF_DEF_  "Target"          "" ;',
        'BA_DEF_DEF_  "DefaultValue"    0.0 ;',
        "",
    ]

    # ── attribute values ──────────────────────────────────────────────────────
    for fid, fr in frames.items():
        lines.append(f'BA_ "Direction"       BO_ {fid} "{fr["direction"]}" ;')
        lines.append(f'BA_ "GenMsgCycleTime" BO_ {fid} {fr["cycle_ms"]} ;')
        for sig in fr["signals"]:
            lines.append(
                f'BA_ "Target"       SG_ {fid} {sig["name"]} "{sig["target"]}" ;'
            )
            lines.append(
                f'BA_ "DefaultValue" SG_ {fid} {sig["name"]} {_fmt(sig["default"])} ;'
            )
    lines.append("")

    # ── signal comments ───────────────────────────────────────────────────────
    for fid, fr in frames.items():
        for sig in fr["signals"]:
            if sig["comment"]:
                escaped = sig["comment"].replace("\\", "\\\\").replace('"', '\\"')
                lines.append(f'CM_ SG_ {fid} {sig["name"]} "{escaped}" ;')
    lines.append("")

    with open(dbc_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))

    total_sigs = sum(len(fr["signals"]) for fr in frames.values())
    rx_count = sum(1 for fr in frames.values() if fr["direction"] == "rx")
    tx_count = sum(1 for fr in frames.values() if fr["direction"] == "tx")
    print(f"Generated {dbc_path}")
    print(f"  {len(frames)} frames ({rx_count} RX, {tx_count} TX), {total_sigs} signals")


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert can_map.csv to a DBC file.",
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
