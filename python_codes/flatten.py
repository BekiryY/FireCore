"""
flatten.py
----------
Collects every *.v file that lives *directly* under the src/ folder
(non-recursive) and writes them all into a single flattened.txt file
at the project root.

Usage:
    python flatten.py
"""

import os
import glob

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
SRC_DIR      = os.path.join(SCRIPT_DIR, "src")
OUTPUT_FILE  = os.path.join(SCRIPT_DIR, "flattened.txt")

# ── Gather files ───────────────────────────────────────────────────────────────
pattern   = os.path.join(SRC_DIR, "*.v")          # direct children only
v_files   = sorted(glob.glob(pattern))            # sort for deterministic order

if not v_files:
    print(f"[warn] No *.v files found directly under: {SRC_DIR}")
else:
    print(f"[info] Found {len(v_files)} file(s):")
    for f in v_files:
        print(f"       {os.path.basename(f)}")

# ── Write output ───────────────────────────────────────────────────────────────
SEPARATOR = "=" * 80

with open(OUTPUT_FILE, "w", encoding="utf-8") as out:
    for path in v_files:
        filename = os.path.basename(path)

        # Header banner for each file
        out.write(f"{SEPARATOR}\n")
        out.write(f"// FILE: {filename}\n")
        out.write(f"{SEPARATOR}\n\n")

        with open(path, "r", encoding="utf-8", errors="replace") as src:
            out.write(src.read())

        out.write("\n\n")   # blank lines between files

print(f"[done] Written -> {OUTPUT_FILE}")