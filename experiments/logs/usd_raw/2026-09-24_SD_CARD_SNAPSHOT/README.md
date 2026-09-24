# SD card snapshot — before 2026-09-24 re-fly

**Purpose:** Full copy of uSD logs **as they sat on the cards** after the bad lab session, so
new flights are not mixed up with old `thesisNN` files on the cards.

| Subfolder | Source | Status |
|---|---|---|
| `THESIS2/` | `/media/georg/THESIS2` | **43 non-empty `thesis*` files** (~74 MB), verified with `cmp` |
| `THESIS1/` | `/media/georg/THESIS1` | **32 non-empty `thesis*` files** (~52 MB), verified with `cmp`; **card cleared** |

**Also on desk (renamed copies):** `../2026-09-24_THESIS1/`, `../2026-09-24_THESIS2/`.

## After snapshot — clear cards (optional but recommended)

Firmware log prefix is `thesis` (see `flying_drone_stack/tools/usd_thesis_config.txt`). Deleting
old `thesis*` on the card avoids “which day is thesis48?” confusion; the next log usually picks
the next free index (often back toward low numbers if the card is empty).

**Only delete on-card files after this snapshot folder verifies (`cmp`).**

```bash
# THESIS2 (example — card mounted at /media/georg/THESIS2)
SNAP=experiments/logs/usd_raw/2026-09-24_SD_CARD_SNAPSHOT/THESIS2
CARD=/media/georg/THESIS2
for f in "$CARD"/thesis*; do
  [ -s "$f" ] || continue
  cmp -s "$f" "$SNAP/$(basename "$f")" && rm -v "$f"
done
# remove 0-byte placeholders too if you want a clean card
rm -v "$CARD"/thesis* 2>/dev/null
```

## Archive THESIS1 when mounted

```bash
SNAP=experiments/logs/usd_raw/2026-09-24_SD_CARD_SNAPSHOT/THESIS1
CARD=/media/georg/THESIS1
mkdir -p "$SNAP"
for f in "$CARD"/thesis*; do
  [ -f "$f" ] && [ -s "$f" ] && cp -a "$f" "$SNAP/"
done
# then cmp + rm on card as above with CARD=/media/georg/THESIS1
```

## Re-fly checklist

1. Mocap stable in viewer → short hover → uSD file **non-zero** on **both** cards.
2. New logs → new dated folder e.g. `usd_raw/2026-09-25_SD_CARD_SNAPSHOT/` or `2026-09-25_THESIS2/`.
