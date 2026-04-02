# Define your 16 sets of coordinates here.
# Format: [Latitude, Longitude]
N = [51.133696570341655, 71.43163913937695]
S = [51.11026057362518, 71.42052168109727]
E = [51.12521564839819, 71.46731192666734]
W = [51.126580141666295, 71.40707493653436]
RITZ_CARLTON = [51.124765685067246, 71.43210200919789]
ABU_DHABI_1 = [51.12143227387381, 71.42878515268714]
ABU_DHABI_2 = [51.12172304520112, 71.42652738082138]
MEGA = [51.08672776951535, 71.41118884135531]
KHAN_SHATYR = [51.132599389420065, 71.40672935111475]
VOKZAL = [51.19034801714996, 71.4070625470186]
FOOTBALL_FIELD = [51.13220507, 71.37301862]
DEST = RITZ_CARLTON
N = VOKZAL
S = MEGA
W = KHAN_SHATYR

LOCATIONS = {
    "Set 1: N-N": {"start": N, "dest": DEST, "ref": N},
    "Set 2: N-E": {"start": N, "dest": DEST, "ref": E},
    "Set 3: N-S": {"start": N, "dest": DEST, "ref": S},
    "Set 4: N-W": {"start": N, "dest": DEST, "ref": W},
    "Set 5: E-E": {"start": E, "dest": DEST, "ref": E},
    "Set 6: E-S": {"start": E, "dest": DEST, "ref": S},
    "Set 7: E-W": {"start": E, "dest": DEST, "ref": W},
    "Set 8: E-N": {"start": E, "dest": DEST, "ref": N},
    "Set 9: S-S": {"start": S, "dest": DEST, "ref": S},
    "Set 10: S-W": {"start": S, "dest": DEST, "ref": W},
    "Set 11: S-N": {"start": S, "dest": DEST, "ref": N},
    "Set 12: S-E": {"start": S, "dest": DEST, "ref": E},
    "Set 13: W-W": {"start": W, "dest": DEST, "ref": W},
    "Set 14: W-N": {"start": W, "dest": DEST, "ref": N},
    "Set 15: W-E": {"start": W, "dest": DEST, "ref": E},
    "Set 16: W-S": {"start": W, "dest": DEST, "ref": S},
}
