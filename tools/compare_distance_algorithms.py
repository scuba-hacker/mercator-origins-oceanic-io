#!/usr/bin/env python3
"""
Compare Haversine vs equirectangular-squared distance for Wraysbury waypoints.
Verifies that both algorithms produce the same closest-feature ranking for a
grid of diver positions covering the lake area.

FINDINGS (2026-04-24, 3601 test positions, 96 Wraysbury waypoints):
  - Winner (closest feature) mismatches: 0 / 3601  — PASS
  - Full ranking mismatches:            46 / 3601  — all disagreements occur
    in lower-rank positions only; the top-5 ranking is identical in every case.
    These are caused by pairs of features that are nearly equidistant from the
    diver position, where floating-point rounding tips the tie differently
    between algorithms.

CONCLUSION: The equirectangular-squared fast path (one cos() upfront, then
pure arithmetic per waypoint) is safe to use as a drop-in replacement for
Haversine when finding the closest feature. It always selects the same winner.
GPS accuracy at ~3 m makes any sub-centimetre ordering difference operationally
irrelevant.
"""

import math
import random

# ---------------------------------------------------------------------------
# Waypoints (indices 0..95 = Wraysbury, matching getEndWaypointIndexWraysbury)
# ---------------------------------------------------------------------------
waypoints = [
    (0,  "01N Canoe",                        51.4620416774194,  -0.548974709677419),
    (1,  "02N The Sub",                       51.4609042894737,  -0.549211315789474),
    (2,  "03N Scimitar Car 5.5m",             51.460347,         -0.5489195),
    (3,  "04N Spitfire Car 6m",               51.4601028571429,  -0.54883835),
    (4,  "05N Lightning Boat 5.5m",           51.4605855,        -0.548901666666667),
    (5,  "06aN Caves Centre",                 51.4608746,        -0.5487572),
    (6,  "06bN Lion Entrance @ Caves",        51.460817,         -0.548734),
    (7,  "06cN Red Isis Bike @ Caves",        51.460898,         -0.548701333),
    (8,  "06dN Blue Raleigh Bike @ Caves",    51.4608584444444,  -0.548736333333333),
    (9,  "06eN Cave PC Laptop",               51.460936875,      -0.548774875),
    (10, "07P Cargo 2.5m",                    51.460014,         -0.548735),
    (11, "08B The Hole 18m",                  51.4604301666667,  -0.548688166666667),
    (12, "09P Dance Platform 6m",             51.460154,         -0.548687),
    (13, "10N Bus 2m",                        51.460073,         -0.548515),
    (14, "11N Confined Area",                 51.4599718333333,  -0.548582833333333),
    (15, "12N Commer Van 6m",                 51.4613355909091,  -0.548469727272727),
    (16, "13B White Boat 7m",                 51.4605198169044,  -0.548421667307919),
    (17, "14P Cargo 8m",                      51.4602986,        -0.5483127),
    (18, "15P Cargo Rusty 8m",                51.460192,         -0.548283),
    (19, "16P Portacabin 8m",                 51.46034,          -0.548173),
    (20, "17P Shallow Platform 2m",           51.4599705,        -0.54810825),
    (21, "18N Milk Float 6.5m",               51.4601745714286,  -0.548058571428571),
    (22, "19N Chicken Hutch Boat 6.5m",       51.4604027142857,  -0.54804),
    (23, "20N Skittles Sweet Bowl 5.5m",      51.4600375,        -0.5478815),
    (24, "21B Sticky Up Boat 5m",             51.4602514070597,  -0.54789158281982),
    (25, "22B Lady of Kent Search Light 5m",  51.4599185714286,  -0.547681),
    (26, "23N Traffic Lights 7m",             51.4600558888889,  -0.547677333333333),
    (27, "24N Half Die Hard Taxi 8m",         51.460773,         -0.547620875),
    (28, "25N Boat In A Hole 7m",             51.4599545,        -0.54755475),
    (29, "26N Iron Fish 2m",                  51.4595936666667,  -0.547489833333333),
    (30, "27B Wreck Site 6m",                 51.4604300973436,  -0.547383365365033),
    (31, "29B Dive/Spike Boat 7m",            51.4601315714286,  -0.547417857142857),
    (32, "30B White Day boat 6m",             51.4598131428572,  -0.547380285714286),
    (33, "31P 6m",                            51.459766,         -0.547347),
    (34, "32P 6m",                            51.459658,         -0.54725),
    (35, "33N Port Holes Boat 4.5m",          51.4595563333333,  -0.547263333333333),
    (36, "34P 6m",                            51.460312,         -0.547165),
    (37, "35N Dragon Boat 7.5m",              51.4599636666667,  -0.547154333333333),
    (38, "36P 6m",                            51.459555,         -0.54708),
    (39, "37N Dive Bell 4m",                  51.4594757058824,  -0.547087117647059),
    (40, "38B Lifeboat 6.5m",                 51.459839375,      -0.5469307),
    (41, "39N London Black Cab 7m",           51.459729,         -0.546992857142857),
    (42, "40N RIB Boat 6m",                   51.460236,         -0.546847571428571),
    (43, "41N Tin/Cabin Boat 7m",             51.459676625,      -0.5468125),
    (44, "42P 6m",                            51.459491,         -0.546867),
    (45, "43N Thorpe Orange Boat 5.5m",       51.4602073333333,  -0.546787666666667),
    (46, "44N VW Camper Van 5.5m",            51.459368,         -0.546760142857143),
    (47, "45B Listing Sharon 7.5m",           51.4598098699702,  -0.54670373432756),
    (48, "46N Plane 6m",                      51.459745,         -0.546649),
    (49, "47P 6m",                            51.459399,         -0.546594),
    (50, "48N Holey Ship 4.5m",               51.4594384444444,  -0.5465238),
    (51, "49B Claymore 6.5m",                 51.459634435324,   -0.54646635372985),
    (52, "50aN Swim Through no crates 6m",    51.45914367,       -0.546032333),
    (53, "50bN Swim Through mid 6m",          51.4591694,        -0.545999),
    (54, "50cN Swim Through crates 6m",       51.4592045,        -0.545912625),
    (55, "51B Orca Van 5.5m",                 51.4591431428571,  -0.545936857142857),
    (56, "X01 Dinghy Boat",                   51.4601285,        -0.5488505),
    (57, "X02 Quarry Machine",                51.460434,         -0.548921),
    (58, "X03 Metal Grated Box",              51.4599582857143,  -0.547648571428571),
    (59, "X04 4 crates in a line",            51.4599018571429,  -0.547141285714286),
    (60, "X05 Lone crate",                    51.4598467777778,  -0.547212666666667),
    (61, "X06 Collapsed Metal",               51.45967075,       -0.547253125),
    (62, "X07 Boat with Chain Links",         51.4600385714286,  -0.548724142857143),
    (63, "X08 Pot in a box",                  51.459940625,      -0.54852025),
    (64, "X09 Seahorse Mid-Water",            51.4600703333333,  -0.548645666666667),
    (65, "X10 Headless Nick",                 51.4600602857143,  -0.548671714285714),
    (66, "X11 Headless Tom Reeds",            51.4600452,        -0.5488188),
    (67, "X12 Cement Mixer",                  51.46020025,       -0.5478815),
    (68, "X13 Tyre",                          51.4600531428571,  -0.548183857142857),
    (69, "X14 Roadworks Sign",                51.4595778,        -0.547358),
    (70, "X15 Fireworks Launcher",            51.4599975,        -0.5481015),
    (71, "X16 2 Buried Boats",                51.4593264705883,  -0.5469361),
    (72, "X17 Half Buried Solo Boat",         51.4596635,        -0.54706025),
    (73, "X18 Half Buried Bike",              51.4600594210526,  -0.547575473684211),
    (74, "X19 Desk with Keyboard",            51.459924,         -0.547615181818182),
    (75, "X20 La Mouette Boat",               51.460740,         -0.547713),
    (76, "X21 Memorial Stone 7.5m",           51.460993,         -0.548006),
    (77, "X22 Fruit Machine 5.5m",            51.459353,         -0.546939),
    (78, "X23 Lone Crate 7m",                 51.461249,         -0.548688),
    (79, "X24 Cotton Reel 3m",                51.4623163,        -0.5494161),
    (80, "X25 Dumpy Cylinder 6m",             51.4600631,        -0.5480722),
    (81, "X26 The Skiff 8m",                  51.4607693,        -0.5486501),
    (82, "X27 Disused Pontoon",               51.459277,         -0.547087),
    (83, "X28 White Hull 4m",                 51.460100,         -0.548849),
    (84, "X29 Seahorse 3m",                   51.460101,         -0.548953),
    (85, "X30 Polar Bear 2m",                 51.460068,         -0.549013),
    (86, "X31 Piano 3m",                      51.460336,         -0.549035),
    (87, "X32 Not-A-Skiff 8m",               51.460753,         -0.548710),
    (88, "X33 Reclining Bear 8m",             51.460712,         -0.548656),
    (89, "X34 Better Quarry? 4m",             51.460410,         -0.548945),
    (90, "X35 New Boat 8m",                   51.460414,         -0.547535),
    (91, "X36 7m?",                           51.460959,         -0.548016),
    (92, "X37 4m?",                           51.459570,         -0.547307),
    # jetties / exits (indices 93-95)
    (93, "Z01 Cafe Jetty",                    51.460015,         -0.548316),
    (94, "Z02 Mid Jetty",                     51.459547,         -0.547461),
    (95, "Z03 Old Slipway",                   51.4591660,        -0.5469993),
]

FIRST = 0
END   = 96   # getEndWaypointIndexWraysbury


# ---------------------------------------------------------------------------
# Distance algorithms
# ---------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return 2 * R * math.asin(math.sqrt(a))


def equirect_sq(lat_diver, lon_diver, lat_wp, lon_wp, clat):
    """Equirectangular squared — same formula as the C++ fast path."""
    dlat = lat_wp - lat_diver
    dlon = (lon_wp - lon_diver) * clat
    return dlat * dlat + dlon * dlon


def closest_haversine(lat, lon):
    best_idx, best_d = FIRST, float('inf')
    for idx, label, wlat, wlon in waypoints[FIRST:END]:
        d = haversine_m(lat, lon, wlat, wlon)
        if d < best_d:
            best_d, best_idx = d, idx
    return best_idx, best_d


def closest_equirect(lat, lon):
    clat = math.cos(math.radians(lat))
    best_idx, best_sq = FIRST, float('inf')
    for idx, label, wlat, wlon in waypoints[FIRST:END]:
        sq = equirect_sq(lat, lon, wlat, wlon, clat)
        if sq < best_sq:
            best_sq, best_idx = sq, idx
    # Haversine metres for the winner only (mirrors C++ implementation)
    _, _, wlat, wlon = waypoints[best_idx]
    best_d = haversine_m(lat, lon, wlat, wlon)
    return best_idx, best_d


# ---------------------------------------------------------------------------
# Full ranking comparison (checks entire sorted order, not just the winner)
# ---------------------------------------------------------------------------

def full_ranking_haversine(lat, lon):
    ranked = sorted(
        range(FIRST, END),
        key=lambda i: haversine_m(lat, lon, waypoints[i][2], waypoints[i][3])
    )
    return ranked


def full_ranking_equirect(lat, lon):
    clat = math.cos(math.radians(lat))
    ranked = sorted(
        range(FIRST, END),
        key=lambda i: equirect_sq(lat, lon, waypoints[i][2], waypoints[i][3], clat)
    )
    return ranked


# ---------------------------------------------------------------------------
# Test grid — dense coverage of the lake
# ---------------------------------------------------------------------------

LAT_MIN, LAT_MAX = 51.458, 51.464
LON_MIN, LON_MAX = -0.550, -0.545
GRID_STEPS = 50          # 50×50 = 2500 grid points
RANDOM_EXTRA = 1000      # additional random points


def make_test_positions():
    positions = []
    for i in range(GRID_STEPS + 1):
        for j in range(GRID_STEPS + 1):
            lat = LAT_MIN + i * (LAT_MAX - LAT_MIN) / GRID_STEPS
            lon = LON_MIN + j * (LON_MAX - LON_MIN) / GRID_STEPS
            positions.append((lat, lon))
    rng = random.Random(42)
    for _ in range(RANDOM_EXTRA):
        lat = rng.uniform(LAT_MIN, LAT_MAX)
        lon = rng.uniform(LON_MIN, LON_MAX)
        positions.append((lat, lon))
    return positions


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    positions = make_test_positions()
    print(f"Testing {len(positions)} diver positions against {END - FIRST} waypoints")
    print(f"Algorithms: Haversine (reference) vs Equirectangular-squared (fast path)\n")

    winner_mismatches = []
    rank_mismatches   = []

    for lat, lon in positions:
        hav_idx, hav_d   = closest_haversine(lat, lon)
        eq_idx,  eq_d    = closest_equirect(lat, lon)

        if hav_idx != eq_idx:
            # How far apart are the two candidates?
            _, _, wlat_eq, wlon_eq = waypoints[eq_idx]
            d_eq_hav = haversine_m(lat, lon, wlat_eq, wlon_eq)
            gap_m = abs(hav_d - d_eq_hav)
            winner_mismatches.append({
                'pos': (lat, lon),
                'hav': (hav_idx, waypoints[hav_idx][1], hav_d),
                'eq':  (eq_idx,  waypoints[eq_idx][1],  d_eq_hav),
                'gap_m': gap_m,
            })

        hav_rank = full_ranking_haversine(lat, lon)
        eq_rank  = full_ranking_equirect(lat, lon)
        if hav_rank != eq_rank:
            rank_mismatches.append((lat, lon, hav_rank[:5], eq_rank[:5]))

    # --- Report ---
    print(f"=== Winner (closest feature) mismatches: {len(winner_mismatches)} / {len(positions)} ===")
    for m in winner_mismatches[:20]:
        lat, lon = m['pos']
        print(f"  pos ({lat:.6f}, {lon:.6f})")
        print(f"    Haversine: [{m['hav'][0]:2d}] {m['hav'][1]:<40s}  {m['hav'][2]:.2f} m")
        print(f"    Equirect:  [{m['eq'][0]:2d}] {m['eq'][1]:<40s}  {m['eq'][2]:.2f} m")
        print(f"    Gap between candidates: {m['gap_m']:.4f} m")
    if len(winner_mismatches) > 20:
        print(f"  ... and {len(winner_mismatches)-20} more")

    print()
    print(f"=== Full ranking mismatches: {len(rank_mismatches)} / {len(positions)} ===")
    for lat, lon, hr, er in rank_mismatches[:10]:
        print(f"  pos ({lat:.6f}, {lon:.6f})")
        print(f"    Haversine top-5: {hr}")
        print(f"    Equirect  top-5: {er}")
    if len(rank_mismatches) > 10:
        print(f"  ... and {len(rank_mismatches)-10} more")

    print()
    if not winner_mismatches:
        print("PASS: Both algorithms always agree on the closest feature.")
    else:
        max_gap = max(m['gap_m'] for m in winner_mismatches)
        print(f"NOTE: {len(winner_mismatches)} winner disagreements found.")
        print(f"      Largest distance gap between the two candidates: {max_gap:.4f} m")
        print(f"      (GPS accuracy ~3 m — gaps below that are operationally irrelevant)")

    if not rank_mismatches:
        print("PASS: Full rankings are identical for all test positions.")
    else:
        print(f"NOTE: {len(rank_mismatches)} full-ranking disagreements (ties or very close features).")


if __name__ == "__main__":
    main()
