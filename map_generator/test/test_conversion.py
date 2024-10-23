from mgrs import MGRS


class MGRSPrecision:
    _10_KILO_METER = 1
    _1_KILO_METER = 2
    _100_METER = 3
    _10_METER = 4
    _1_METER = 5
    _100_MILLI_METER = 6
    _10_MILLI_METER = 7
    _1_MILLI_METER = 8
    _100_MICRO_METER = 9


def main():
    lat = 52.434312
    lon = 16.939062

    m = MGRS()
    mgrs_code = m.toMGRS(lat, lon, MGRSPrecision._1_METER)

    print("MGRS Coordinates:")
    print(f"MGRS: {mgrs_code}")
    GZD_ID_size = 5
    mgrs_zone = mgrs_code[:GZD_ID_size]
    precision = MGRSPrecision._1_METER
    x = float(mgrs_code[GZD_ID_size : GZD_ID_size + precision]) * 10 ** (
        MGRSPrecision._1_METER - precision
    )
    y = float(
        mgrs_code[GZD_ID_size + precision : GZD_ID_size + 2 * precision]
    ) * 10 ** (MGRSPrecision._1_METER - precision)

    print(f"MGRS Zone: {mgrs_zone}")
    print(f"X: {x:.5f}")
    print(f"Y: {y:.5f}\n")

    lat_reversed, lon_reversed = m.toLatLon(mgrs_code)
    print("Reversed Coordinates from MGRS:")
    print(f"Latitude: {lat_reversed}")
    print(f"Longitude: {lon_reversed}")

    print(f"Original Latitude: {lat}")
    print(f"Original Longitude: {lon}")


if __name__ == "__main__":
    main()
