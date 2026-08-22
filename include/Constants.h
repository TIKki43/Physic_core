#pragma once

namespace Constants
{
inline constexpr double G{ 6.67430e-11 }; // m^3 * kg^-1 * s^-2
inline constexpr double c{ 299792458.0 }; // m/s

inline constexpr double DefaultDeltaTime{ 0.001f };
namespace AstroDynamicConstants
{
inline constexpr double SunJ2{ 2.000000000e-7 };
inline constexpr double MercuryJ2{ 5.031720235e-5 };
inline constexpr double VenusJ2{ 4.404435325e-6 };
inline constexpr double EarthJ2{ 1.082626680e-3 };
inline constexpr double MarsJ2{ 1.956600000e-3 };
inline constexpr double JupiterJ2{ 1.469650000e-2 };
inline constexpr double SaturnJ2{ 1.629060000e-2 };
inline constexpr double UranusJ2{ 3.510700000e-3 };
inline constexpr double NeptuneJ2{ 3.536300000e-3 };

inline constexpr double SunRefRadius{ 6.957000000e8 };
inline constexpr double MercuryRefRadius{ 2.440000000e6 };
inline constexpr double VenusRefRadius{ 6.051000000e6 };
inline constexpr double EarthRefRadius{ 6.378136300e6 };
inline constexpr double MarsRefRadius{ 3.396000000e6 };
inline constexpr double JupiterRefRadius{ 7.149200000e7 };
inline constexpr double SaturnRefRadius{ 6.033000000e7 };
inline constexpr double UranusRefRadius{ 2.555900000e7 };
inline constexpr double NeptuneRefRadius{ 2.476400000e7 };

}
}
