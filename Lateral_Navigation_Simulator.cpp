#include <algorithm> 
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

constexpr double m_pi{3.141592653589793};
constexpr double MEAN_EARTH_RADIUS_MI{3958.8}; // statute miles
constexpr double FEET_PER_MILE{5280.0};

// ===================== ENUMS =====================

enum class FlightPathLnavTrack : std::uint8_t
{
    Waypoints,
    Airways,
    Holds,
    TrackToFix,
    DirectTo,
    RadiusToFix,
    InterceptLegs
};

std::string to_string(FlightPathLnavTrack lnavtrack)
{
    switch (lnavtrack)
    {
    case FlightPathLnavTrack::Waypoints:     return "Waypoints";
    case FlightPathLnavTrack::Airways:       return "Airways";
    case FlightPathLnavTrack::Holds:         return "Holds";
    case FlightPathLnavTrack::TrackToFix:    return "TrackToFix";
    case FlightPathLnavTrack::DirectTo:      return "DirectTo";
    case FlightPathLnavTrack::RadiusToFix:   return "RadiusToFix";
    case FlightPathLnavTrack::InterceptLegs: return "InterceptLegs";
    }
    return "UNKNOWN";
}

enum class LegType : std::uint8_t
{
    IF,   // Initial Fix
    TF,   // Track to Fix
    DF,   // Direct to Fix
    CF,   // Course to Fix
    RF,   // Radius to Fix (curved)
    AF,   // Arc-to-Fix (DME arc)
    HA,   // Holding Pattern—Altitude
    HF,   // Holding Pattern—Fix
    HM    // Holding Pattern—Manual termination
};

// ===================== TURN + XTK MATH =====================

struct TurnAndXtkComputer
{
    static constexpr double gravity_ft_per_sec2 = 32.174;

    // Bank angle for level turn: phi = atan(V^2 / (R * g))
    static double compute_bank_angle_rad(double true_air_speed_ft_per_sec,
                                         double turn_radius_ft) noexcept
    {
        if (true_air_speed_ft_per_sec <= 0.0 || turn_radius_ft <= 0.0)
            return 0.0;

        const double ratio =
            (true_air_speed_ft_per_sec * true_air_speed_ft_per_sec) /
            (turn_radius_ft * gravity_ft_per_sec2);

        return std::atan(ratio);
    }

    // Turn rate: psi_dot = g * tan(phi) / V
    static double compute_turn_rate_rad_per_sec(double true_air_speed_ft_per_sec,
                                                double bank_angle_rad) noexcept
    {
        if (true_air_speed_ft_per_sec <= 0.0)
            return 0.0;

        return (gravity_ft_per_sec2 * std::tan(bank_angle_rad)) /
               true_air_speed_ft_per_sec;
    }

    // Cross-track error: XTK = d * sin(bearing_error)
    static double compute_cross_track_error(double distance_from_previous_ft,
                                            double bearing_error_rad) noexcept
    {
        return distance_from_previous_ft * std::sin(bearing_error_rad);
    }
};

// ===================== LNAV COMMAND + CONTROLLER =====================

struct LNAVCommand
{
    double target_bank_angle_rad{};
};

class LNAVController
{
public:
    LNAVCommand update(double distance_from_previous_ft,
                       double desired_course_rad,
                       double actual_track_rad,
                       double ground_speed_ft_per_sec)
    {
        (void)ground_speed_ft_per_sec; // not used yet

        LNAVCommand cmd{};

        // Bearing error = desired course - actual track
        double bearing_error_rad = desired_course_rad - actual_track_rad;

        // Cross-track error (in the same units as distance_from_previous_ft)
        double xtk_ft = TurnAndXtkComputer::compute_cross_track_error(
            distance_from_previous_ft, bearing_error_rad);

        constexpr double xtk_gain = 0.0005;              // tuning parameter
        constexpr double max_bank = 25.0 * m_pi / 180.0; // 25 deg in rad

        // Simple proportional law: bank command proportional to XTK
        double bank_cmd = xtk_gain * xtk_ft;

        // Clamp to ± max_bank
        if (bank_cmd >  max_bank) bank_cmd =  max_bank;
        if (bank_cmd < -max_bank) bank_cmd = -max_bank;

        cmd.target_bank_angle_rad = bank_cmd;
        return cmd;
    }
};

// ===================== NAV DATA STRUCTS =====================

struct Waypoint
{
    std::string code;
    double latitude_deg{};
    double longitude_deg{};
    double altitude_ft{};
};

struct Leg
{
    LegType type{LegType::TF};
    Waypoint from;
    Waypoint to;
};

struct WaypointLocations
{
    std::vector<Waypoint> waypoints{
        {"BWI", 39.177540, -76.668526},
        {"MCO", 28.424618, -81.310753},
        {"DFW", 32.897480, -97.040443},
        {"IKA", 35.416110,  51.152220},
        {"ATL", 33.640411, -84.419853},
        {"HKG", 22.308046, 113.918480},
        {"LAX", 33.942791, -118.410042},
        {"HND", 35.553333, 139.781111},
        {"PEK", 40.080000, 116.585000},
        {"ICN", 37.462500, 126.439167},
    };

    // route from BWI → MCO → ... → ICN
    std::vector<std::size_t> route_indices{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
};

const Waypoint* find_waypoint(const WaypointLocations& fp, const std::string& code)
{
    for (const auto& ap : fp.waypoints)
    {
        if (ap.code == code)
            return &ap;
    }
    return nullptr;
}

// ===================== GEODESY HELPERS =====================

inline double degrees_to_radians(double degrees) noexcept
{
    return degrees * m_pi / 180.0;
}

inline double radians_to_degrees(double radians) noexcept
{
    return radians * 180.0 / m_pi;
}

double great_circle_distance_miles(const Waypoint& a, const Waypoint& b)
{
    const double lat1 = degrees_to_radians(a.latitude_deg);
    const double lon1 = degrees_to_radians(a.longitude_deg);
    const double lat2 = degrees_to_radians(b.latitude_deg);
    const double lon2 = degrees_to_radians(b.longitude_deg);

    const double delta_longitude = lon2 - lon1;

    double cos_sigma =
        std::sin(lat1) * std::sin(lat2) +
        std::cos(lat1) * std::cos(lat2) * std::cos(delta_longitude);

    cos_sigma = std::clamp(cos_sigma, -1.0, 1.0);

    const double sigma    = std::acos(cos_sigma);
    const double distance = MEAN_EARTH_RADIUS_MI * sigma;

    return distance;
}

double total_route_distance_miles(const WaypointLocations& fp)
{
    double total = 0.0;

    for (std::size_t i = 1; i < fp.route_indices.size(); ++i)
    {
        const auto from_idx = fp.route_indices[i - 1];
        const auto to_idx   = fp.route_indices[i];

        const Waypoint& from = fp.waypoints[from_idx];
        const Waypoint& to   = fp.waypoints[to_idx];

        total += great_circle_distance_miles(from, to);
    }

    return total;
}

double initial_course_deg(const Waypoint& a, const Waypoint& b)
{
    const double lat1 = degrees_to_radians(a.latitude_deg);
    const double lon1 = degrees_to_radians(a.longitude_deg);
    const double lat2 = degrees_to_radians(b.latitude_deg);
    const double lon2 = degrees_to_radians(b.longitude_deg);

    const double dlon = lon2 - lon1;

    const double y = std::sin(dlon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) -
                     std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing_rad = std::atan2(y, x);

    // Convert to degrees 0..360
    double bearing_deg = radians_to_degrees(bearing_rad);
    if (bearing_deg < 0)
        bearing_deg += 360.0;

    return bearing_deg;
}

// ===================== AIRCRAFT STATE =====================

struct AircraftState
{
    double latitude_deg{};
    double longitude_deg{};
    double altitude_ft{};

    double heading_rad{};              // track / heading (rad)
    double ground_speed_ft_per_sec{};  // ft/s

    double bank_angle_rad{};           // current bank
};

// ===================== LNAV DIAGNOSTIC LOGGER =====================

class LNAVDiagnosticLogger
{
public:
    explicit LNAVDiagnosticLogger(const std::string& filename)
        : m_file(filename)
    {
        if (!m_file)
        {
            std::cerr << "ERROR: Could not open diagnostic log file: "
                      << filename << '\n';
            return;
        }

        m_file << "step,"
               << "leg_index,"
               << "from_code,"
               << "to_code,"
               << "lat_deg,"
               << "lon_deg,"
               << "desired_course_deg,"
               << "actual_track_deg,"
               << "xtk_ft,"
               << "bank_cmd_deg,"
               << "dist_to_to_mi\n";
    }

    bool is_open() const noexcept { return static_cast<bool>(m_file); }

    void log(std::uint64_t step,
             std::size_t leg_index,
             const Waypoint& from,
             const Waypoint& to,
             const AircraftState& ac,
             double desired_course_deg,
             double xtk_ft,
             double bank_cmd_deg,
             double dist_to_to_mi)
    {
        if (!m_file) return;

        double actual_track_deg = radians_to_degrees(ac.heading_rad);

        m_file << step << ","
               << leg_index << ","
               << from.code << ","
               << to.code << ","
               << ac.latitude_deg << ","
               << ac.longitude_deg << ","
               << desired_course_deg << ","
               << actual_track_deg << ","
               << xtk_ft << ","
               << bank_cmd_deg << ","
               << dist_to_to_mi
               << "\n";
    }

private:
    std::ofstream m_file;
};

// ===================== AIRCRAFT UPDATE =====================

void update_aircraft(AircraftState& ac,
                     const Waypoint& from,
                     const Waypoint& to,
                     LNAVController& lnav,
                     double dt_seconds)
{
    // 1) Desired course for this leg (from -> to)
    double desired_course_deg = initial_course_deg(from, to);
    double desired_course_rad = degrees_to_radians(desired_course_deg);

    // 2) Distance from "from" waypoint to current aircraft position
    Waypoint ac_wp{"AC", ac.latitude_deg, ac.longitude_deg, ac.altitude_ft};
    double dist_mi = great_circle_distance_miles(from, ac_wp);
    double distance_from_previous_ft = dist_mi * FEET_PER_MILE;

    // 3) Call LNAV controller to get bank command
    double actual_track_rad = ac.heading_rad;

    LNAVCommand cmd = lnav.update(distance_from_previous_ft,
                                  desired_course_rad,
                                  actual_track_rad,
                                  ac.ground_speed_ft_per_sec);

    // 4) Update aircraft bank (simple: follow command directly)
    ac.bank_angle_rad = cmd.target_bank_angle_rad;

    // 5) Compute turn rate and update heading
    double turn_rate_rad_per_sec =
        TurnAndXtkComputer::compute_turn_rate_rad_per_sec(
            ac.ground_speed_ft_per_sec,
            ac.bank_angle_rad);

    ac.heading_rad += turn_rate_rad_per_sec * dt_seconds;

    // Normalize heading to 0..2π
    if (ac.heading_rad < 0)
        ac.heading_rad += 2.0 * m_pi;
    else if (ac.heading_rad >= 2.0 * m_pi)
        ac.heading_rad -= 2.0 * m_pi;

    // 6) Move aircraft forward along its heading (simple local flat-earth step)
    double ds_ft    = ac.ground_speed_ft_per_sec * dt_seconds;
    double north_ft = ds_ft * std::cos(ac.heading_rad);
    double east_ft  = ds_ft * std::sin(ac.heading_rad);

    double R_ft    = MEAN_EARTH_RADIUS_MI * FEET_PER_MILE;
    double lat_rad = degrees_to_radians(ac.latitude_deg);

    double dlat_rad = north_ft / R_ft;
    double dlon_rad = east_ft / (R_ft * std::cos(lat_rad));

    ac.latitude_deg  += radians_to_degrees(dlat_rad);
    ac.longitude_deg += radians_to_degrees(dlon_rad);

    // === CROSSWIND DRIFT (simple sideways push) ================== // <<< NEW
    double wind_cross_ft = 15.0; // 15 ft per second sideways (crosswind)     // <<< NEW
    ac.longitude_deg += radians_to_degrees(wind_cross_ft / R_ft);            // <<< NEW
}

// ===================== TEST HARNESS =====================

int main()
{
    WaypointLocations fp;

    // 1) Initialize aircraft NEAR the FIRST waypoint in the route,
    //    but slightly offset laterally to create initial XTK.
    std::size_t first_idx = fp.route_indices.front();
    const Waypoint& first_wp = fp.waypoints[first_idx];

    AircraftState ac{};
    ac.latitude_deg  = first_wp.latitude_deg + 0.03; // ~2 nm north (lateral offset)  // <<< NEW
    ac.longitude_deg = first_wp.longitude_deg;
    ac.altitude_ft   = 40000.0;
    ac.ground_speed_ft_per_sec = 450.0 * 1.68781; // 450 kt

    LNAVController lnav;
    LNAVDiagnosticLogger diag("lnav_diag.csv");

    double dt = 1.0; // 1 second per step

    // 2) Outer loop: over each leg in the route
    for (std::size_t leg = 1; leg < fp.route_indices.size(); ++leg)
    {
        std::size_t from_idx = fp.route_indices[leg - 1];
        std::size_t to_idx   = fp.route_indices[leg];

        const Waypoint& from = fp.waypoints[from_idx];
        const Waypoint& to   = fp.waypoints[to_idx];

        // Compute leg distance + initial course
        double leg_distance_mi = great_circle_distance_miles(from, to);
        double init_course_deg = initial_course_deg(from, to);

        std::cout << "LEG " << leg
                  << ": " << from.code << " -> " << to.code
                  << ", distance = " << leg_distance_mi << " mi, "
                  << "initial course = " << init_course_deg << " deg\n";

        // Align heading roughly toward new leg, but introduce a 12° heading error
        ac.heading_rad = degrees_to_radians(init_course_deg + 12.0);         // <<< NEW

        // 3) Inner loop: simulate this leg in time steps
        for (std::uint64_t step = 0; step < 20000; ++step)
        {
            update_aircraft(ac, from, to, lnav, dt);

            // Position of aircraft as a waypoint
            Waypoint ac_wp{"AC", ac.latitude_deg, ac.longitude_deg, ac.altitude_ft};

            // Distance from aircraft to TO waypoint (for leg-complete logic)
            double dist_to_to_mi = great_circle_distance_miles(ac_wp, to);

            // ==== XTK and bank command for logging ====

            // Distance from FROM waypoint to aircraft
            double dist_from_from_mi         = great_circle_distance_miles(from, ac_wp);
            double distance_from_previous_ft = dist_from_from_mi * FEET_PER_MILE;

            // Desired vs actual course
            double desired_course_rad = degrees_to_radians(init_course_deg);
            double actual_track_rad   = ac.heading_rad;
            double bearing_error_rad  = desired_course_rad - actual_track_rad;

            // Cross-track error (same formula as TurnAndXtkComputer)
            double xtk_ft = distance_from_previous_ft * std::sin(bearing_error_rad);

            // Proportional bank command (must match LNAVController’s xtk_gain)
            constexpr double xtk_gain = 0.0005;
            double bank_cmd_rad = xtk_gain * xtk_ft;
            double bank_cmd_deg = radians_to_degrees(bank_cmd_rad);

            // ==== PRINT BLOCK ====
            if (step % 10 == 0)
            {
                std::cout << "  t = " << step << " s, "
                          << "Lat = " << ac.latitude_deg
                          << ", Lon = " << ac.longitude_deg
                          << ", Heading = " << radians_to_degrees(ac.heading_rad)
                          << ", Dist to " << to.code << " (mi) = " << dist_to_to_mi
                          << ", XTK (ft) = " << xtk_ft
                          << ", BankCmd (deg) = " << bank_cmd_deg
                          << '\n';
            }

            // CSV logging
            if (diag.is_open())
            {
                diag.log(step,
                         leg,
                         from,
                         to,
                         ac,
                         init_course_deg,
                         xtk_ft,
                         bank_cmd_deg,
                         dist_to_to_mi);
            }

            // Leg-complete condition
            if (dist_to_to_mi < 5.0)
            {
                std::cout << "  LEG COMPLETE: " << from.code
                          << " -> " << to.code
                          << " at t = " << step << " s\n";
                break;
            }
        }
    }

    return 0;
}

