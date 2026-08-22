#include "Body.h"
#include "Constants.h"
#include "PreInitializer.h"
#include "SpiceUsr.h"
#include "Utils.h"
#include "Vec3.h"
#include "WorldPhysics.h"
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

static constexpr std::string_view META_KERNEL{
  "C:\\Users\\tikki\\Desktop\\Projects\\PhysicCore\\CassiniMetaKernels\\cas_"
  "2005_"
  "physicalcoords.tm"
};

static constexpr std::string_view INITIAL_EPOCH{ "2005-01-01T00:00:00Z" };

using Seconds = std::chrono::duration<double>;

static constexpr const Seconds PHYSICAL_STEP_SECONDS{
  900.0
}; // seconds per physic step
static constexpr const Seconds MAX_ELAPSED_SECONDS{
  1825.0 * 86400.0
}; // 5 years of simulation; loaded kernels cover 2005->2010
static constexpr const Seconds TRIAL_SAMPLE_SECONDS{ 3600.0 }; // per 1h

static constexpr std::string_view CSV_NAME_BARY{
  "./outputs/CassiniNasaVal/BarySystemValidation_900_test.csv"
};
static constexpr std::string_view CSV_NAME_PHYS{
  "./outputs/CassiniNasaVal/PhysSystemValidation_900_test.csv"
};

enum class WorldType
{
  Physical,
  Barycenter
};

template <typename T>
concept HasBodiesRange
    = std::ranges::range<decltype (std::declval<T> ().Bodies)>
      && requires (const std::ranges::range_value_t<
                   decltype (std::declval<T> ().Bodies)> &body) {
           { body.GetPosition () } -> std::same_as<Vec3>;
         };

void
WritePositionErrors (const double CurET, Seconds elapsed,
                     HasBodiesRange auto &System, std::ostream &ccsv)
{

  for (const auto &body : System.Bodies)
    {
      SpiceDouble state[6]{};
      SpiceDouble lt{};
      spkezr_c (body.GetBodyType ().c_str (), CurET, "J2000", "NONE",
                "SOLAR SYSTEM BARYCENTER", state, &lt);
      const Vec3 TrueCurPosition
          = Vec3 (state[0] * 1000.0, state[1] * 1000.0, state[2] * 1000.0);
      const Vec3 CurPosition = body.GetPosition ();
      const Vec3 Error = CurPosition - TrueCurPosition;
      const double ErrorKm = Error.Length () / 1000.0;
      ccsv << elapsed.count () << "," << body.GetBodyType () << "," << Error.x
           << "," << Error.y << "," << Error.z << "," << ErrorKm << "\n";
    }
}

static void
InitWorld (WorldPhysics &world, WorldType Type, double &initialET)
{
  if (Type == WorldType::Barycenter)
    {
      PreInitializer::Missions::Cassini::SetBarycenterSolarSystem (
          world, META_KERNEL.data (), INITIAL_EPOCH.data ());
    }
  else
    {
      PreInitializer::Missions::Cassini::SetPhysicalSolarSystem (
          world, META_KERNEL.data (), INITIAL_EPOCH.data ());
    }
  PreInitializer::Missions::Cassini::SetMajorAsteroids (
      world, META_KERNEL.data (), INITIAL_EPOCH.data ());

  SpiceChar returnAction[]{ "RETURN" };
  SpiceChar noErrorPrint[]{ "NONE" };
  erract_c ("SET", 0, returnAction);
  errprt_c ("SET", 0, noErrorPrint);
  furnsh_c (META_KERNEL.data ());
  str2et_c (INITIAL_EPOCH.data (),
            &initialET); // converting EPOCH to numeric time
}

int
main ()
{

  WorldPhysics PhysicalSolarSystemWorld;
  WorldPhysics BarycenterSystemsWorld;

  double initialET{}; // numeric EPOCH; Initial J2000 point
  InitWorld (PhysicalSolarSystemWorld, WorldType::Physical, initialET);
  InitWorld (BarycenterSystemsWorld, WorldType::Barycenter, initialET);

  std::ofstream PhyserrorCsv (CSV_NAME_PHYS.data ());
  PhyserrorCsv.precision (15);
  PhyserrorCsv
      << "ElapsedSeconds,ObjectName,ErrorXm,ErrorYm,ErrorZm,ErrorKm\n";

  std::ofstream BaryerrorCsv (CSV_NAME_BARY.data ());
  BaryerrorCsv.precision (15);
  BaryerrorCsv
      << "ElapsedSeconds,BarySystemName,ErrorXm,ErrorYm,ErrorZm,ErrorKm\n";

  Seconds elapsed{};
  Seconds lastTrailSample{};
  while (elapsed < MAX_ELAPSED_SECONDS)
    {
      Integrator::RungeKutta4Integration1PNJ2 (
          PhysicalSolarSystemWorld.Bodies,
          PhysicalSolarSystemWorld.GravityModel,
          PHYSICAL_STEP_SECONDS.count ());
      Integrator::RungeKutta4Integration1PNJ2 (
          BarycenterSystemsWorld.Bodies, BarycenterSystemsWorld.GravityModel,
          PHYSICAL_STEP_SECONDS.count ());
      elapsed += PHYSICAL_STEP_SECONDS;
      if (elapsed - lastTrailSample >= TRIAL_SAMPLE_SECONDS)
        {
          const double CurET = elapsed.count () + initialET;
          WritePositionErrors (CurET, elapsed, PhysicalSolarSystemWorld,
                               PhyserrorCsv);
          WritePositionErrors (CurET, elapsed, BarycenterSystemsWorld,
                               BaryerrorCsv);

          lastTrailSample = elapsed;
        }
    }
}
