KPL/MK

   Project-local Cassini meta-kernel  --  year 2005  --  CSPICE experiment
   =======================================================================

   NOTE ON PROVENANCE
   ------------------
   The downloaded Cassini archive (cosp_1000) does NOT ship the yearly
   meta-kernels described in extras/mk/mkinfo.txt (that directory contains
   only mkinfo.txt, no cas_YYYY_vVV.tm files). This file is therefore a
   fresh, project-local meta-kernel created following the conventions in
   mkinfo.txt. No original archived meta-kernel was modified.

   Per mkinfo.txt, PATH_VALUES has been set to the actual local location of
   the archive's 'data' directory (the archive default was './data').

   For computing geometric planet positions in the J2000 frame relative to
   the Solar System Barycenter, only two kernels are required:
     - the leapseconds kernel (LSK)  -> UTC to ET conversion
     - a spacecraft/planet/satellite ephemeris (SPK) covering 2005-01-01
   The SPK below (post-mission uniform reconstruction, 2018) covers
   2004-12-01 .. 2005-01-15, which includes 2005-01-01T00:00:00 UTC.

   NOTE: the additional_bsp files are referenced through the $EXTRA path
   symbol, NOT by full absolute path. A SPICE text-kernel string value is
   limited to 80 characters, and
     'C:/Users/.../CassiniData/additional_bsp/mar099s.bsp'
   is 81 characters, so it would be silently TRUNCATED (to '...mar099s.bs')
   and the kernel would fail to load. PATH_SYMBOLS avoids that limit.

\begindata

   PATH_VALUES  = ( 'C:/Users/tikki/Desktop/Projects/PhysicCore/CassiniData/cosp_1000/data'
                    'C:/Users/tikki/Desktop/Projects/PhysicCore/CassiniData/additional_bsp' )

   PATH_SYMBOLS = ( 'KERNELS'
                    'EXTRA' )

   KERNELS_TO_LOAD = (
                 '$KERNELS/lsk/naif0012.tls'
                 '$KERNELS/pck/pck00010.tpc'

                 '$EXTRA/asteroids_data/gm_Horizons.pck'
                 '$KERNELS/pck/gm_de440.tpc'

                 '$KERNELS/spk/180628RU_SCPSE_04336_05015.bsp'

                 '$EXTRA/mar099s.bsp'
                 '$EXTRA/jup365.bsp'
                 '$EXTRA/sat415.bsp'
                 '$EXTRA/sat441.bsp'
                 '$EXTRA/ura184_part-3.bsp'
                 '$EXTRA/nep097.bsp'
                 '$EXTRA/plu060.bsp'
                 '$EXTRA/de440s.bsp'

                 '$EXTRA/asteroids_data/20000001.bsp'
                 '$EXTRA/asteroids_data/20000002.bsp'
                 '$EXTRA/asteroids_data/20000003.bsp'
                 '$EXTRA/asteroids_data/20000004.bsp'
                 '$EXTRA/asteroids_data/20000007.bsp'
                 '$EXTRA/asteroids_data/20000010.bsp'
                 '$EXTRA/asteroids_data/20000015.bsp'
                 '$EXTRA/asteroids_data/20000016.bsp'
                 '$EXTRA/asteroids_data/20000031.bsp'
                 '$EXTRA/asteroids_data/20000052.bsp'
                 '$EXTRA/asteroids_data/20000065.bsp'
                 '$EXTRA/asteroids_data/20000087.bsp'
                 '$EXTRA/asteroids_data/20000088.bsp'
                 '$EXTRA/asteroids_data/20000107.bsp'
                 '$EXTRA/asteroids_data/20000511.bsp'
                 '$EXTRA/asteroids_data/20000704.bsp'
              )

\begintext

   End of meta-kernel.
