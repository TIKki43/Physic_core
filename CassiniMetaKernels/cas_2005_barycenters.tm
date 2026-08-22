KPL/MK

   Project-local Cassini meta-kernel  --  5-year barycenter demo
   =============================================================

   Created for CassiniBarycenterSystemDemo2D.cpp. cas_2005_local.tm is left
   untouched.

   Instead of chaining ~70 per-window Cassini SCPSE SPKs (which would be slow
   to load and unwieldy), this loads a single COMPACT SPK that was produced
   with NAIF spkmerge from that same 200128RU_SCPSE series, keeping only the
   9 bodies this demo needs (Sun + the 8 planet barycenters):

       cassini_barycenters_2005_2010.bsp   (~0.5 MB, built by spkmerge)
       - source: 71 files 200128RU_SCPSE_04303_05004 .. _09355_10014
       - bodies: 1,2,3,4,5,6,7,8,10  (all w.r.t. SOLAR SYSTEM BARYCENTER)
       - coverage: 2004-12-15 .. 2010-01-10  (single continuous interval,
         verified with brief -- no gaps)
       - reproduce with: deps/cspice/exe/spkmerge.exe < merge_setup.txt
         (run from CassiniData/cosp_1000/data/spk/)

   The demo starts at 2005-01-01T00:00:00, giving ~5 years (1835 days) of
   forward coverage for both the initial state and the reference trajectory.

   gm_de440.tpc provides the GM (BODYn_GM) values used for the masses.
   PATH_VALUES points at the local archive 'data' directory (for lsk + pck).
   The merged SPK lives in the project root, so it is given by absolute path.

\begindata

   PATH_VALUES  = ( 'C:/Users/tikki/Desktop/Projects/PhysicCore/CassiniData/cosp_1000/data' )

   PATH_SYMBOLS = ( 'KERNELS' )

   KERNELS_TO_LOAD = ( '$KERNELS/lsk/naif0012.tls'
                       '$KERNELS/pck/gm_de440.tpc'
                       'C:/Users/tikki/Desktop/Projects/PhysicCore/cassini_barycenters_2005_2010.bsp'
                     )

\begintext

   End of meta-kernel.
