# Campus Tirol Motorsport KiCad Libraries

This repository hosts the custom symbols and footprints used by the **Campus Tirol Motorsport** Formula-Student Team.

To make use of these parts clone this repository locally,
create a KiCad Environment Variable e.g. `KICAD_CTM` pointing to it.
(Preferences>Configure Paths)
![Configure Environment Variable](/images/Env_Variable.png)

Add the Libraries and footprints to Kicad in Global Libraries (Preferences>Manage Symbols Library and Preferences>Manage Footprint Library) as followed:
Schematic symbols can then be found in `${KICAD_CTM}/ctm.kicad_sym` the footprints in `${KICAD_CTM}/ctm.pretty`.
![Configure Symbol library](/images/Symbol_lib.png)
![Configure Footprint library](/images/footp_lib.png)

If you would like to contribute to these libraries, please go ahead and
open a merge request.
