# Scripts

**filter_positions.py**  
A python script for filtering out positions from post-processed data,  
with the help of time stamps from the rover.

Usage:  
`python3 filter_positions <POSITION DATA> <TIME STAMPS>`

* Position data file format is `.pos`
* Time stamp file format is `.txt`
  * Time stamps should be formatted as `YYYY:MM:DD HH:MM:SS.MS\n`  
  where `MS` stands for milliseconds.
* Only requires python3 to be installed.
----
**convert.sh**   
A scripted file for simplifying conversion of `.ubx` files to  
`.obs, .nav, .sbas` and also statically post-processes the data to produce a usable `.pos`.

Uses @rtklibexplorer [RTKBLIB](https://github.com/rtklibexplorer/RTKLIB) fork.
> Convbin & rnx2rtkp needs to be installed and callable as `convbin` & `rnx2rtkp`

Usage:  
`./convert.sh <BASE.UBX PATH> <BASE.NAV PATH> <ROVER.UBX PATH>`

---

**install_dependencies.sh**  
Linux based Bash script that installs rtklib and other dependencies for linux operating systems (\*only tested on *ubuntu 20.04lts*).

* git
* rtklibexplorer/RTKLIB (\*v.demo5, dec 2021)
  * convbin
  * rnx2rtkp
