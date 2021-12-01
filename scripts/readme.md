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

**convert.sh**  
A scripted file for simplifying conversion of `.ubx` files to  
`.obs, .nav, .sbas` and also postproccessing.  

Currently does not produces best result.  
**DOING THE CONVERSION AND POSTPROCCESSING BY HAND IS CURRENTLY FOR THE BEST**
