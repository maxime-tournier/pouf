POUF !
======

*pouf* is a small plugin for the [SOFA][sofa] framework. It is aimed
at developping contact solvers and physically based character
animation tools. If you are looking for simple
Finite-State-Machine/PID character control with frictional contacts,
you might want to give it a try.

The plugin itself contains solvers, mappings and forcefields written
in c++, but it also comes with python scripts to ease the creation and
scripting of SOFA scenes.

![fancy screenshot](https://raw.github.com/maxime-tournier/pouf/master/doc/screenshot.png)

So far *pouf* has only been tested on OSX 10.9 and Debian. It
*might* work under windows.

Requirements
------------

- a working [SOFA] installation with the python plugin activated.

Building
--------

Just copy/link the directory inside either `sofa/applications/plugins`
or `sofa/applications-dev/plugins` then re-run `cmake`.

Edit `CMakeCache.txt` to set:

	 SOFA-DEVPLUGIN_POUF:BOOL=ON

Save, then run `make`.

Usage
-----

**IMPORTANT:** you must start sofa with the DAG graph option:

`bin/ruSofa -s dag`


Have a look in the `scene` directory: it contains various example
scenes for the following features:

- `robot.py`: basic articulated rigid body
- `servo.py`: robot with PID controllers
- `fsm.py`: Finite State Machine + PID control
- `contact.py`: drawing contact infos (points/forces, support polygon, center of pressure)
- `constraint.py`: kinematic constraints from python
- `am.py`: angular momentum damping 

You can open these scenes directly from SOFA.

[sofa]: http://www.sofa-framework.org/
