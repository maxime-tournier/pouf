POUF !
======

*pouf* is a small plugin for the [SOFA][sofa] framework. It is aimed
at developping contact solvers and physically based character
animation tools.

If you are looking for simple Finite-State-Machine/PID character
control, you might want to give it a try.

The plugin itself contains solvers, mappings and forcefields written
in c++, but it also comes with python scripts to ease the creation and
scripting of SOFA scenes.

So far this plugin has only been tested on OSX 10.9 and Debian. It
*might* work under windows.

![fancy screenshot](https://raw.github.com/maxime-tournier/pouf/master/doc/screenshot.png)


Requirements
------------

- a working [SOFA] installation. Enabling the Python plug-in is highly
  recommended.

Installation
------------

Just copy the directory inside either `sofa/applications/plugins` or
`sofa/applications-dev/plugins` then re-run `cmake`.

Edit `CMakeCache.txt` to set:

	 SOFA-DEVPLUGIN_POUF:BOOL=ON

Save, then rebuild. That should be it.

Usage
-----

Have a look in the `scene` directory: it contains various example
scenes for the following features:

- `robot.py`: basic articulated rigid body
- `servo.py`: robot with PID controllers
- `fsm.py`: Finite State Machine + PID control
- `contact.py`: drawing + contact infos (points/forces, support polygon, center of pressure)


[sofa]: http://www.sofa-framework.org/
