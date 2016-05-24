POUF
====

*pouf* is a small python plugin for the [SOFA][sofa] framework. It is
aimed at developping contact solvers and physically-based character
animation tools. If you are looking for simple
Finite-State-Machine/PID character control with frictional contacts,
you might want to give it a try.

The plugin itself used to contain solvers, mappings and forcefields
written in c++, but most of it has been merged to the `Compliant`
plugin which comes with SOFA. `pouf` now only contains python scripts
to ease the creation and scripting of SOFA scenes for robot control.

![fancy screenshot](https://raw.github.com/maxime-tournier/pouf/master/doc/screenshot.png)


Requirements
------------

- a working [SOFA] installation with the `SofaPython` and `Compliant` plugins activated.
- Numpy/Scipy


Using
-----

(TODO is this still correct? now the plugin is python-only)


Just copy/link the directory inside either `sofa/applications/plugins`
or `sofa/applications-dev/plugins`.

Examples
--------

The `scene` directory contains various example scenes demonstrating the
following features:

- `robot.py`: basic articulated rigid body
- `servo.py`: robot with PID controllers
- `fsm.py`: Finite State Machine + PID control
- `contact.py`: drawing contact infos (points/forces, support polygon, center of pressure)
- `constraint.py`: kinematic constraints from python
- `am.py`: angular momentum damping (broken :-/)

You can open these scenes directly from SOFA.

[sofa]: http://www.sofa-framework.org/

