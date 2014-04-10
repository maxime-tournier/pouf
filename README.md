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


Requirements
------------

- a working [SOFA] installation. The Python plug-in is highly recommended.
- a c++11 compiler

Installation
------------

Just copy the directory inside either `sofa/applications/plugins` or
`sofa/applications-dev/plugins` then re-run `cmake`.

Edit `CMakeCache.txt` to set:

	 SOFA-DEVPLUGIN_POUF:BOOL=ON

Save, then rebuild. That should be it.

Usage
-----

Have a look in the `scene` directory, which contains various example
scenes built with this plugin.


[sofa]: http://www.sofa-framework.org/
