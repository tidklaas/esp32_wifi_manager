# Overview
This is a component for ESP-IDF projects. It mainly aims to provide a
stateless interface for setting, querying and maintaining the network
configuration. Additional features are wrappers around the WPS and
AP scanning interfaces, as well as a fall-back mechanism for when a
newly set WiFi configuration turns out to be unusable for connecting
to a given AP.

# Rationale
Due to the ESP-IDF'programming model one has to keep track of the various
events generated by the WiFi and networking subsystems. This makes it
hard to query or modify the networking configuration through a (mostly)
stateless user interface provided by, for example, a web server.

Another pitfall is that users might lose access to the device by setting
an invalid configuration (as in a mistyped password or SSID). In this
case the WiFi Manager module will fall back to the previous configuration
instead.

There are also some convenience wrappers around the AP scanning and
WPS functionalities.

# Usage
Copy or clone this repository into a directory under your project's
'components' directory. You should now find the menu 'WiFi Manager'
in the 'Component config' menu. Enable the 'WiFi Manager' option and
enter the sub-menu. Here you can configure if the manager should run as
a dedicated task or utilise the global timer queue. You can also change
the compiled-in default configuration when the ESP is in AP or AP+STA
mode.

N.b. If you choose a timer queue based configuration, it might be
necessary to increase the timer task's stack size.

The WiFi Manager module must be started by calling the function
`esp_wmngr_init()` from your main project, after the NVS, default
event loop and TCP adapter have been initialised. From here on the
WiFi Manager will manage all the networking configuration and changes
should only be made via the provided `esp_wmngr_*()` functions.

Please consult the provided documentation in the Doxygen folder for
further information on the provided API.