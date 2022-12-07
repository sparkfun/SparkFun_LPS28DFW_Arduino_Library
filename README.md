SparkFun LPS28DFW Arduino Library
========================================
<table class="table table-striped table-bordered"?
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/21221"><img src="https://cdn.sparkfun.com/assets/parts/2/0/9/7/7/21221_SEN-_Barometer-_01.jpg" alt="SparkFun Absolute Digital Barometer - LPS28DFW (Qwiic)"></a></td>
    <td><a href="https://www.sparkfun.com/products/21222"><img src="https://cdn.sparkfun.com/r/600-600/assets/parts/2/0/9/7/8/21222_SEN-_Mirco_Barometer-_01.jpg" alt="SparkFun Micro Triple Axis Accelerometer Breakout - BMA400 (Qwiic)"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/21221"><i>SparkFun Absolute Digital Barometer - LPS28DFW (Qwiic)</i></a></td>
    <td><a href="https://www.sparkfun.com/products/21222"><i>SparkFun Micro Absolute Digital Barometer - LPS28DFW (Qwiic)</i></a></td>
  </tr>
</table>

This library provides an easy way to control the LPS28DFW Absolute Digital Barometer. It provides high accuracy pressure data in a water-resistant package. Measurements can be filtered to reduce noise, and an on-board FIFO buffer can be used to store measurements for more efficient burst reading of data. In also has a few interrupt features, such as when the measured pressure exceeds some threshold values.

This library implements [STMicroelectronics' LPS28DFW API](https://github.com/STMicroelectronics/lps28dfw-pid) in an Arduino-friendly way. All functions return error codes to indicate the result of each operation, where LPS28DFW_OK (`0`) indicates success. Most examples ignore the error codes to reduce clutter, but an example is included to demonstrate error handling techniques.

## Repository Contents
* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** - General library properties for the Arduino package manager.

## Documentation
* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Hardware Repo](https://github.com/sparkfun/SparkFun_Qwiic_Barometer_LPS28DFW)** - Repository for the LPS28DFW board.
* **[Library](https://github.com/sparkfun/SparkFun_LPS28DFW_Arduino_Library)** - This library, providing functions to write applications for the LPS28DFW with Arduino IDE.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-absolute-digital-barometer---lps28dfw-qwiic-hookup-guide)** - Basic hookup guide for the LPS28DFW.
* **[LICENSE.md](./LICENSE.md)** - License Information

## Product Versions
* [SEN-21221](https://www.sparkfun.com/products/21208) - Standard Size Initial Release.
* [SEN-21222](https://www.sparkfun.com/products/21207) - Micro Size Initial Release.

## Version History

* [v1.0.0](https://github.com/sparkfun/SparkFun_LPS28DFW_Arduino_Library/releases/tag/v1.0.0) - Initial public release.

## License Information

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

_<COLLABORATION CREDIT>_
