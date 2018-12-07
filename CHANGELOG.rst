^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ibeo_lux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2018-12-07)
------------------
* Merge pull request `#5 <https://github.com/astuff/ibeo_lux/issues/5>`_ from astuff/maint/add_urls
  Adding website URL to package.xml. Fixing changelog.
* Merge pull request `#6 <https://github.com/astuff/ibeo_lux/issues/6>`_ from ShepelIlya/patch-1
* Added filling of number_of_objects field to ObjectData2280 message
  Now uint16 field with number of objects in message is filling in IbeoLuxRosMsgHandler::fill2280 function.
* Contributors: Joshua Whitley, Rinda Gunjala, Sam Rustan, ShepelIlya, Zach Oakes

2.0.0 (2018-07-05)
------------------
* Updating package.xml with correct URLs.
* Changing package.xml to reflect correct license.
* Adding product page to README.
* Enabling Travis.
* Cleaning up package location in prep for open-source.
* Moving to using ibeo_core in preparation for open-sourcing.
* Updating package.xml to format 2.
* Lowering duration on wireframe boxes.
* Fixing segfault in read.
* Breaking out of read loop if ROS dies.
* Adding namespace to object boxes.
* Lighter-weight parsing.
* Fixing several display-related bugs. Adding label to PointCloud2 for layer.
* Marker type was set twice for object boxes.
* Testing new marker size scale.
* Fixing orientation scaling. Changes to core.
* Changing to new parsing method.
* Making connection to socket the same as other drivers.
* Adding C++11 support for network_utils.
* Moving sensor-specific data into core.
* Removing using namespace std. Adding driver namespace.
* Making node attempt to reconnect continuously, even after disconnect.
* Moving sensor-specific messages to astuff_sensor_msgs.
* Updating network_interface calls and adding better error reporting.
* upping the hz more to correct visualization lag
* upped hz to 1000 to correct visualization issues
* Adds thread sleeping and callback spin
  Rates were determined by observed output hz from
  driver operation before.
  That and apparently when the intended hz is too
  low it segfaults
* frame_id and stamp now being set on fusion_image message
* core change
* contour points publish to as_tx/object_contour_points
* wireframes publish on as_tx/objects
* increased text label size on objects
* pointclouds to pub to 'as_tx/point_cloud' topic
* problem with labels corrected
* adds classification labels to wireframes on fusion ECU
* Adds checks depending on run mode
  If its running as a single lux then it will only parse lux messages
  opposite if its fusion. this prevents publishing to topics that dont exist.
* Corrects a few issues with Single Lux running
  driver now  parses the single lux scan point correctly. before
  before they were obscenely small values resulting in what appeared
  to be a single point.
  contour points show up in the proper distance. luxi report in centimeters
  but the driver was not scaling it. now it does. you're welcome.
  Added namespacing for objects in RViz. Now they are namespaced by their
  respective classification. Though somehow that change caused the text
  to stop displaying so more investigation is required.
* Changing topic names to be more standardized.
* Changing launch files to match new driver name.
* Changing driver name from ibeo_lux_driver to ibeo_lux.
* changed include to know where network_interface is
* core change and moved division for cm to node code
* adds argument of if sensor is lux or fusion
* converted cubes to line lists for wireframe boxes
* - added the ethernet_raw_tx msg
* - added fusion messages
* - added lux scan data
  - added lux object data
  - added lux vehicle status data
* - added the readme file
* - first commit for ROS Lux driver
* Contributors: Daniel Stanek, Joe Kale, Joshua Whitley, Lu Xu, Sam Rustan
