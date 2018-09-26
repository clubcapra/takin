# capra_qrcode_detection

Capra-Takin's **capra_qrcode_detection** package is a simple wrapper
around zbar_ros's barcode_reader, used to detect QR code data in
real-time simulation or real-world scenarios.

### Dependencies

See [capra_qrcode_detection dependecies](doc/dependencies.md)

### Usage

$ *roslaunch capra_qrcode_detection qrcode_detection.launch*

Detected QR code data will be published through the `/barcode` topic.
