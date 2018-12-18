# gr-applications
Application interface for gnuradio simulation or USRP transmissions

# Installation

- prerequisite:
  
  OpenCV (>=4.0.0)
  
- install gr-applications:
```
cd gr-applications
mkdir build
cd build
cmake ..
make
sudo make install
```

# Supported Data formats:

- Images:
  Integrate OpenCV image reader, thus reads (.bmp, .png, .jpg...)

- Videos:
  Currently support .avi files only, but can support default camera capturing.
  note: in camera capturing mode, the image is compressed in format of jpg.
