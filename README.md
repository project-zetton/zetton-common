# zetton-common

Common definitions and utilities in ROS environment for Project Zetton.

## Prerequisites

Recommended environment:

- Ubuntu 18.04
- ROS Melodic
- OpenCV 4.1.0+
- GStreamer 1.14+

## Usage

### Stream

- `CvGstStreamSource`: capturing stream with OpenCV and GStreamer. [[example code]](example/cv_gst_rtsp_stream.cc)

   | Platform   | Protocol | Codec | Result                              |
   | ---------- | -------- | ----- | ----------------------------------- |
   | PC w/ CPU  | RTSP     | H264  | Works                               |
   | PC w/ GPU  | RTSP     | H264  | Works in GUI (not in headless mode) |
   | Jetson TX2 | RTSP     | H264  | Works                               |

## License

- For academic use, this project is licensed under the 2-clause BSD License - see the [LICENSE file](LICENSE) for details.
- For commercial use, please contact [Yusu Pan](mailto:xxdsox@gmail.com).
