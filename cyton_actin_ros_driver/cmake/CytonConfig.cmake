# Make sure we can find the Actin distribution
set(CYTON_ROOT_DIR ${PROJECT_SOURCE_DIR}/../Robai/Cyton\ Gamma\ 1500\ Viewer_4.0.12-20160307)

set(toolkits ${CYTON_ROOT_DIR}/toolkits)
set(external ${CYTON_ROOT_DIR}/external)

set(src ${toolkits}/examples/src)
# set(render_inc "${external}/render_OSS-20150520-gcc4.8-amd64/OSG-3.2.1/include" "${external}/render_OSS-20150520-gcc4.8-amd64/Qt-5.3.2/include")
# set(render_lib  "${external}/sensor_OSS-20150520-gcc4.8-amd64/OpenCV-2.4.10/lib" "${external}/sensor_OSS-20150520-gcc4.8-amd64/tiff-3.9.7/lib" "${external}/sensor_OSS-20150520-gcc4.8-amd64/libdc1394-2.1.0/lib" "${external}/sensor_OSS-20150520-gcc4.8-amd64/libraw1394-2.0.2/lib" ${external}/render_OSS-20150520-gcc4.8-amd64/Qt-5.3.2/lib ${external}/render_OSS-20150520-gcc4.8-amd64/OSG-3.2.1/lib)
# set(qt_moc_path "${external}/render_OSS-20150520-gcc4.8-amd64/Qt-5.3.2/bin")

# Set directory to include headers
set(CYTON_INCLUDE_DIRS ${CYTON_ROOT_DIR}/include)

set(CYTON_RENDER_INCLUDE_DIRS ${external}/render_OSS-20150520-gcc4.8-amd64/OSG-3.2.1/include ${external}/render_OSS-20150520-gcc4.8-amd64/Qt-5.3.2/include)

# Set directory to library files.
set(CYTON_LIBRARY_DIRS ${CYTON_ROOT_DIR}/lib ${CYTON_ROOT_DIR}/bin)

set(CYTON_RENDER_LIBRARY_DIRS ${external}/sensor_OSS-20150520-gcc4.8-amd64/OpenCV-2.4.10/lib ${external}/render_OSS-20150520-gcc4.8-amd64/OSG-3.2.1/lib ${external}/render_OSS-20150520-gcc4.8-amd64/Qt-5.3.2/lib ${external}/sensor_OSS-20150520-gcc4.8-amd64/tiff-3.9.7/lib ${external}/sensor_OSS-20150520-gcc4.8-amd64/libjpeg9a/lib)

# set(qt_lib Qt5Core Qt5Gui Qt5Widgets)
# set(osg_lib osg)

# CYTON DEFINITIONS:
set(CYTON_DEFINITIONS -DEC_BUILD_SHARED_LIBS
                      -DEC_HAVE_ACTIN     
                      -DUNICODE
                      -D_UNICODE
                      "-DACTIN_VERSION_MAJOR=4"
                      "-DACTIN_VERSION_MINOR=0"
                      "-DACTIN_VERSION_PATCH=12"
                      "-DACTIN_VERSION=\"4.0.12\"")

########CYTON CONFIGURATION###########
#Can remove some libraries based on application
 set(CYTON_LIBRARIES
      ecConvertSimulation
      ecCytonHardwareInterface
      ecSystemSimulation
      ecSimulation
      ecConvertSystem
      ecRendCore
      ecVisualization
      ecFoundCommon
      ecFoundCore
      ecXml
      ecXmlReaderWriter
      ecManipulation
      ecManipulationDirector
      ecManipulator
      ecGeometry
      ecControl
      ecPathPlanning
      ecMatrixUtilities
      ecControlCore
      ecFunction
      ecPlugins
      ecRemoteCommand
)
