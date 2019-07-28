INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_USB_COM_READ usb_com_read)

FIND_PATH(
    USB_COM_READ_INCLUDE_DIRS
    NAMES usb_com_read/api.h
    HINTS $ENV{USB_COM_READ_DIR}/include
        ${PC_USB_COM_READ_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    USB_COM_READ_LIBRARIES
    NAMES gnuradio-usb_com_read
    HINTS $ENV{USB_COM_READ_DIR}/lib
        ${PC_USB_COM_READ_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(USB_COM_READ DEFAULT_MSG USB_COM_READ_LIBRARIES USB_COM_READ_INCLUDE_DIRS)
MARK_AS_ADVANCED(USB_COM_READ_LIBRARIES USB_COM_READ_INCLUDE_DIRS)

