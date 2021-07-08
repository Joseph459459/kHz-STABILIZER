QT       += core gui printsupport concurrent serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

CONFIG(release, debug|release) {
    CONFIG += optimize_full
}

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += /opt/pylon/include

SOURCES += \
    camera_select.cpp \
    console.cpp \
    feedback_cam.cpp \
    filt.cpp \
    kHz_Stabilizer.cpp \
    main.cpp \
    monitor_cam.cpp \
    processing_thread.cpp \
    qcustomplot.cpp

HEADERS += \
    SDTFT_algo.h \
    camera_select.h \
    console.h \
    feedback_cam.h \
    filt.h \
    imageviewer.h \
    kHz_Stabilizer.h \
    kHz_macros.h \
    monitor_cam.h \
    processing_thread.h \
    qcustomplot.h

FORMS += \
    camera_select.ui \
    feedback_cam.ui \
    kHz_Stabilizer.ui \
    monitor_cam.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -llapack -lblas -larmadillo -lgsl -lfftw3f -lfftw3

LIBS += -L$$(PYLON_ROOT)/lib/ -lpylonbase
LIBS += -L$$(PYLON_ROOT)/lib/ -lpylonbase-6.2.0
LIBS += -L$$(PYLON_ROOT)/lib/ -lpylon_TL_gige
LIBS += -L$$(PYLON_ROOT)/lib/ -lpylon_TL_gige-6.2.0
LIBS += -L$$(PYLON_ROOT)/lib/ -lpylonc-6.2.0
LIBS += -L$$(PYLON_ROOT)/lib/ -lGenApi_gcc_v3_1_Basler_pylon
LIBS += -L$$(PYLON_ROOT)/lib/ -lGCBase_gcc_v3_1_Basler_pylon


RESOURCES += \
    khz_Stabilizer.qrc


