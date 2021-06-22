QT       += core gui printsupport concurrent serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

QMAKE_CXXFLAGS_RELEASE -= -O2

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
    FASTSTABILIZER.cpp \
    cameraselect.cpp \
    console.cpp \
    feedback_cam.cpp \
    filt.cpp \
    main.cpp \
    monitor_cam.cpp \
    processing_thread.cpp \
    qcustomplot.cpp

HEADERS += \
    FASTSTABILIZER.h \
    FS_macros.h \
    cameraselect.h \
    console.h \
    feedback_cam.h \
    imageviewer.h \
    monitor_cam.h \
    processing_thread.h \
    qcustomplot.h

FORMS += \
    FASTSTABILIZER.ui \
    cameraselect.ui \
    feedback_cam.ui \
    monitor_cam.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -llapack -lblas -larmadillo -lgsl -lfftw3f

LIBS += -L$$PWD/../../../opt/pylon/lib/ -lpylonbase
LIBS += -L$$PWD/../../../opt/pylon/lib/ -lpylonbase-6.2.0
LIBS += -L$$PWD/../../../opt/pylon/lib/ -lpylon_TL_gige
LIBS += -L$$PWD/../../../opt/pylon/lib/ -lpylon_TL_gige-6.2.0
LIBS += -L$$PWD/../../../opt/pylon/lib/ -lpylonc-6.2.0
LIBS += -L$$PWD/../../../opt/pylon/lib/ -lGenApi_gcc_v3_1_Basler_pylon
LIBS += -L$$PWD/../../../opt/pylon/lib/ -lGCBase_gcc_v3_1_Basler_pylon

RESOURCES += \
    khz_Stabilizer.qrc


