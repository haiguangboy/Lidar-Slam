/****************************************************************************
** Meta object code from reading C++ file 'opengl_3d_scene.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../include/imu_tk/vis_extra/opengl_3d_scene.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'opengl_3d_scene.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_OpenGL3DScene[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      36,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      21,   15,   14,   14, 0x05,
      38,   14,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
      68,   53,   14,   14, 0x0a,
     107,  102,   14,   14, 0x2a,
     136,   14,   14,   14, 0x0a,
     178,  157,   14,   14, 0x0a,
     252,  238,   14,   14, 0x0a,
     285,  278,   14,   14, 0x2a,
     305,   14,   14,   14, 0x0a,
     330,  321,   14,   14, 0x0a,
     355,   14,   14,   14, 0x2a,
     378,  376,   14,   14, 0x0a,
     395,  391,   14,   14, 0x0a,
     417,   14,   14,   14, 0x2a,
     432,  391,   14,   14, 0x0a,
     456,   14,   14,   14, 0x2a,
     473,  391,   14,   14, 0x0a,
     497,   14,   14,   14, 0x2a,
     514,  391,   14,   14, 0x0a,
     539,   14,   14,   14, 0x2a,
     557,  391,   14,   14, 0x0a,
     584,   14,   14,   14, 0x2a,
     604,  391,   14,   14, 0x0a,
     632,   14,   14,   14, 0x2a,
     653,  391,   14,   14, 0x0a,
     677,   14,   14,   14, 0x2a,
     694,  391,   14,   14, 0x0a,
     717,   14,   14,   14, 0x2a,
     733,  391,   14,   14, 0x0a,
     757,   14,   14,   14, 0x2a,
     774,  391,   14,   14, 0x0a,
     797,   14,   14,   14, 0x2a,
     813,  391,   14,   14, 0x0a,
     837,   14,   14,   14, 0x2a,
     854,  391,   14,   14, 0x0a,
     877,   14,   14,   14, 0x2a,

       0        // eod
};

static const char qt_meta_stringdata_OpenGL3DScene[] = {
    "OpenGL3DScene\0\0value\0zoomChanged(int)\0"
    "sceneUpdated()\0name,update_gl\0"
    "updateStructure(std::string,bool)\0"
    "name\0updateStructure(std::string)\0"
    "updateAllStructure()\0tx,ty,tz,r1x,r2y,r3z\0"
    "moveCamera(GLfloat,GLfloat,GLfloat,GLfloat,GLfloat,GLfloat)\0"
    "enable,offset\0setAutoAdjust(bool,float)\0"
    "enable\0setAutoAdjust(bool)\0setGlobalView()\0"
    "good_pos\0setFirstPersonView(bool)\0"
    "setFirstPersonView()\0z\0setZoom(int)\0"
    "inc\0moveCameraUp(GLfloat)\0moveCameraUp()\0"
    "moveCameraDown(GLfloat)\0moveCameraDown()\0"
    "moveCameraLeft(GLfloat)\0moveCameraLeft()\0"
    "moveCameraRight(GLfloat)\0moveCameraRight()\0"
    "moveCameraForward(GLfloat)\0"
    "moveCameraForward()\0moveCameraBackward(GLfloat)\0"
    "moveCameraBackward()\0turnCameraCCWY(GLfloat)\0"
    "turnCameraCCWY()\0turnCameraCWY(GLfloat)\0"
    "turnCameraCWY()\0turnCameraCCWX(GLfloat)\0"
    "turnCameraCCWX()\0turnCameraCWX(GLfloat)\0"
    "turnCameraCWX()\0turnCameraCCWZ(GLfloat)\0"
    "turnCameraCCWZ()\0turnCameraCWZ(GLfloat)\0"
    "turnCameraCWZ()\0"
};

void OpenGL3DScene::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        OpenGL3DScene *_t = static_cast<OpenGL3DScene *>(_o);
        switch (_id) {
        case 0: _t->zoomChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->sceneUpdated(); break;
        case 2: _t->updateStructure((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 3: _t->updateStructure((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 4: _t->updateAllStructure(); break;
        case 5: _t->moveCamera((*reinterpret_cast< GLfloat(*)>(_a[1])),(*reinterpret_cast< GLfloat(*)>(_a[2])),(*reinterpret_cast< GLfloat(*)>(_a[3])),(*reinterpret_cast< GLfloat(*)>(_a[4])),(*reinterpret_cast< GLfloat(*)>(_a[5])),(*reinterpret_cast< GLfloat(*)>(_a[6]))); break;
        case 6: _t->setAutoAdjust((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 7: _t->setAutoAdjust((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->setGlobalView(); break;
        case 9: _t->setFirstPersonView((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->setFirstPersonView(); break;
        case 11: _t->setZoom((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->moveCameraUp((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 13: _t->moveCameraUp(); break;
        case 14: _t->moveCameraDown((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 15: _t->moveCameraDown(); break;
        case 16: _t->moveCameraLeft((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 17: _t->moveCameraLeft(); break;
        case 18: _t->moveCameraRight((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 19: _t->moveCameraRight(); break;
        case 20: _t->moveCameraForward((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 21: _t->moveCameraForward(); break;
        case 22: _t->moveCameraBackward((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 23: _t->moveCameraBackward(); break;
        case 24: _t->turnCameraCCWY((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 25: _t->turnCameraCCWY(); break;
        case 26: _t->turnCameraCWY((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 27: _t->turnCameraCWY(); break;
        case 28: _t->turnCameraCCWX((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 29: _t->turnCameraCCWX(); break;
        case 30: _t->turnCameraCWX((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 31: _t->turnCameraCWX(); break;
        case 32: _t->turnCameraCCWZ((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 33: _t->turnCameraCCWZ(); break;
        case 34: _t->turnCameraCWZ((*reinterpret_cast< GLfloat(*)>(_a[1]))); break;
        case 35: _t->turnCameraCWZ(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData OpenGL3DScene::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject OpenGL3DScene::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_OpenGL3DScene,
      qt_meta_data_OpenGL3DScene, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &OpenGL3DScene::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *OpenGL3DScene::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *OpenGL3DScene::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_OpenGL3DScene))
        return static_cast<void*>(const_cast< OpenGL3DScene*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int OpenGL3DScene::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 36)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 36;
    }
    return _id;
}

// SIGNAL 0
void OpenGL3DScene::zoomChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void OpenGL3DScene::sceneUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
