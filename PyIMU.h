//
// Created by artem on 15.02.18.
//

#ifndef PYIMUMODULEFORC_PYIMU_H
#define PYIMUMODULEFORC_PYIMU_H

#define MPU6050_ADRESS  0X68
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define A_SENSETIVE     16384.f
#define G_SENSETIVE     131.f
#define MAXPATH         60
#define UDELAY          50

#include <python3.5/Python.h>
#include "python3.5/structmember.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <time.h>
#include "IMUlib.h"

#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#undef I2C_SMBUS_I2C_BLOCK_DATA
#define I2C_SMBUS_I2C_BLOCK_BROKEN      6
#define I2C_SMBUS_I2C_BLOCK_DATA        8
#endif



static PyObject* PyIMUError;    // ошибка модуля

static Quaternion quaternion = {1.f, 0.f, 0.f, 0.f};  // действующий кватернион

clock_t lastTick = 0;   // время с предыдущего вызова


//static PyTypeObject PyMFilter_Type;   // тип данных фильтра
#define PyMFilter_Check(x) ((x) -> ob_type == &PyMFilter_Type)        // ф-ия проверки фильтр ли это

typedef struct                              // структура объекта
{
    PyObject_HEAD
    float beta;
    float yaw;
    float pitch;
    float roll;
    int exit;   // метка выхода из бесконечного цикла
} PyMFilterObject;

int openI2Cport(int port);
int closeI2Cport(int port);
int setSlaveAdress(int port, int addr); // установить адрес ведомого устройства
int initMPU6050(int port);         // инициализация MPU6050
int readMPU6050Data(int port, int32_t* data); // получение данных с MPU6050

static void MFilter_dealloc(PyMFilterObject* self);      // Деструктор
static PyObject* PyMFilter_new(PyTypeObject *type, PyObject *args, PyObject *kwds);     // создает новый динамический объект и заполняет параметры
static int MFilter_init(PyMFilterObject *self, PyObject *args);      // вызывается при вызове конструктора, args - заполняемые параметры
static PyObject* MFilter_updateAngle(PyMFilterObject *self, PyObject *args);     // ф-ия пользовательская
static PyObject* MFilter_updateAngleInCycle(PyMFilterObject *self, PyObject *args);  // бесконечный цикл проверки углов с I2C
static PyObject* MFilter_exitInCycle(PyMFilterObject *self);        // выход из бесконечного цикла

static PyObject* MFilter_ThreadTest(PyMFilterObject *self, PyObject *args);

PyMODINIT_FUNC PyInit_PyIMU(void);      // ф-ия инициализации модуля


static PyMemberDef PyIMU_members[] =        // переменные
        {
                {"beta",  T_FLOAT, offsetof(PyMFilterObject, beta),  READONLY, "beta parameter"},
                {"yaw",   T_FLOAT, offsetof(PyMFilterObject, yaw),   READONLY, "yaw angle"},
                {"pitch", T_FLOAT, offsetof(PyMFilterObject, pitch), READONLY, "pitch angle"},
                {"roll",  T_FLOAT, offsetof(PyMFilterObject, roll),  READONLY, "roll angle"},
                {NULL}  // Sentinel
        };

static PyMethodDef PyMFilter_methods[] =        // методы
        {
                {"updateAngle", (PyCFunction)MFilter_updateAngle, METH_VARARGS, "TempUpdateAngle"},
                {"updateAngleInCycle", (PyCFunction)MFilter_updateAngleInCycle, METH_VARARGS, "Update angle in cycle"},
                {"exit", (PyCFunction)MFilter_exitInCycle, METH_NOARGS, "Exit in infinity cycle"},
                {"test", (PyCFunction)MFilter_ThreadTest, METH_VARARGS, "Test threading in cpython"},
                {NULL} // Sentinel
        };

static PyTypeObject PyMFilter_Type = {      // определения своего класса // тип данных фильтра
        PyVarObject_HEAD_INIT(NULL, 0)
        "PyIMU.MFilter",            // tp_name
        sizeof(PyMFilterObject),    // basic size
        0,                          // tp_itemsize
        (destructor)MFilter_dealloc, // tp_dealloc
        0,                          // tp_print
        0,                         // tp_getattr
        0,                          // tp_setattr
        0,                          // tp_reserved
        0,                          // tp_repr
        0,                          // tp_as_number
        0,                          // tp_as_sequence
        0,                          // tp_as_mapping
        0,                          // tp_hash
        0,                          // tp_call
        0,                          // tp_str
        0,                          // tp_getattro
        0,                          // tp_setattro
        0,                          // tp_as_buffer
        Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,         // tp_flags
        "MajvikFilter object",      // tp_doc
        0,                         // tp_traverse
        0,                         // tp_clear
        0,                         // tp_richcompare
        0,                         // tp_weaklistoffset
        0,                         // tp_iter
        0,                         // tp_iternext
        PyMFilter_methods,         // tp_methods
        PyIMU_members,             // tp_members
        0,                         // tp_getset
        0,                         // tp_base
        0,                         // tp_dict
        0,                         // tp_descr_get
        0,                         // tp_descr_set
        0,                         // tp_dictoffset
        (initproc)MFilter_init,    // tp_init
        0,                         // tp_alloc
        PyMFilter_new,               // tp_new
};


static PyModuleDef PyIMUModule =        // определение модуля
        {
                PyModuleDef_HEAD_INIT,
                "PyIMU",
                "Module for IMU sensor,"
                -1,
                NULL
        };




#endif //PYIMUMODULEFORC_PYIMU_H
