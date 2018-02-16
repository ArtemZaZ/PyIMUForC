//
// Created by artem on 15.02.18.
//

#include "PyIMU.h"

int openI2Cport(int port)
{
    char path[MAXPATH];
    if(snprintf(path, MAXPATH, "/dev/i2c-%d", port) >= MAXPATH)
    {
        // Вызвать ошибку
        return -1;
    }
    if((port = open(path, O_RDWR)) < 0)
    {
        // Вызвать ошибку
        return -1;
    }
    return port;
}

int closeI2Cport(int port)
{
    if((port != -1) && (close(port) == -1))
    {
        //Вызвать ошибку
        return -1;
    }
    return 0;
}

int setSlaveAdress(int port, int addr)
{
    if((ioctl(port, I2C_SLAVE), addr) < 0)
    {
        //Вызвать ошибку
        return -1;
    }
    return 0;
}

void initMPU6050(int port)
{
    i2c_smbus_write_byte_data(port, PWR_MGMT_1, 0x00);
    i2c_smbus_write_byte_data(port, GYRO_CONFIG, 0x00);
    i2c_smbus_write_byte_data(port, ACCEL_CONFIG, 0x00);
}

uint8_t* readMPU6050Data(int port)
{
    union i2c_smbus_data rawdata;
    rawdata.block[0] = 12;
    uint8_t data[6];
    setSlaveAdress(port, MPU6050_ADRESS);
    i2c_smbus_access(port, I2C_SMBUS_READ, ACCEL_XOUT_H, I2C_SMBUS_BLOCK_DATA, &rawdata);
    data[0] = rawdata.block[1] << 8 | rawdata.block[2];
    data[1] = rawdata.block[3] << 8 | rawdata.block[4];
    data[2] = rawdata.block[5] << 8 | rawdata.block[6];

    data[3] = rawdata.block[7] << 8 | rawdata.block[8];
    data[4] = rawdata.block[9] << 8 | rawdata.block[10];
    data[5] = rawdata.block[11] << 8 | rawdata.block[12];

    return data;
}

static void MFilter_dealloc(PyMFilterObject* self)      // Деструктор
{
    Py_TYPE(self)->tp_free((PyObject*)self);
}


static PyObject* PyMFilter_new(PyTypeObject *type, PyObject *args, PyObject *kwds)  // создает новый динамический объект и заполняет параметры
{
    PyMFilterObject* self;

    self = (PyMFilterObject *)type->tp_alloc(type, 0);
    if(self != NULL)
    {
        self -> yaw = 0.f;
        self -> pitch = 0.f;
        self -> roll = 0.f;
        self -> beta = 0.f;
        self -> exit = 0;
    }
    return (PyObject*)self;
}


static int MFilter_init(PyMFilterObject *self, PyObject *args)      // вызывается при вызове конструктора, args - заполняемые параметры
{
    if(! PyArg_ParseTuple(args, "f", &self -> beta)) return -1;
    return 0;
}

/*
static PyObject* MFilter_updateAngle(PyMFilterObject *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"yaw", "pitch", "roll", NULL};
    if(!PyArg_ParseTupleAndKeywords(args, kwds, "fff", kwlist, self->yaw, self->pitch, self->roll))
        return NULL;
    return PyLong_FromLong(1);
}
*/

static PyObject* MFilter_updateAngle(PyMFilterObject *self, PyObject *args)     // ф-ия пользовательского обновления углов с внешними данными
{
    Vector a;     // ускорения
    Vector w;     // угловые скорости
    float time;
    if(!PyArg_ParseTuple(args, "fffffff", &(a.x), &(a.y), &(a.z), &(w.x), &(w.y), &(w.z), &time)) return NULL;
    quaternion = updateFilterIterator(quaternion, a, w, self -> beta, time);
    Vector temp = quatToEulerAngle(quaternion);
    self -> yaw = temp.x;
    self -> pitch = temp.y;
    self -> roll = temp.z;
    return Py_None;
}

static PyObject* MFilter_updateAngleInCycle(PyMFilterObject *self, PyObject *args)    // бесконечный цикл проверки углов с I2C
{
    self -> exit = 0;
    int bus;
    uint8_t* data;
    float time = 0.0001;
    if(! PyArg_ParseTuple(args, "i", &bus)) return NULL;
    bus = openI2Cport(bus);
    setSlaveAdress(bus, MPU6050_ADRESS);
    initMPU6050(bus);
    while (!(self -> exit))
    {
        data = readMPU6050Data(bus);
        quaternion = updateFilterIterator(quaternion, (Vector){data[0], data[1], data[2]}, (Vector){data[3], data[4], data[5]}, self->beta, time);
        Vector temp = quatToEulerAngle(quaternion);
        self -> yaw = temp.x;
        self -> pitch = temp.y;
        self -> roll = temp.z;
    }
    closeI2Cport(bus);
    return Py_None;
}

static PyObject* MFilter_exitInCycle(PyMFilterObject *self)
{
    self -> exit = 1;
    return Py_None;
}

PyMODINIT_FUNC PyInit_PyIMU(void)
{
    PyObject* m;
    //PyMFilter_Type.tp_new = PyType_GenericNew;
    if(PyType_Ready(&PyMFilter_Type) < 0) return NULL;

    m = PyModule_Create(&PyIMUModule);
    if(m == NULL) return NULL;

    PyIMUError = PyErr_NewException("PyIMU.error", NULL, NULL);
    Py_INCREF(PyIMUError);
    PyModule_AddObject(m, "error", PyIMUError);

    Py_INCREF(&PyMFilter_Type);                 // добавление своего типа данных в модуль
    PyModule_AddObject(m, "MFilter", (PyObject *)&PyMFilter_Type);
    return m;
}

int main(int argc, char *argv[])
{
    wchar_t *program = Py_DecodeLocale(argv[0], NULL);
    if (program == NULL)
    {
        fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
        exit(1);
    }
    /*добавляем встроенный модуль перед инициализацией питона*/
    PyImport_AppendInittab("PyIMU", PyInit_PyIMU);
    /*передаем argv[0] интерпретатору питона */
    Py_SetProgramName(program);
    /*инициализируем интерпретатор питона*/
    Py_Initialize();
    /*опционально - импортируем модуль? модуль может импортировать также внешний скрипт*/
    PyImport_ImportModule("PyIMU");
    PyMem_RawFree(program);
    return 0;
}