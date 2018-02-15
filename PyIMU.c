//
// Created by artem on 15.02.18.
//

#include "PyIMU.h"

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
    return NULL;
}

static PyObject* MFilter_updateAngleInCycle(PyMFilterObject *self, PyObject *args)    // бесконечный цикл проверки углов с I2C
{
    int bus;
    if(! PyArg_ParseTuple(args, "i", &bus)) return NULL;
    /*
     * подключение к I2C и т.д.
     */
    while (!Exit)
    {

    }
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