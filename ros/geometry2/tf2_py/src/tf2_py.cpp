#include <Python.h>

#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>

// Run x (a tf method, catching TF's exceptions and reraising them as Python exceptions)
//
#define WRAP(x) \
  do { \
  try \
  { \
    x; \
  }  \
  catch (const tf2::ConnectivityException &e) \
  { \
    PyErr_SetString(tf2_connectivityexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::LookupException &e) \
  { \
    PyErr_SetString(tf2_lookupexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::ExtrapolationException &e) \
  { \
    PyErr_SetString(tf2_extrapolationexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::InvalidArgumentException &e) \
  { \
    PyErr_SetString(tf2_invalidargumentexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::TimeoutException &e) \
  { \
    PyErr_SetString(tf2_timeoutexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::TransformException &e) \
  { \
    PyErr_SetString(tf2_exception, e.what()); \
    return NULL; \
  } \
  } while (0)

static PyObject *pModulerospy = NULL;
static PyObject *pModulegeometrymsgs = NULL;
static PyObject *tf2_exception = NULL;
static PyObject *tf2_connectivityexception = NULL, *tf2_lookupexception = NULL, *tf2_extrapolationexception = NULL, 
                *tf2_invalidargumentexception = NULL, *tf2_timeoutexception = NULL;

struct buffer_core_t {
  PyObject_HEAD
  tf2::BufferCore *bc;
};

static PyTypeObject buffer_core_Type = {
  PyObject_HEAD_INIT(&PyType_Type)
  0,                               /*size*/
  "_tf2.BufferCore",                /*name*/
  sizeof(buffer_core_t),           /*basicsize*/
};

static PyObject *PyObject_BorrowAttrString(PyObject* o, const char *name)
{
    PyObject *r = PyObject_GetAttrString(o, name);
    if (r != NULL)
      Py_DECREF(r);
    return r;
}

static PyObject *transform_converter(const geometry_msgs::TransformStamped* transform)
{
  PyObject *pclass, *pargs, *pinst = NULL;
  pclass = PyObject_GetAttrString(pModulegeometrymsgs, "TransformStamped");
  if(pclass == NULL)
  {
    printf("Can't get geometry_msgs.msg.TransformedStamped\n");
    return NULL;
  }

  pargs = Py_BuildValue("()");
  if(pargs == NULL)
  {
    printf("Can't build argument list\n");
    return NULL;
  }

  pinst = PyEval_CallObject(pclass, pargs);
  Py_DECREF(pclass);
  Py_DECREF(pargs);
  if(pinst == NULL)
  {
    printf("Can't create class\n");
    return NULL;
  }

  //we need to convert the time to python
  PyObject *rospy_time = PyObject_GetAttrString(pModulerospy, "Time");
  PyObject *args = Py_BuildValue("ii", transform->header.stamp.sec, transform->header.stamp.nsec);
  PyObject *time_obj = PyObject_CallObject(rospy_time, args);
  Py_DECREF(args);
  Py_DECREF(rospy_time);

  PyObject* pheader = PyObject_GetAttrString(pinst, "header");
  PyObject_SetAttrString(pheader, "stamp", time_obj);
  Py_DECREF(time_obj);

  PyObject_SetAttrString(pheader, "frame_id", PyString_FromString((transform->header.frame_id).c_str()));
  Py_DECREF(pheader);

  PyObject *ptransform = PyObject_GetAttrString(pinst, "transform");
  PyObject *ptranslation = PyObject_GetAttrString(ptransform, "translation");
  PyObject *protation = PyObject_GetAttrString(ptransform, "rotation");
  Py_DECREF(ptransform);

  PyObject_SetAttrString(pinst, "child_frame_id", PyString_FromString((transform->child_frame_id).c_str()));

  PyObject_SetAttrString(ptranslation, "x", PyFloat_FromDouble(transform->transform.translation.x));
  PyObject_SetAttrString(ptranslation, "y", PyFloat_FromDouble(transform->transform.translation.y));
  PyObject_SetAttrString(ptranslation, "z", PyFloat_FromDouble(transform->transform.translation.z));
  Py_DECREF(ptranslation);

  PyObject_SetAttrString(protation, "x", PyFloat_FromDouble(transform->transform.rotation.x));
  PyObject_SetAttrString(protation, "y", PyFloat_FromDouble(transform->transform.rotation.y));
  PyObject_SetAttrString(protation, "z", PyFloat_FromDouble(transform->transform.rotation.z));
  PyObject_SetAttrString(protation, "w", PyFloat_FromDouble(transform->transform.rotation.w));
  Py_DECREF(protation);

  return pinst;
}

static int rostime_converter(PyObject *obj, ros::Time *rt)
{
  PyObject *tsr = PyObject_CallMethod(obj, (char*)"to_sec", NULL);
  if (tsr == NULL) {
    PyErr_SetString(PyExc_TypeError, "time must have a to_sec method, e.g. rospy.Time or rospy.Duration");
    return 0;
  } else {
    (*rt).fromSec(PyFloat_AsDouble(tsr));
    Py_DECREF(tsr);
    return 1;
  }
}

static int rosduration_converter(PyObject *obj, ros::Duration *rt)
{
  PyObject *tsr = PyObject_CallMethod(obj, (char*)"to_sec", NULL);
  if (tsr == NULL) {
    PyErr_SetString(PyExc_TypeError, "time must have a to_sec method, e.g. rospy.Time or rospy.Duration");
    return 0;
  } else {
    (*rt).fromSec(PyFloat_AsDouble(tsr));
    Py_DECREF(tsr);
    return 1;
  }
}

static int BufferCore_init(PyObject *self, PyObject *args, PyObject *kw)
{
  ros::Duration cache_time;

  cache_time.fromSec(tf2::BufferCore::DEFAULT_CACHE_TIME);

  if (!PyArg_ParseTuple(args, "|O&", rosduration_converter, &cache_time))
    return -1;

  ((buffer_core_t*)self)->bc = new tf2::BufferCore(cache_time);

  return 0;
}

/* This may need to be implemented later if we decide to have it in the core
static PyObject *getTFPrefix(PyObject *self, PyObject *args)
{
  if (!PyArg_ParseTuple(args, ""))
    return NULL;
  tf::Transformer *t = ((transformer_t*)self)->t;
  return PyString_FromString(t->getTFPrefix().c_str());
}
*/

static PyObject *allFramesAsYAML(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  return PyString_FromString(bc->allFramesAsYAML().c_str());
}

static PyObject *allFramesAsString(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  return PyString_FromString(bc->allFramesAsString().c_str());
}

static PyObject *canTransformCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame;
  ros::Time time;
  static const char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", (char**)keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  std::string error_msg;
  bool can_transform = bc->canTransform(target_frame, source_frame, time, &error_msg);
  //return PyBool_FromLong(t->canTransform(target_frame, source_frame, time));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

static PyObject *canTransformFullCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  static const char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", (char**)keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;
  std::string error_msg;
  bool can_transform = bc->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, &error_msg);
  //return PyBool_FromLong(t->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

/* Debugging stuff that may need to be implemented later
static PyObject *asListOfStrings(std::vector< std::string > los)
{
  PyObject *r = PyList_New(los.size());
  size_t i;
  for (i = 0; i < los.size(); i++) {
    PyList_SetItem(r, i, PyString_FromString(los[i].c_str()));
  }
  return r;
}

static PyObject *chain(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  std::vector< std::string > output;
  static const char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", (char**)keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;

  WRAP(t->chainAsVector(target_frame, target_time, source_frame, source_time, fixed_frame, output));
  return asListOfStrings(output);
}

static PyObject *getLatestCommonTime(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *source, *dest;
  std::string error_string;
  ros::Time time;

  if (!PyArg_ParseTuple(args, "ss", &source, &dest))
    return NULL;
  int r = t->getLatestCommonTime(source, dest, time, &error_string);
  if (r == 0) {
    PyObject *rospy_time = PyObject_GetAttrString(pModulerospy, "Time");
    PyObject *args = Py_BuildValue("ii", time.sec, time.nsec);
    PyObject *ob = PyObject_CallObject(rospy_time, args);
    Py_DECREF(args);
    Py_DECREF(rospy_time);
    return ob;
  } else {
    PyErr_SetString(tf_exception, error_string.c_str());
    return NULL;
  }
}
*/

static PyObject *lookupTransformCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame;
  ros::Time time;
  static const char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", (char**)keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  geometry_msgs::TransformStamped transform;
  WRAP(transform = bc->lookupTransform(target_frame, source_frame, time));
  geometry_msgs::Vector3 origin = transform.transform.translation;
  geometry_msgs::Quaternion rotation = transform.transform.rotation;
  //TODO: Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
  //return Py_BuildValue("(ddd)(dddd)",
  //    origin.x, origin.y, origin.z,
  //    rotation.x, rotation.y, rotation.z, rotation.w);
}

static PyObject *lookupTransformFullCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  static const char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", (char**)keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;
  geometry_msgs::TransformStamped transform;
  WRAP(transform = bc->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
  geometry_msgs::Vector3 origin = transform.transform.translation;
  geometry_msgs::Quaternion rotation = transform.transform.rotation;
  //TODO: Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
}
/*
static PyObject *lookupTwistCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *tracking_frame, *observation_frame;
  ros::Time time;
  ros::Duration averaging_interval;
  static const char *keywords[] = { "tracking_frame", "observation_frame", "time", "averaging_interval", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&O&", (char**)keywords, &tracking_frame, &observation_frame, rostime_converter, &time, rosduration_converter, &averaging_interval))
    return NULL;
  geometry_msgs::Twist twist;
  WRAP(twist = bc->lookupTwist(tracking_frame, observation_frame, time, averaging_interval));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}

static PyObject *lookupTwistFullCore(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *tracking_frame, *observation_frame, *reference_frame, *reference_point_frame;
  ros::Time time;
  ros::Duration averaging_interval;
  double px, py, pz;

  if (!PyArg_ParseTuple(args, "sss(ddd)sO&O&",
                        &tracking_frame,
                        &observation_frame,
                        &reference_frame,
                        &px, &py, &pz,
                        &reference_point_frame,
                        rostime_converter, &time,
                        rosduration_converter, &averaging_interval))
    return NULL;
  geometry_msgs::Twist twist;
  tf::Point pt(px, py, pz);
  WRAP(twist = bc->lookupTwist(tracking_frame, observation_frame, reference_frame, pt, reference_point_frame, time, averaging_interval));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}
*/
static PyObject *setTransform(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  PyObject *py_transform;
  char *authority;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority))
    return NULL;

  geometry_msgs::TransformStamped transform;
  PyObject *header = PyObject_BorrowAttrString(py_transform, "header");
  transform.child_frame_id = PyString_AsString(PyObject_BorrowAttrString(py_transform, "child_frame_id"));
  transform.header.frame_id = PyString_AsString(PyObject_BorrowAttrString(header, "frame_id"));
  if (rostime_converter(PyObject_BorrowAttrString(header, "stamp"), &transform.header.stamp) != 1)
    return NULL;

  PyObject *mtransform = PyObject_BorrowAttrString(py_transform, "transform");
  PyObject *translation = PyObject_BorrowAttrString(mtransform, "translation");
  transform.transform.translation.x = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "x"));
  transform.transform.translation.y = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "y"));
  transform.transform.translation.z = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "z"));
  PyObject *rotation = PyObject_BorrowAttrString(mtransform, "rotation");
  transform.transform.rotation.x = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "x"));
  transform.transform.rotation.y = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "y"));
  transform.transform.rotation.z = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "z"));
  transform.transform.rotation.w = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "w"));

  bc->setTransform(transform, authority);
  Py_RETURN_NONE;
}

static PyObject *setTransformStatic(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  PyObject *py_transform;
  char *authority;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority))
    return NULL;

  geometry_msgs::TransformStamped transform;
  PyObject *header = PyObject_BorrowAttrString(py_transform, "header");
  transform.child_frame_id = PyString_AsString(PyObject_BorrowAttrString(py_transform, "child_frame_id"));
  transform.header.frame_id = PyString_AsString(PyObject_BorrowAttrString(header, "frame_id"));
  if (rostime_converter(PyObject_BorrowAttrString(header, "stamp"), &transform.header.stamp) != 1)
    return NULL;

  PyObject *mtransform = PyObject_BorrowAttrString(py_transform, "transform");
  PyObject *translation = PyObject_BorrowAttrString(mtransform, "translation");
  transform.transform.translation.x = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "x"));
  transform.transform.translation.y = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "y"));
  transform.transform.translation.z = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "z"));
  PyObject *rotation = PyObject_BorrowAttrString(mtransform, "rotation");
  transform.transform.rotation.x = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "x"));
  transform.transform.rotation.y = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "y"));
  transform.transform.rotation.z = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "z"));
  transform.transform.rotation.w = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "w"));

  // only difference to above is is_static == True
  bc->setTransform(transform, authority, true);
  Py_RETURN_NONE;
}

static PyObject *clear(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  bc->clear();
  Py_RETURN_NONE;
}

/* May come back eventually, but not in public api right now
static PyObject *frameExists(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *frame_id_str;
  if (!PyArg_ParseTuple(args, "s", &frame_id_str))
    return NULL;
  return PyBool_FromLong(t->frameExists(frame_id_str));
}

static PyObject *getFrameStrings(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  std::vector< std::string > ids;
  t->getFrameStrings(ids);
  return asListOfStrings(ids);
}
*/

static struct PyMethodDef buffer_core_methods[] =
{
  {"all_frames_as_yaml", allFramesAsYAML, METH_VARARGS},
  {"all_frames_as_string", allFramesAsString, METH_VARARGS},
  {"set_transform", setTransform, METH_VARARGS},
  {"set_transform_static", setTransformStatic, METH_VARARGS},
  {"can_transform_core", (PyCFunction)canTransformCore, METH_KEYWORDS},
  {"can_transform_full_core", (PyCFunction)canTransformFullCore, METH_KEYWORDS},
  //{"chain", (PyCFunction)chain, METH_KEYWORDS},
  {"clear", (PyCFunction)clear, METH_KEYWORDS},
  //{"frameExists", (PyCFunction)frameExists, METH_VARARGS},
  //{"getFrameStrings", (PyCFunction)getFrameStrings, METH_VARARGS},
  //{"getLatestCommonTime", (PyCFunction)getLatestCommonTime, METH_VARARGS},
  {"lookup_transform_core", (PyCFunction)lookupTransformCore, METH_KEYWORDS},
  {"lookup_transform_full_core", (PyCFunction)lookupTransformFullCore, METH_KEYWORDS},
  //{"lookupTwistCore", (PyCFunction)lookupTwistCore, METH_KEYWORDS},
  //{"lookupTwistFullCore", lookupTwistFullCore, METH_VARARGS},
  //{"getTFPrefix", (PyCFunction)getTFPrefix, METH_VARARGS},
  {NULL,          NULL}
};

static PyMethodDef module_methods[] = {
  // {"Transformer", mkTransformer, METH_VARARGS},
  {0, 0, 0},
};

extern "C" void init_tf2()
{
  PyObject *item, *m, *d;

#if PYTHON_API_VERSION >= 1007
  tf2_exception = PyErr_NewException((char*)"tf2.TransformException", NULL, NULL);
  tf2_connectivityexception = PyErr_NewException((char*)"tf2.ConnectivityException", tf2_exception, NULL);
  tf2_lookupexception = PyErr_NewException((char*)"tf2.LookupException", tf2_exception, NULL);
  tf2_extrapolationexception = PyErr_NewException((char*)"tf2.ExtrapolationException", tf2_exception, NULL);
  tf2_invalidargumentexception = PyErr_NewException((char*)"tf2.InvalidArgumentException", tf2_exception, NULL);
  tf2_timeoutexception = PyErr_NewException((char*)"tf2.TimeoutException", tf2_exception, NULL);
#else
  tf2_exception = PyString_FromString("tf2.error");
  tf2_connectivityexception = PyString_FromString("tf2.ConnectivityException");
  tf2_lookupexception = PyString_FromString("tf2.LookupException");
  tf2_extrapolationexception = PyString_FromString("tf2.ExtrapolationException");
  tf2_invalidargumentexception = PyString_FromString("tf2.InvalidArgumentException");
  tf2_timeoutexception = PyString_FromString("tf2.TimeoutException");
#endif

  pModulerospy = PyImport_Import(item= PyString_FromString("rospy")); Py_DECREF(item);
  pModulegeometrymsgs = PyImport_ImportModule("geometry_msgs.msg");

  if(pModulegeometrymsgs == NULL)
  {
    printf("Cannot load geometry_msgs module");
    return;
  }

  buffer_core_Type.tp_alloc = PyType_GenericAlloc;
  buffer_core_Type.tp_new = PyType_GenericNew;
  buffer_core_Type.tp_init = BufferCore_init;
  buffer_core_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  buffer_core_Type.tp_methods = buffer_core_methods;
  if (PyType_Ready(&buffer_core_Type) != 0)
    return;

  m = Py_InitModule("_tf2", module_methods);
  PyModule_AddObject(m, "BufferCore", (PyObject *)&buffer_core_Type);
  d = PyModule_GetDict(m);
  PyDict_SetItemString(d, "TransformException", tf2_exception);
  PyDict_SetItemString(d, "ConnectivityException", tf2_connectivityexception);
  PyDict_SetItemString(d, "LookupException", tf2_lookupexception);
  PyDict_SetItemString(d, "ExtrapolationException", tf2_extrapolationexception);
  PyDict_SetItemString(d, "InvalidArgumentException", tf2_invalidargumentexception);
  PyDict_SetItemString(d, "TimeoutException", tf2_timeoutexception);
}
