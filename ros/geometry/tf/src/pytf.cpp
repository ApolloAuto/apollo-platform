/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <Python.h>

#include "tf/tf.h"

// Run x (a tf method, catching TF's exceptions and reraising them as Python exceptions)
//
#define WRAP(x) \
  do { \
  try \
  { \
    x; \
  }  \
  catch (const tf::ConnectivityException &e) \
  { \
    PyErr_SetString(tf_connectivityexception, e.what()); \
    return NULL; \
  } \
  catch (const tf::LookupException &e) \
  { \
    PyErr_SetString(tf_lookupexception, e.what()); \
    return NULL; \
  } \
  catch (const tf::ExtrapolationException &e) \
  { \
    PyErr_SetString(tf_extrapolationexception, e.what()); \
    return NULL; \
  } \
  } while (0)

static PyObject *pModulerospy = NULL;
static PyObject *tf_exception = NULL;
static PyObject *tf_connectivityexception = NULL, *tf_lookupexception = NULL, *tf_extrapolationexception = NULL;

struct transformer_t {
  PyObject_HEAD
  tf::Transformer *t;
};

static PyTypeObject transformer_Type = {
  PyObject_HEAD_INIT(&PyType_Type)
  0,                               /*size*/
  "_tf.Transformer",                /*name*/
  sizeof(transformer_t),           /*basicsize*/
};

static PyObject *PyObject_BorrowAttrString(PyObject* o, const char *name)
{
    PyObject *r = PyObject_GetAttrString(o, name);
    if (r != NULL)
      Py_DECREF(r);
    return r;
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

static int Transformer_init(PyObject *self, PyObject *args, PyObject *kw)
{
  int interpolating = 1;
  ros::Duration cache_time;

  cache_time.fromSec(tf::Transformer::DEFAULT_CACHE_TIME);

  if (!PyArg_ParseTuple(args, "|iO&", &interpolating, rosduration_converter, &cache_time))
    return -1;

  ((transformer_t*)self)->t = new tf::Transformer(interpolating, cache_time);
  ((transformer_t*)self)->t->fall_back_to_wall_time_ = true;

  return 0;
}

static PyObject *setUsingDedicatedThread(PyObject *self, PyObject *args)
{
  int value;
  if (!PyArg_ParseTuple(args, "i", &value))
    return NULL;
  tf::Transformer *t = ((transformer_t*)self)->t;
  t->setUsingDedicatedThread(value);
  return PyString_FromString(t->allFramesAsDot().c_str());
}

static PyObject *getTFPrefix(PyObject *self, PyObject *args)
{
  if (!PyArg_ParseTuple(args, ""))
    return NULL;
  tf::Transformer *t = ((transformer_t*)self)->t;
  return PyString_FromString(t->getTFPrefix().c_str());
}

static PyObject *allFramesAsDot(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  static const char *keywords[] = { "time", NULL };
  ros::Time time;
  if (!PyArg_ParseTupleAndKeywords(args, kw, "|O&", (char**)keywords, rostime_converter, &time))
    return NULL;
  return PyString_FromString(t->allFramesAsDot(time.toSec()).c_str());
}

static PyObject *allFramesAsString(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  return PyString_FromString(t->allFramesAsString().c_str());
}

static PyObject *canTransform(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame;
  ros::Time time;
  static const char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", (char**)keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  return PyBool_FromLong(t->canTransform(target_frame, source_frame, time));
}

static PyObject *canTransformFull(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
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
  return PyBool_FromLong(t->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
}

static PyObject *waitForTransform(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame;
  ros::Time time;
  ros::Duration timeout;
  ros::Duration polling_sleep_duration(0.01);
  std::string error_string;
  static const char *keywords[] = { "target_frame", "source_frame", "time", "timeout", "polling_sleep_duration", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&O&|O&", (char**)keywords,
    &target_frame,
    &source_frame,
    rostime_converter, &time,
    rosduration_converter, &timeout,
    rosduration_converter, &polling_sleep_duration))
    return NULL;
  bool r;
  Py_BEGIN_ALLOW_THREADS
  r = t->waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration, &error_string);
  Py_END_ALLOW_THREADS
  if (r == true) {
    Py_RETURN_NONE;
  } else {
    PyErr_SetString(tf_exception, error_string.c_str());
    return NULL;
  }
}

static PyObject *waitForTransformFull(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  ros::Duration timeout;
  ros::Duration polling_sleep_duration(0.01);
  std::string error_string;
  static const char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", "timeout", "polling_sleep_duration", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&sO&|O&", (char**)keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame,
                        rosduration_converter, &timeout,
                        rosduration_converter, &polling_sleep_duration))
    return NULL;
  int r;
  Py_BEGIN_ALLOW_THREADS
  r = t->waitForTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, polling_sleep_duration, &error_string);
  Py_END_ALLOW_THREADS
  if (r == true) {
    Py_RETURN_NONE;
  } else {
    PyErr_SetString(tf_exception, error_string.c_str());
    return NULL;
  }
}

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

static PyObject *lookupTransform(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame;
  ros::Time time;
  static const char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", (char**)keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  tf::StampedTransform transform;
  WRAP(t->lookupTransform(target_frame, source_frame, time, transform));
  tf::Vector3 origin = transform.getOrigin();
  tf::Quaternion rotation = transform.getRotation();
  return Py_BuildValue("(ddd)(dddd)",
      origin.x(), origin.y(), origin.z(),
      rotation.x(), rotation.y(), rotation.z(), rotation.w());
}

static PyObject *lookupTransformFull(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
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
  tf::StampedTransform transform;
  WRAP(t->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform));
  tf::Vector3 origin = transform.getOrigin();
  tf::Quaternion rotation = transform.getRotation();
  return Py_BuildValue("(ddd)(dddd)",
      origin.x(), origin.y(), origin.z(),
      rotation.x(), rotation.y(), rotation.z(), rotation.w());
}

static PyObject *lookupTwist(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *tracking_frame, *observation_frame;
  ros::Time time;
  ros::Duration averaging_interval;
  static const char *keywords[] = { "tracking_frame", "observation_frame", "time", "averaging_interval", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&O&", (char**)keywords, &tracking_frame, &observation_frame, rostime_converter, &time, rosduration_converter, &averaging_interval))
    return NULL;
  geometry_msgs::Twist twist;
  WRAP(t->lookupTwist(tracking_frame, observation_frame, time, averaging_interval, twist));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}

static PyObject *lookupTwistFull(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
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
  WRAP(t->lookupTwist(tracking_frame, observation_frame, reference_frame, pt, reference_point_frame, time, averaging_interval, twist));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}

static PyObject *setTransform(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  PyObject *py_transform;
  char *authority = (char*)"default_authority";

  if (!PyArg_ParseTuple(args, "O|s", &py_transform, &authority))
    return NULL;
  tf::StampedTransform transform;
  PyObject *header = PyObject_BorrowAttrString(py_transform, "header");
  transform.child_frame_id_ = PyString_AsString(PyObject_BorrowAttrString(py_transform, "child_frame_id"));
  transform.frame_id_ = PyString_AsString(PyObject_BorrowAttrString(header, "frame_id"));
  if (rostime_converter(PyObject_BorrowAttrString(header, "stamp"), &transform.stamp_) != 1)
    return NULL;

  PyObject *mtransform = PyObject_BorrowAttrString(py_transform, "transform");
  PyObject *translation = PyObject_BorrowAttrString(mtransform, "translation");
  double tx = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "x"));
  double ty = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "y"));
  double tz = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "z"));
  PyObject *rotation = PyObject_BorrowAttrString(mtransform, "rotation");
  double qx = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "x"));
  double qy = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "y"));
  double qz = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "z"));
  double qw = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "w"));

  transform.setData(tf::Transform(
    tf::Quaternion(tfScalar(qx), tfScalar(qy), tfScalar(qz), tfScalar(qw)),
    tf::Vector3(tfScalar(tx), tfScalar(ty), tfScalar(tz))));
  t->setTransform(transform, authority);
  Py_RETURN_NONE;
}

static PyObject *clear(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  t->clear();
  Py_RETURN_NONE;
}

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

static struct PyMethodDef transformer_methods[] =
{
  {"allFramesAsDot", (PyCFunction)allFramesAsDot, METH_KEYWORDS},
  {"allFramesAsString", allFramesAsString, METH_VARARGS},
  {"setTransform", setTransform, METH_VARARGS},
  {"canTransform", (PyCFunction)canTransform, METH_KEYWORDS},
  {"canTransformFull", (PyCFunction)canTransformFull, METH_KEYWORDS},
  {"waitForTransform", (PyCFunction)waitForTransform, METH_KEYWORDS},
  {"waitForTransformFull", (PyCFunction)waitForTransformFull, METH_KEYWORDS},
  {"chain", (PyCFunction)chain, METH_KEYWORDS},
  {"clear", (PyCFunction)clear, METH_KEYWORDS},
  {"frameExists", (PyCFunction)frameExists, METH_VARARGS},
  {"getFrameStrings", (PyCFunction)getFrameStrings, METH_VARARGS},
  {"getLatestCommonTime", (PyCFunction)getLatestCommonTime, METH_VARARGS},
  {"lookupTransform", (PyCFunction)lookupTransform, METH_KEYWORDS},
  {"lookupTransformFull", (PyCFunction)lookupTransformFull, METH_KEYWORDS},
  {"lookupTwist", (PyCFunction)lookupTwist, METH_KEYWORDS},
  {"lookupTwistFull", lookupTwistFull, METH_VARARGS},
  {"setUsingDedicatedThread", (PyCFunction)setUsingDedicatedThread, METH_VARARGS},
  {"getTFPrefix", (PyCFunction)getTFPrefix, METH_VARARGS},
  {NULL,          NULL}
};

static PyMethodDef module_methods[] = {
  // {"Transformer", mkTransformer, METH_VARARGS},
  {NULL, NULL, NULL},
};

extern "C" void init_tf()
{
  PyObject *item, *m, *d;

#if PYTHON_API_VERSION >= 1007
  tf_exception = PyErr_NewException((char*)"tf.Exception", NULL, NULL);
  tf_connectivityexception = PyErr_NewException((char*)"tf.ConnectivityException", tf_exception, NULL);
  tf_lookupexception = PyErr_NewException((char*)"tf.LookupException", tf_exception, NULL);
  tf_extrapolationexception = PyErr_NewException((char*)"tf.ExtrapolationException", tf_exception, NULL);
#else
  tf_exception = PyString_FromString("tf.error");
  tf_connectivityexception = PyString_FromString("tf.ConnectivityException");
  tf_lookupexception = PyString_FromString("tf.LookupException");
  tf_extrapolationexception = PyString_FromString("tf.ExtrapolationException");
#endif

  pModulerospy = PyImport_Import(item= PyString_FromString("rospy")); Py_DECREF(item);

  transformer_Type.tp_alloc = PyType_GenericAlloc;
  transformer_Type.tp_new = PyType_GenericNew;
  transformer_Type.tp_init = Transformer_init;
  transformer_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  transformer_Type.tp_methods = transformer_methods;
  if (PyType_Ready(&transformer_Type) != 0)
    return;

  m = Py_InitModule("_tf", module_methods);
  PyModule_AddObject(m, "Transformer", (PyObject *)&transformer_Type);
  d = PyModule_GetDict(m);
  PyDict_SetItemString(d, "Exception", tf_exception);
  PyDict_SetItemString(d, "ConnectivityException", tf_connectivityexception);
  PyDict_SetItemString(d, "LookupException", tf_lookupexception);
  PyDict_SetItemString(d, "ExtrapolationException", tf_extrapolationexception);
}
