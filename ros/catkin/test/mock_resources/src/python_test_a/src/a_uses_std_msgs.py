# add build/gen/py to pythonpath before running
import std_msgs
import std_msgs.msg

some_string = std_msgs.msg.String
some_string.data = "Some Data in a std_msgs.msg.String"
print (some_string.data)
