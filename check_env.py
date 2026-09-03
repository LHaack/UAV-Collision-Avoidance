
import sys
import os

print(f"Python Executable: {sys.executable}")
print(f"User: {os.environ.get('USER')}")
print("sys.path:")
for p in sys.path:
    print(f"  {p}")

print("-" * 20)
print("Attempting to import numpy...")
try:
    import numpy
    print(f"SUCCESS: numpy version {numpy.__version__} at {numpy.__file__}")
except ImportError as e:
    print(f"FAILURE: {e}")

print("-" * 20)
print("Attempting to import sensor_msgs.msg...")
try:
    import sensor_msgs.msg
    print(f"SUCCESS: sensor_msgs.msg imported from {sensor_msgs.msg.__file__}")
except ImportError as e:
    print(f"FAILURE: {e}")
